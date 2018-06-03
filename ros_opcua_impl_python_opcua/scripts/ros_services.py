#!/usr/bin/env python

# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_service_caller/src/rqt_service_caller/service_caller_widget.py
import math
import random
import time

import genpy
import rospy
import rosservice
from opcua import ua, uamethod
from opcua.ua.uaerrors import UaError

import ros_server
import ros_global


class OpcUaROSService:

    def __init__(self, server, parent, idx, service_name):
        self.server = server
        self.name = service_name
        self.parent = self.recursive_create_objects(service_name, idx, parent)
        self._class = rosservice.get_service_class_by_name(self.name)
        self.proxy = rospy.ServiceProxy(self.name, self._class)
        self.counter = 0
        self._nodes = {}
        self.expressions = {}
        self._eval_locals = {}

        for module in (math, random, time):
            self._eval_locals.update(module.__dict__)
        self._eval_locals['genpy'] = genpy
        del self._eval_locals['__name__']
        del self._eval_locals['__doc__']
        # Build the Array of inputs
        self.sample_req = self._class._request_class()
        self.sample_resp = self._class._response_class()
        inputs = get_args(self.sample_req)
        self.outputs = get_args(self.sample_resp)
        self.method = self.parent.add_method(idx, self.name, self.call_service, inputs, self.outputs)
        rospy.loginfo('Created ROS Service with name: %s' % self.name)

    @uamethod
    def call_service(self, parent, *inputs):
        try:
            rospy.loginfo('Calling Service with name: ' + self.name)
            input_msg = self.create_message_instance(inputs, self.sample_req)
            rospy.logdebug('Created Input Request for Service %s : %s' % (self.name, str(input_msg)))
            response = self.proxy(input_msg)
            rospy.logdebug('got response: ' + str(response))
            rospy.logdebug('Creating response message object')
            return_values = []
            for slot in response.__slots__:
                rospy.logdebug('Converting slot: ' + str(getattr(response, slot)))
                return_values.append(getattr(response, slot))
                rospy.logdebug('Current Response list: ' + str(return_values))
            return return_values
        except (TypeError, rospy.ROSException, rospy.ROSInternalException, rospy.ROSSerializationException,
                UaError, rosservice.ROSServiceException) as e:
            rospy.logerr('Error when calling service ' + self.name, e)

    def create_message_instance(self, inputs, sample):
        rospy.logdebug('Creating message for goal call')
        already_set = []
        if isinstance(inputs, tuple):
            arg_counter = 0
            object_counter = 0
            while arg_counter < len(inputs) and object_counter < len(sample.__slots__):
                cur_arg = inputs[arg_counter]
                cur_slot = sample.__slots__[object_counter]
                real_slot = getattr(sample, cur_slot)
                rospy.logdebug(
                    'cur_arg: ' + str(cur_arg) + ' cur_slot_name: ' + str(cur_slot) + ' real slot content: ' + str(
                        real_slot))
                if hasattr(real_slot, '_type'):
                    rospy.logdebug('We found an object with name ' + str(cur_slot) + ', creating it recursively')
                    already_set, arg_counter = self.create_object_instance(already_set, real_slot, cur_slot,
                                                                           arg_counter, inputs, sample)
                    object_counter += 1
                else:
                    already_set.append(cur_slot)
                    # set the attribute in the request
                    setattr(sample, cur_slot, cur_arg)
                    arg_counter += 1
                    object_counter += 1

        return sample

    def create_object_instance(self, already_set, obj, name, counter, inputs, sample):
        rospy.loginfo('Create Object Instance Notify')
        object_counter = 0
        while object_counter < len(obj.__slots__) and counter < len(inputs):
            cur_arg = inputs[counter]
            cur_slot = obj.__slots__[object_counter]
            real_slot = getattr(obj, cur_slot)
            rospy.loginfo(
                'cur_arg: ' + str(cur_arg) + ' cur_slot_name: ' + str(cur_slot) + ' real slot content: ' + str(
                    real_slot))
            if hasattr(real_slot, '_type'):
                rospy.logdebug('Recursive Object found in request/response of service call')
                already_set, counter = self.create_object_instance(already_set, real_slot, cur_slot, counter, inputs,
                                                                   sample)
                object_counter += 1
            else:
                already_set.append(cur_slot)
                setattr(obj, cur_slot, cur_arg)
                object_counter += 1
                counter += 1
                # sets the object as an attribute in the request were trying to build
        setattr(sample, name, obj)
        return already_set, counter

    def recursive_delete_items(self, item):
        self.proxy.close()
        for child in item.get_children():
            self.recursive_delete_items(child)
            if child in self._nodes:
                del self._nodes[child]
            self.server.server.delete_nodes([child])
        self.server.server.delete_nodes([self.method])
        ros_server.own_rosnode_cleanup()

    def recursive_create_objects(self, name, idx, parent):
        hierarchy = name.split('/')
        if len(hierarchy) in (0, 1):
            return parent
        for name in hierarchy:
            if name != '':
                try:
                    node_with_same_name = self.server.find_service_node_with_same_name(name)
                    rospy.logdebug('node_with_same_name for name: ' + str(name) + ' is : ' + str(node_with_same_name))
                    if node_with_same_name is not None:
                        rospy.logdebug('recursive call for same name for: ' + name)
                        return self.recursive_create_objects(ros_server.next_name(hierarchy, hierarchy.index(name)),
                                                             idx,
                                                             node_with_same_name)
                    else:
                        new_parent = parent.add_object(
                            ua.NodeId(name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                            ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                        return self.recursive_create_objects(ros_server.next_name(hierarchy, hierarchy.index(name)),
                                                             idx,
                                                             new_parent)
                except IndexError, ua.UaError:
                    new_parent = parent.add_object(
                        ua.NodeId(name + str(random.randint(0, 10000)), parent.nodeid.NamespaceIndex,
                                  ua.NodeIdType.String),
                        ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                    return self.recursive_create_objects(ros_server.next_name(hierarchy, hierarchy.index(name)), idx,
                                                         new_parent)
        return parent


def get_args(sample_req):
    array = []
    for slot_name, type_name_child in zip(sample_req.__slots__, sample_req.__getattribute__('_slot_types')):
        slot = getattr(sample_req, slot_name)
        if hasattr(slot, '_type'):  # clearly not needed
            array_to_merge = get_args(slot)
            array.extend(array_to_merge)
        else:
            if isinstance(slot, list):
                arg = ua.Argument()
                arg.Name = slot_name
                arg.DataType = ua.NodeId(ros_global.get_object_ids('array'))
                arg.ValueRank = 1
                arg.ArrayDimensions = [1]
                arg.Description = ua.LocalizedText('Array')
            else:
                arg = ua.Argument()
                arg.Name = slot_name
                arg.DataType = ua.NodeId(ros_global.get_object_ids(type(slot).__name__))
                arg.ValueRank = -1
                arg.ArrayDimensions = []
                arg.Description = ua.LocalizedText(slot_name)
            array.append(arg)

    return array


def refresh_services(namespace_ros, server, service_dict, idx, services_object_opc):
    ros_services = rosservice.get_service_list(namespace=namespace_ros)

    for service_name_ros in ros_services:
        try:
            if service_name_ros not in service_dict or service_dict[service_name_ros] is None:
                service = OpcUaROSService(server, services_object_opc, idx, service_name_ros)
                service_dict[service_name_ros] = service
        except (rosservice.ROSServiceException, rosservice.ROSServiceIOException) as e:
            try:
                rospy.logerr('Error when trying to refresh services', e)
            except TypeError as e2:
                rospy.logerr('Error when logging an Exception, can not convert everything to string', e2)

    remove_inactive_services(server, service_dict, idx)


def remove_inactive_services(server, service_dict, idx):
    # TODO: refactor the iteration using dict.keys() or dict.items(), so that no dict changed error happens
    # and the structure can be trimmed thinner
    # TODO: the to be deleted methods can be cached in maybe another list, to accelerate the speed of creating services
    to_be_deleted = []
    ros_services = rosservice.get_service_list()
    for service_nameOPC in service_dict:
        found = False
        for ros_service in ros_services:
            if service_nameOPC == ros_service:
                found = True
        if not found and service_dict[service_nameOPC] is not None:
            service_dict[service_nameOPC].recursive_delete_items(
                server.server.get_node(ua.NodeId(service_nameOPC, idx)))
            to_be_deleted.append(service_nameOPC)
        if len(service_dict[service_nameOPC].parent.get_children()) == 0:
            server.server.delete_nodes([service_dict[service_nameOPC].parent])
    for name in to_be_deleted:
        del service_dict[name]
