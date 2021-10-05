#!/usr/bin/env python

# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_service_caller/src/rqt_service_caller/service_caller_widget.py
import math
import random
import time

import genpy
import rospy
import rosservice
from opcua import ua, uamethod, common

import ros_server


class OpcUaROSService:
    def __init__(self, server, parent, idx, service_name, service_class):
        self.server = server
        self.name = service_name
        self.parent = self.recursive_create_objects(service_name, idx, parent)
        self._class = service_class
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
        inputs = getargarray(self.sample_req)
        self.outputs = getargarray(self.sample_resp)
        self.method = self.parent.add_method(idx, self.name, self.call_service, inputs, self.outputs)
        rospy.loginfo("Created ROS Service with name: %s", self.name)

    @uamethod
    def call_service(self, parent, *inputs):
        try:
            rospy.loginfo("Calling Service with name: " + self.name)
            input_msg = self.create_message_instance(inputs, self.sample_req)
            rospy.logdebug("Created Input Request for Service " + self.name + " : " + str(input_msg))
            response = self.proxy(input_msg)
            rospy.logdebug("got response: " + str(response))
            rospy.logdebug("Creating response message object")
            return_values = []
            for slot in response.__slots__:
                rospy.logdebug("Converting slot: " + str(getattr(response, slot)))
                return_values.append(getattr(response, slot))
                rospy.logdebug("Current Response list: " + str(return_values))
            return return_values
        except (TypeError, rospy.ROSException, rospy.ROSInternalException, rospy.ROSSerializationException,
                common.uaerrors.UaError, rosservice.ROSServiceException) as e:
            rospy.logerr("Error when calling service " + self.name, e)

    def create_message_instance(self, inputs, sample):
        rospy.logdebug("Creating message for goal call")
        already_set = []
        if isinstance(inputs, tuple):
            arg_counter = 0
            object_counter = 0
            while (arg_counter < len(inputs) and object_counter < len(sample.__slots__)):
                cur_arg = inputs[arg_counter]
                cur_slot = sample.__slots__[object_counter]
                real_slot = getattr(sample, cur_slot)
                rospy.logdebug(
                    "cur_arg: " + str(cur_arg) + " cur_slot_name: " + str(cur_slot) + " real slot content: " + str(
                        real_slot))
                if hasattr(real_slot, '_type'):
                    rospy.logdebug("We found an object with name " + str(cur_slot) + ", creating it recursively")
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

    def create_object_instance(self, already_set, object, name, counter, inputs, sample):
        rospy.loginfo("Create Object Instance Notify")
        object_counter = 0
        while (object_counter < len(object.__slots__) and counter < len(inputs)):
            cur_arg = inputs[counter]
            cur_slot = object.__slots__[object_counter]
            real_slot = getattr(object, cur_slot)
            rospy.loginfo(
                "cur_arg: " + str(cur_arg) + " cur_slot_name: " + str(cur_slot) + " real slot content: " + str(
                    real_slot))
            if hasattr(real_slot, '_type'):
                rospy.logdebug("Recursive Object found in request/response of service call")
                already_set, counter = self.create_object_instance(already_set, real_slot, cur_slot, counter, inputs,
                                                                   sample)
                object_counter += 1
            else:
                already_set.append(cur_slot)
                setattr(object, cur_slot, cur_arg)
                object_counter += 1
                counter += 1
                # sets the object as an attribute in the request were trying to build
        setattr(sample, name, object)
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
        hierachy = name.split('/')
        if len(hierachy) == 0 or len(hierachy) == 1:
            return parent
        for name in hierachy:
            if name != '':
                try:
                    nodewithsamename = self.server.find_service_node_with_same_name(name, idx)
                    rospy.logdebug("nodewithsamename for name: " + str(name) + " is : " + str(nodewithsamename))
                    if nodewithsamename is not None:
                        rospy.logdebug("recursive call for same name for: " + name)
                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx,
                                                             nodewithsamename)
                    else:
                        newparent = parent.add_object(
                            ua.NodeId(name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                            ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx,
                                                             newparent)
                except (IndexError, common.uaerrors.UaError) as e:
                    newparent = parent.add_object(
                        ua.NodeId(name + str(random.randint(0, 10000)), parent.nodeid.NamespaceIndex,
                                  ua.NodeIdType.String),
                        ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                    return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx,
                                                         newparent)
        return parent


def getargarray(sample_req):
    array = []
    for slot_name in sample_req.__slots__:
        slot = getattr(sample_req, slot_name)
        if hasattr(slot, '_type'):
            array_to_merge = getargarray(slot)
            array.extend(array_to_merge)
        else:
            if isinstance(slot, list):
                arg = ua.Argument()
                arg.Name = slot_name
                arg.DataType = ua.NodeId(getobjectidfromtype("array"))
                arg.ValueRank = -1
                arg.ArrayDimensions = [1]
                arg.Description = ua.LocalizedText("Array")
            else:
                arg = ua.Argument()
                arg.Name = slot_name
                arg.DataType = ua.NodeId(getobjectidfromtype(type(slot).__name__))
                arg.ValueRank = -1
                arg.ArrayDimensions = []
                arg.Description = ua.LocalizedText(slot_name)
            array.append(arg)

    return array


def refresh_services(namespace_ros, server, servicesdict, idx, services_object_opc):
    rosservices = rosservice.get_service_list(namespace=namespace_ros)

    for service_name_ros in rosservices:
        try:
            if service_name_ros not in servicesdict or servicesdict[service_name_ros] is None:
                service = OpcUaROSService(server, services_object_opc, idx, service_name_ros,
                                          rosservice.get_service_class_by_name(service_name_ros))
                servicesdict[service_name_ros] = service
        except (rosservice.ROSServiceException, rosservice.ROSServiceIOException) as e:
            try:
                rospy.logerr("Error when trying to refresh services", e)
            except TypeError as e2:
                rospy.logerr("Error when logging an Exception, can't convert everything to string")
    # use extra iteration as to not get "dict changed during iteration" errors
    tobedeleted = []
    rosservices = rosservice.get_service_list()
    for service_nameOPC in servicesdict:
        found = False
        for rosservice_name in rosservices:
            if service_nameOPC == rosservice_name:
                found = True
        if not found and servicesdict[service_nameOPC] is not None:
            servicesdict[service_nameOPC].recursive_delete_items(
                server.server.get_node(ua.NodeId(service_nameOPC, idx)))
            tobedeleted.append(service_nameOPC)
        if len(servicesdict[service_nameOPC].parent.get_children()) == 0:
            server.server.delete_nodes([servicesdict[service_nameOPC].parent])
    for name in tobedeleted:
        del servicesdict[name]


def getobjectidfromtype(type_name):
    if type_name == 'bool':
        dv = ua.ObjectIds.Boolean
    elif type_name == 'byte':
        dv = ua.ObjectIds.Byte
    elif type_name == 'int':
        dv = ua.ObjectIds.Int16
    elif type_name == 'int8':
        dv = ua.ObjectIds.SByte
    elif type_name == 'uint8':
        dv = ua.ObjectIds.Byte
    elif type_name == 'int16':
        dv = ua.ObjectIds.Int16
        rospy.roswarn("Int16??")
    elif type_name == 'uint16':
        dv = ua.ObjectIds.UInt16
    elif type_name == 'int32':
        dv = ua.ObjectIds.Int32
    elif type_name == 'uint32':
        dv = ua.ObjectIds.UInt32
    elif type_name == 'int64':
        dv = ua.ObjectIds.Int64
    elif type_name == 'uint64':
        dv = ua.ObjectIds.UInt64
    elif type_name == 'float' or type_name == 'float32' or type_name == 'float64':
        dv = ua.ObjectIds.Float
    elif type_name == 'double':
        dv = ua.ObjectIds.Double
    elif type_name == 'string' or type_name == 'str':
        dv = ua.ObjectIds.String
    elif type_name == 'array':
        dv = ua.ObjectIds.Enumeration
    elif type_name == 'Time' or type_name == 'time':
        dv = ua.ObjectIds.Time
    else:
        rospy.logerr("Can't create type with name " + type_name)
        return None
    return dv
