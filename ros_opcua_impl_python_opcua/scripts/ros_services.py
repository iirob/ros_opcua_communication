# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_service_caller/src/rqt_service_caller/service_caller_widget.py
import random

import rospy
import rosservice

from opcua import ua, uamethod
from opcua.ua.uaerrors import UaError

import ros_global


class OpcUaROSService:

    def __init__(self, server, parent, idx, service_name):
        self._server = server
        self._name = service_name
        self._idx = idx
        self.parent = self.recursive_create_objects(self._name, self._idx, parent)
        self._service_class = rosservice.get_service_class_by_name(self._name)
        self._proxy = rospy.ServiceProxy(self._name, self._service_class)
        self._nodes = {}

        # Build the Array of inputs
        self._service_req = getattr(self._service_class, '_request_class')()
        self._service_resp = getattr(self._service_class, '_response_class')()
        inputs = get_args(self._service_req)
        outputs = get_args(self._service_resp)
        self.method = self.parent.add_method(self._idx, self._name, self._call_service, inputs, outputs)
        rospy.loginfo('Created ROS Service with name: ' + self._name)

    @uamethod
    def _call_service(self, parent, *inputs):
        rospy.logdebug('Parent of the method is: ' + parent.to_string())
        try:
            rospy.loginfo('Calling Service with name: ' + self._name)
            # TODO: Refactor here after the get_args function is deleted
            input_msg = type(self._service_req)(*inputs)
            rospy.logdebug('Created Input Request for Service %s : %s' % (self._name, str(input_msg)))
            response = self._proxy(input_msg)
            rospy.logdebug('got response: ' + str(response))
            rospy.logdebug('Creating response message object')
            return_values = []
            for slot in response.__slots__:
                rospy.logdebug('Converting slot: ' + str(getattr(response, slot)))
                return_values.append(getattr(response, slot))
                rospy.logdebug('Current Response list: ' + str(return_values))
            if len(return_values) == 0:
                return None
            return return_values
        except (TypeError, rospy.ROSException, rospy.ROSInternalException, rospy.ROSSerializationException,
                UaError, rosservice.ROSServiceException) as e:
            rospy.logerr('Error when calling service ' + self._name, e)

    def recursive_create_objects(self, name, idx, parent):
        hierarchy = [element for element in name.split('/') if element]
        if len(hierarchy) <= 1:
            return parent
        for name in hierarchy:
            try:
                node_with_same_name = self._server.find_service_node_with_same_name(name)
                rospy.logdebug('node_with_same_name for name: ' + str(name) + ' is : ' + str(node_with_same_name))
                if node_with_same_name is not None:
                    rospy.logdebug('recursive call for same name for: ' + name)
                    return self.recursive_create_objects(
                        ros_global.next_name(hierarchy, hierarchy.index(name)),
                        idx,
                        node_with_same_name)
                else:
                    new_parent = parent.add_object(
                        ua.NodeId(name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                        ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                    return self.recursive_create_objects(
                        ros_global.next_name(hierarchy, hierarchy.index(name)),
                        idx,
                        new_parent)
            except (IndexError, ua.UaError) as e:
                print('Error' + e.message)
                new_parent = parent.add_object(
                    ua.NodeId(name + str(random.randint(0, 10000)), parent.nodeid.NamespaceIndex,
                              ua.NodeIdType.String),
                    ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                return self.recursive_create_objects(
                    ros_global.next_name(hierarchy, hierarchy.index(name)), idx,
                    new_parent)
        return parent


def get_args(arg_class):
    array = []
    for slot_name, type_name_child in zip(arg_class.__slots__, getattr(arg_class, '_slot_types')):
        slot = getattr(arg_class, slot_name)
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
            if service_name_ros not in service_dict.keys():
                service = OpcUaROSService(server, services_object_opc, idx, service_name_ros)
                service_dict[service_name_ros] = service
        except (rosservice.ROSServiceException, rosservice.ROSServiceIOException) as e:
            rospy.logerr('Error when trying to refresh services', e)

    remove_inactive_services(server, service_dict)


def remove_inactive_services(server, service_dict):
    # TODO: the to be deleted methods can be cached in maybe another list, to accelerate the speed of creating services
    # TODO: Reorganize the objects in address space so that the ros_node can be deleted recursively
    ros_services = rosservice.get_service_list()
    delete_targets = []
    for service_nameOPC in service_dict.keys():
        if service_nameOPC not in ros_services:
            if len(service_dict[service_nameOPC].parent.get_children()) <= 1:
                target_node = service_dict[service_nameOPC].parent
            else:
                target_node = service_dict[service_nameOPC].method
            delete_targets.append(target_node)
            del service_dict[service_nameOPC]
    server.server.delete_nodes(delete_targets, recursive=True)

    ros_global.rosnode_cleanup()
