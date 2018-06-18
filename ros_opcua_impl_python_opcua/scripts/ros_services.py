# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_service_caller/src/rqt_service_caller/service_caller_widget.py
import random

import rosservice

from opcua import uamethod
from opcua.ua.uaerrors import UaError

from ros_global import *
from ros_opc_ua import *


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
        if parent.get_node_class() != ua.NodeClass.Object:
            return None
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
                        next_name(hierarchy, hierarchy.index(name)),
                        idx,
                        node_with_same_name)
                else:
                    new_parent = parent.add_object(
                        ua.NodeId(name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                        ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                    return self.recursive_create_objects(
                        next_name(hierarchy, hierarchy.index(name)),
                        idx,
                        new_parent)
            except (IndexError, ua.UaError) as e:
                print('Error' + e.message)
                new_parent = parent.add_object(
                    ua.NodeId(name + str(random.randint(0, 10000)), parent.nodeid.NamespaceIndex,
                              ua.NodeIdType.String),
                    ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                return self.recursive_create_objects(
                    next_name(hierarchy, hierarchy.index(name)), idx,
                    new_parent)
        return parent


class OpcUaROSServiceNew:

    def __init__(self, server, idx, created_data_types, created_variable_types):
        self._server = server
        self._idx = idx

        self._data_types = created_data_types
        self._variable_types = created_variable_types
        self.created_object_types = {}

        ot_base_node = self._server.nodes.base_object_type
        self._ot_root = ot_base_node.add_object_type(nodeid_generator(self._idx),
                                                     ua.QualifiedName('ROSServiceType', self._idx),
                                                     is_abstract=True)

    @uamethod
    def _call_service(self, parent, *inputs):
        parent_node = self._server.get_node(parent)
        # Method in a not instantiated node should not be called
        if parent_node.get_node_class() != ua.NodeClass.Object:
            return None
        service_name = parent_node.get_properties()[0].get_value()
        service_type = parent_node.get_references(ua.ObjectIds.HasTypeDefinition)[0]
        rospy.logdebug('Parent of the method is: ' + parent.to_string())
        result = ua.CallMethodResult()
        try:
            rospy.loginfo('Calling Service with name: ' + service_name)
            service_class = get_service_class(service_type)
            service_req = getattr(service_class, '_request_class')
            service_resp = getattr(service_class, '_request_class')
            input_msg = service_req(*inputs)
            rospy.logdebug('Created Input Request for Service %s : %s' % (service_name, str(input_msg)))
            service_proxy = rospy.ServiceProxy(service_name, service_class)
            response = service_proxy(input_msg)
            rospy.logdebug('Calling service successful, got response: ' + str(response))

            result.StatusCode = ua.StatusCodes.Good

            # TODO: how to return a UA Object???
            for slot in response.__slots__:
                rospy.logdebug('Converting slot: ' + str(getattr(response, slot)))
                result.OutputArguments.append(getattr(response, slot))

        except (TypeError, rospy.ROSException, rospy.ROSInternalException, rospy.ROSSerializationException,
                UaError, rosservice.ROSServiceException) as e:
            rospy.logerr('Error when calling service ' + service_name, e)
            result.StatusCode = ua.StatusCodes.Bad
        finally:
            return result

    def _get_args(self, arg_class):
        args = []
        for slot_name, type_name in zip(arg_class.__slots__, getattr(arg_class, '_slot_types')):
            arg = ua.Argument()
            arg.Name = slot_name
            arg.Description = ua.LocalizedText(slot_name)
            base_type_str, array_size = extract_array_info(type_name)
            process_ros_array(array_size, arg)
            if base_type_str in ROS_BUILD_IN_DATA_TYPES.keys():
                arg.DataType = ROS_BUILD_IN_DATA_TYPES[base_type_str]
            elif base_type_str in self._data_types.keys():
                arg.DataType = self._data_types[base_type_str].nodeid
            else:
                raise rospy.ROSException
            args.append(arg)
        return args

    def _create_service(self, service):
        service_class = get_service_class(service)
        service_node = self._ot_root.add_object_type(nodeid_generator(self._idx),
                                                     ua.QualifiedName(service, self._idx))
        service_node.add_property(nodeid_generator(self._idx),
                                  ua.QualifiedName('ROS service name', self._idx), '')
        service_req = getattr(service_class, '_request_class')
        input_args = self._get_args(service_req)
        service_resp = getattr(service_class, '_response_class')
        output_args = self._get_args(service_resp)
        new_node = service_node.add_method(nodeid_generator(self._idx),
                                           ua.QualifiedName('ROS service call', self._idx),
                                           self._call_service, input_args, output_args)
        self.created_object_types[service] = new_node

    def create_services(self):
        for srv in get_ros_services():
            self._create_service(srv)


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
                arg.DataType = ua.NodeId(get_object_ids('array'))
                arg.ValueRank = 1
                arg.ArrayDimensions = [1]
                arg.Description = ua.LocalizedText('Array')
            else:
                arg = ua.Argument()
                arg.Name = slot_name
                arg.DataType = ua.NodeId(get_object_ids(type(slot).__name__))
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

    rosnode_cleanup()
