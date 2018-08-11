# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_service_caller/src/rqt_service_caller/service_caller_widget.py

import rosservice

from opcua import uamethod

from ros_global import *
from ros_opc_ua import get_ua_class, ua_class_to_ros_msg, ros_msg_to_ua_class


class OpcUaROSService:

    def __init__(self, service_name, node_root, service_node_id, msg_dict):
        self._service_class = rosservice.get_service_class_by_name(service_name)
        self._service_name = service_name
        self.proxy = rospy.ServiceProxy(service_name, self._service_class)
        self._node_root = node_root

        self._ros_service_req = getattr(self._service_class, '_request_class')
        self._req_name = getattr(self._ros_service_req, '_type')
        self._ros_service_resp = getattr(self._service_class, '_response_class')
        self._resp_name = getattr(self._ros_service_resp, '_type')

        in_dt_node_id = msg_dict[self._req_name]
        out_dt_node_id = msg_dict[self._resp_name]
        input_arg = self._create_args(self._ros_service_req, in_dt_node_id)
        output_arg = self._create_args(self._ros_service_resp, out_dt_node_id)

        node_root.add_method(service_node_id, service_name, self._call_service, input_arg, output_arg)
        rospy.loginfo('Created ROS Service with name: ' + service_name)

    @staticmethod
    def _create_args(msg_class, data_type):
        """one extension object contains all info"""
        if not len(getattr(msg_class, '__slots__')):
            return []
        msg_class_name = getattr(msg_class, '_type')
        arg = ua.Argument()
        arg.Name = msg_class_name
        arg.DataType = data_type
        arg.ValueRank = -1
        arg.ArrayDimensions = []
        arg.Description = ua.LocalizedText(msg_class_name)
        return [arg]

    @uamethod
    def _call_service(self, parent, *inputs):
        rospy.loginfo('Calling service %s under ROS node: %s, %s'
                      % (self._service_name, self._node_root.get_display_name().Text, parent.to_string()))
        try:
            response = self.proxy(ua_class_to_ros_msg(inputs[0], self._ros_service_req()))
            rospy.loginfo('Calling service succeeded!')
            return ua.Variant(ros_msg_to_ua_class(response, get_ua_class(self._resp_name)()))
        except Exception as e:
            rospy.logerr('Error when calling service ' + self._service_name, e)


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
