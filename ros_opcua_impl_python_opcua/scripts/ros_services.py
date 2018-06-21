# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_service_caller/src/rqt_service_caller/service_caller_widget.py

import rosservice

from opcua import uamethod
from opcua.ua.uaerrors import UaError

from ros_global import *
from ros_opc_ua import to_camel_case


class OpcUaROSService:

    def __init__(self, service_name, node_root, service_node_id, in_dt_node_id, out_dt_node_id):
        self._service_class = rosservice.get_service_class_by_name(service_name)
        self._service_name = service_name
        self.proxy = rospy.ServiceProxy(service_name, self._service_class)
        self._node_root = node_root

        # Build the Array of inputs
        self._ros_service_req = getattr(self._service_class, '_request_class')
        self._ros_service_resp = getattr(self._service_class, '_response_class')
        input_arg = self._create_args(getattr(self._ros_service_req, '_type'), in_dt_node_id)
        output_arg = self._create_args(getattr(self._ros_service_resp, '_type'), out_dt_node_id)

        node_root.add_method(service_node_id, service_name, self._call_service, input_arg, output_arg)
        rospy.loginfo('Created ROS Service with name: ' + service_name)

    @staticmethod
    def _create_args(arg_name, data_type):
        """one extension object contains all info"""
        arg = ua.Argument()
        arg.Name = arg_name
        arg.DataType = data_type
        arg.ValueRank = -1
        arg.ArrayDimensions = []
        arg.Description = ua.LocalizedText(arg_name)
        return [arg]

    @staticmethod
    def get_ua_class(ua_class_name):
        return getattr(ua, to_camel_case(ua_class_name))

    @uamethod
    def _call_service(self, parent, *inputs):
        rospy.loginfo('Calling service %s under ROS node %s' % (self._service_name, parent.to_string()))
        # result = ua.CallMethodResult()
        try:
            response = self.proxy(self.ua_class_to_ros_msg(inputs[0]))
            # result.StatusCode = ua.StatusCodes.Good
            # result.InputArgumentResults.append(result.StatusCode)
            # result.OutputArguments.append(ua.Variant(self.ros_msg_to_ua_class(response),
            #                                          ua.VariantType.ExtensionObject))
            return ua.Variant(self.ros_msg_to_ua_class(response))
        except (TypeError, rospy.ROSException, rospy.ROSInternalException, rospy.ROSSerializationException,
                UaError, rosservice.ROSServiceException) as e:
            rospy.logerr('Error when calling service ' + self._service_name, e)
            # result.StatusCode = ua.StatusCodes.Bad
            # result.InputArgumentResults.append(result.StatusCode)
        # finally:
        #     return result

    # TODO: create deep copy between nested messages and ua classes
    def ua_class_to_ros_msg(self, ua_class):
        ros_msg = self._ros_service_req()
        for attr in ua_class.ua_types:
            setattr(ros_msg, attr[0], getattr(ua_class, attr[0]))
        return ros_msg

    def ros_msg_to_ua_class(self, ros_msg):
        # To deal with bugs in calling methods with empty extension objects
        if not len(ros_msg.__slots__):
            return None
        ua_class_name = getattr(self._ros_service_resp, '_type')
        ua_class = self.get_ua_class(ua_class_name)()
        for attr in ros_msg.__slots__:
            setattr(ua_class, attr, getattr(ros_msg, attr))
        return ua_class


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
