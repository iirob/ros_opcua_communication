# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_service_caller/src/rqt_service_caller/service_caller_widget.py

import rosservice
import rospy
from opcua import ua, uamethod

from ros_opc_ua import create_args, ros_msg_to_ua_class, ua_class_to_ros_msg, get_ua_class


class OpcUaROSService:

    def __init__(self, service_name, node_root, service_node_id, msg_dict):
        self._service_class = rosservice.get_service_class_by_name(service_name)
        self._service_name = service_name
        self._proxy = rospy.ServiceProxy(service_name, self._service_class)
        self._node_root = node_root

        self._ros_service_req = getattr(self._service_class, '_request_class')
        self._req_name = getattr(self._ros_service_req, '_type')
        self._ros_service_resp = getattr(self._service_class, '_response_class')
        self._resp_name = getattr(self._ros_service_resp, '_type')

        in_dt_node_id = msg_dict[self._req_name]
        out_dt_node_id = msg_dict[self._resp_name]
        input_arg = create_args(self._ros_service_req, in_dt_node_id)
        output_arg = create_args(self._ros_service_resp, out_dt_node_id)

        self._method = node_root.add_method(service_node_id, service_name, self._call_service, input_arg, output_arg)
        rospy.loginfo('Created ROS Service with name: ' + service_name)

    @uamethod
    def _call_service(self, parent, *inputs):
        rospy.loginfo('Calling service %s under ROS node: %s, %s'
                      % (self._service_name, self._node_root.get_display_name().Text, parent.to_string()))
        try:
            response = self._proxy(ua_class_to_ros_msg(inputs[0], self._ros_service_req()))
            rospy.loginfo('Calling service succeeded!')
            return ua.Variant(ros_msg_to_ua_class(response, get_ua_class(self._resp_name)()))
        except Exception as e:
            rospy.logerr('Error when calling service ' + self._service_name, e)

    def delete_node(self):
        self._proxy.close()
        self._method.delete()
        rospy.loginfo('Deleted ROS Service with name: ' + self._service_name)
