# Thanks to
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_action/src/rqt_action/action_plugin.py

import rospy
import roslib.message
import rostopic
import rosservice
import actionlib

from opcua import uamethod
from ros_opc_ua import *


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
            ua_class = inputs[0] if inputs else None
            response = self._proxy(ua_class_to_ros_msg(ua_class, self._ros_service_req()))
            rospy.loginfo('Calling service succeeded!')
            return ua.Variant(ros_msg_to_ua_class(response, get_ua_class(self._resp_name)()))
        except Exception as e:
            rospy.logerr('Error when calling service ' + self._service_name, e)

    def delete_node(self):
        self._proxy.close()
        self._method.delete()
        rospy.loginfo('Deleted ROS Service with name: ' + self._service_name)


class OpcUaROSTopic:

    def __init__(self, topic_name, node_root, msg_dict, *node_id):
        self._topic_name = topic_name
        self._topic_class, _, _ = rostopic.get_topic_class(self._topic_name)
        self._msg_name = getattr(self._topic_class, '_type')
        self._topic = node_root.add_variable(node_id[0], self._topic_name, ua.Variant(None, ua.VariantType.Null),
                                             datatype=msg_dict[self._msg_name])
        self._handler = None

    def delete_node(self):
        self._handler.unregister()
        self._topic.delete()
        rospy.loginfo('Deleted ROS Topic publication with name: ' + self._topic_name)


class OpcUaROSTopicPub(OpcUaROSTopic):

    def __init__(self, topic_name, node_root, msg_dict, *node_id):
        OpcUaROSTopic.__init__(self, topic_name, node_root, msg_dict, *node_id)
        rospy.loginfo('Created UA variable for ROS Publication under topic: ' + topic_name)
        self._handler = rospy.Subscriber(topic_name, self._topic_class, self._message_callback)

    def _message_callback(self, message):
        self._topic.set_value(ros_msg_to_ua_class(message, get_ua_class(self._msg_name)()))


class OpcUaROSTopicSub(OpcUaROSTopic):

    def __init__(self, topic_name, node_root, msg_dict, *node_id):
        OpcUaROSTopic.__init__(self, topic_name, node_root, msg_dict, *node_id)
        self._handler = rospy.Publisher(self._topic_name, self._topic_class, queue_size=1)

        input_arg = create_args(self._topic_class, msg_dict[self._msg_name])
        self._method = self._topic.add_method(node_id[1], 'publish', self._call_publish, input_arg, [])
        rospy.loginfo('Created UA variable for ROS Subscription under ROS topic: ' + topic_name)

    @uamethod
    def _call_publish(self, parent, *inputs):
        rospy.loginfo('Publishing data under ROS topic: %s, under node: %s' % (self._topic_name, parent.to_string()))
        try:
            self._handler.publish(ua_class_to_ros_msg(inputs[0], self._topic_class()))
            rospy.loginfo('Topic publishing succeeded!')
            return
        except Exception as e:
            rospy.logerr('Error when publishing topic ' + self._topic, e)

    def delete_node(self):
        OpcUaROSTopic.delete_node(self)
        self._method.delete()


class OpcUaROSAction:

    def __init__(self, idx, action_name, node_root, msg_dict):
        self._idx = idx
        self._action_name = action_name
        self._root = node_root.add_folder(nodeid_generator(self._idx), 'ROSAction')
        self._dict = msg_dict

        self._goal_class, self._goal = self._get_action_class(self._action_name + '/goal', nodeid_generator(self._idx))
        self._cancel_class, self._cancel = self._get_action_class(self._action_name + '/cancel',
                                                                  nodeid_generator(self._idx))
        self._status_class, self._status = self._get_action_class(self._action_name + '/status',
                                                                  nodeid_generator(self._idx))
        self._result_class, self._result = self._get_action_class(self._action_name + '/result',
                                                                  nodeid_generator(self._idx))
        self._feedback_class, self._feedback = self._get_action_class(self._action_name + '/feedback',
                                                                      nodeid_generator(self._idx))

        action_class_name = getattr(self._goal_class, '_type').replace("Goal", "")
        self._action_class = roslib.message.get_message_class(action_class_name)

        rospy.loginfo('Created ROS Action with name: ' + self._action_name)

    def _get_action_class(self, name, node_id):
        action_class, _, _ = rostopic.get_topic_class(name)
        class_name = getattr(action_class, '_type')
        ua_node = self._root.add_variable(node_id, name, ua.Variant(None, ua.VariantType.Null),
                                          datatype=self._dict[class_name])
        return action_class, ua_node

    def delete_node(self):
        self._goal.delete()
        self._cancel.delete()
        self._status.delete()
        self._result.delete()
        self._feedback.delete()
        rospy.loginfo('Deleted ROS Action with name: ' + self._action_name)


class OpcUaROSActionClient(OpcUaROSAction):

    def __init__(self, idx, action_name, node_root, msg_dict):
        OpcUaROSAction.__init__(self, idx, action_name, node_root, msg_dict)


class OpcUaROSActionServer(OpcUaROSAction):

    def __init__(self, idx, action_name, node_root, msg_dict):
        OpcUaROSAction.__init__(self, idx, action_name, node_root, msg_dict)
        try:
            self.client = actionlib.SimpleActionClient(self._action_name, self._action_class)
        except actionlib.ActionException as e:
            rospy.logerr("Error when creating ActionClient for action " + self._action_name, e)

        goal_data_id = self._dict[getattr(self._goal_class, '_type')]
        self._man_send = self._goal.add_method(nodeid_generator(self._idx), 'send goal', self._send_goal,
                                               create_args(self._goal_class, goal_data_id), [])

        self._man_cancel = self._cancel.add_method(nodeid_generator(self._idx), 'cancel goal',
                                                   self._cancel_goal, [], [])

    @uamethod
    def _send_goal(self, parent, *inputs):
        rospy.loginfo('Calling method %s under ROS node: %s, %s'
                      % ('"send goal"', self._root.get_display_name().Text, parent.to_string()))
        try:
            goal_msg = ua_class_to_ros_msg(inputs[0], self._goal_class())
            rospy.loginfo("Created Message Instances for goal-send: " + goal_msg)
            self.client.send_goal(goal_msg, done_cb=self._update_result, feedback_cb=self._update_feedback,
                                  active_cb=self._update_state)
        except Exception as e:
            rospy.logerr("Error occurred during goal sending for Action " + self._action_name, e)

    @uamethod
    def _cancel_goal(self, parent, *inputs):
        rospy.loginfo('Calling method %s under ROS node: %s, %s'
                      % (self._action_name, self._root.get_display_name().Text, parent.to_string()))
        try:
            if inputs[0]:
                raise Exception('Incorrect number of args!')
            self.client.cancel_all_goals()
            self._update_state()
        except Exception as e:
            rospy.logerr("Error when cancelling a goal for " + self._action_name, e)

    def _update_result(self, state, result):
        rospy.logdebug("updated result cb reached")
        self._status.set_value(ros_msg_to_ua_class(state, self._status_class()))
        self._result.set_value(ros_msg_to_ua_class(result, self._result_class()))

    def _update_state(self):
        self._status.set_value(ros_msg_to_ua_class(self.client.get_state(), self._status_class()))

    def _update_feedback(self, feedback):
        rospy.logdebug("updated feedback cb reached")
        self._feedback.set_value(ros_msg_to_ua_class(feedback, self._feedback_class()))

    def delete_node(self):
        self.client.cancel_all_goals()
        self._man_send.delete()
        self._man_cancel.delete()
        OpcUaROSAction.delete_node(self)
