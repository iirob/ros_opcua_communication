# Thanks to
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_action/src/rqt_action/action_plugin.py

import actionlib
import roslib.message
import rospy
import rostopic

from opcua import ua, uamethod
from opcua.ua.uaerrors import UaError

from ros_opc_ua import create_args, ua_class_to_ros_msg, ros_msg_to_ua_class, nodeid_generator


class OpcUaROSAction:

    def __init__(self, idx, action_name, node_root, msg_dict, is_client):
        self._idx = idx
        self._action_name = action_name
        self._root = node_root
        self._pub_root = self._root.add_folder(nodeid_generator(self._idx), 'Publications')
        self._sub_root = self._root.add_folder(nodeid_generator(self._idx), 'Subscriptions')
        self._dict = msg_dict
        goal_cancel_root = self._pub_root if is_client else self._sub_root
        status_feedback_root = self._sub_root if is_client else self._pub_root
        self._goal_class, self._goal = self._get_action_class(self._action_name + '/goal',
                                                              nodeid_generator(self._idx), goal_cancel_root)
        self._cancel_class, self._cancel = self._get_action_class(self._action_name + '/cancel',
                                                                  nodeid_generator(self._idx), goal_cancel_root)
        self._status_class, self._status = self._get_action_class(self._action_name + '/status',
                                                                  nodeid_generator(self._idx), status_feedback_root)
        self._result_class, self._result = self._get_action_class(self._action_name + '/result',
                                                                  nodeid_generator(self._idx), status_feedback_root)
        self._feedback_class, self._feedback = self._get_action_class(self._action_name + '/feedback',
                                                                      nodeid_generator(self._idx), status_feedback_root)

        action_class_name = getattr(self._goal_class, '_type').replace("Goal", "")
        self._action_class = roslib.message.get_message_class(action_class_name)

        rospy.loginfo('Created ROS Action with name: ' + self._action_name)

    def _get_action_class(self, name, node_id, root):
        action_class, _, _ = rostopic.get_topic_class(name)
        class_name = getattr(action_class, '_type')
        ua_node = root.add_variable(node_id, name, ua.Variant(None, ua.VariantType.Null),
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
        OpcUaROSAction.__init__(self, idx, action_name, node_root, msg_dict, is_client=True)


class OpcUaROSActionServer(OpcUaROSAction):

    def __init__(self, idx, action_name, node_root, msg_dict):
        OpcUaROSAction.__init__(self, idx, action_name, node_root, msg_dict, is_client=False)
        try:
            self.client = actionlib.SimpleActionClient(self._action_name, self._action_class)
        except actionlib.ActionException as e:
            rospy.logerr("Error when creating ActionClient for action " + self._action_name, e)

        goal_data_id = self._dict[getattr(self._goal_class, '_type')]
        self._man_send = self._goal.add_method(nodeid_generator(self._idx), 'manual send goal', self._send_goal,
                                               create_args(self._goal_class, goal_data_id), [])

        self._man_cancel = self._cancel.add_method(nodeid_generator(self._idx), 'manual cancel goal',
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
        except (rospy.ROSException, UaError) as e:
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

# This is not considered yet
# # malformed move_base_simple Action hack
# if 'move_base_simple' in self.name:
#     self.goal_instance = self.goal_class()
# else:
#     self.goal_instance = self.goal_class().goal
# rospy.logdebug("found goal_instance " + str(self.goal_instance))
#
# ROS_GOAL_STATUS = {9: 'Goal LOST',
#                    8: 'Goal RECALLED',
#                    7: 'Goal RECALLING',
#                    6: 'Goal PREEMPTING',
#                    5: 'Goal REJECTED',
#                    4: 'Goal ABORTED',
#                    3: 'Goal SUCCEEDED',
#                    2: 'Goal PREEMPTED',
#                    1: 'Goal ACTIVE',
#                    0: 'Goal PENDING'}
