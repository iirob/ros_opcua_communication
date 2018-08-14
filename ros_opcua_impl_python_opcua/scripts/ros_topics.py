# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_topic/src/rqt_topic/topic_widget.py

import rospy
import rostopic

from opcua import ua, uamethod

from ros_opc_ua import create_args, ua_class_to_ros_msg, ros_msg_to_ua_class, get_ua_class


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
        self._method = self._topic.add_method(node_id[1], 'manual publish', self._call_publish, input_arg, [])
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
