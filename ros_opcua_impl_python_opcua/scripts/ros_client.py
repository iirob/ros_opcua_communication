#!/usr/bin/python
import time
import rospy

from basic_server_client import ROSBasicClient
# from opcua.common.type_dictionary_buider import get_ua_class


class SubHandler(object):

    @staticmethod
    def datachange_notification(node, val, _):
        if not val:
            rospy.loginfo('Empty extension object received in node' + node.nodeid)
        else:
            rospy.loginfo(ROSBasicClient.expand_value(val))


class ROSClient(ROSBasicClient):

    def subscribe_topics(self):
        if not self._topics:
            self._refresh_topics()
        publishing_interval = 500
        sub = self._client.create_subscription(publishing_interval, SubHandler())
        sub.subscribe_data_change(self._topics.values())


if __name__ == '__main__':
    with ROSClient() as client:
        rospy.loginfo(' ----- rosservice ------ ')
        client.list_services()
        time.sleep(1)
        rospy.loginfo(' ----- rostopic ------ ')
        client.list_topics()
        time.sleep(1)
        rospy.loginfo(' ----- rosnode ------ ')
        client.list_ros_nodes()
        time.sleep(1)
        rospy.loginfo(' ----- rosparam ------ ')
        client.list_params()
        time.sleep(1)
        rospy.loginfo(' ----- rostopic values ------ ')
        client.show_topics()
        time.sleep(1)
        rospy.loginfo(' ----- rosparam values ------ ')
        client.show_params()
        time.sleep(1)
        rospy.loginfo(' ----- rosnode details------ ')
        client.show_ros_nodes()
        rospy.loginfo(' ----- static information end------ ')

        # Call Methods
        # time.sleep(1)
        # # Test case for turtlesim
        # obj = get_ua_class('turtlesim/TeleportRelativeRequest')()
        # obj.linear = 1
        # obj.angular = 1
        # client.call_service('/turtle1/teleport_relative', obj)
        # time.sleep(1)
        # # Test case for turtlesim
        # obj = get_ua_class('geometry_msgs/Twist')()
        # obj.linear.x = 1
        # client.publish_topic('/turtle1/cmd_vel', obj)

        # subscribe to topic publications
        time.sleep(1)
        client.subscribe_topics()
        rospy.spin()
