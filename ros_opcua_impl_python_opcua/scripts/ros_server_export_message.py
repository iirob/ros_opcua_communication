#!/usr/bin/python
import rospy

from ros_global import BasicROSServer
from ros_messages import OpcUaROSMessage


class ROSServer(BasicROSServer):

    def __init__(self):
        BasicROSServer.__init__(self)
        self._ros_nodes = {}
        self._ua_nodes = {}
        self._node_items = {}

    @staticmethod
    def _is_action(names):
        for name in names:
            if name.split('/')[-1] in ['cancel', 'goal']:
                return True
        return False

    def load_messages(self):
        rospy.loginfo(' ----- start creating messages ------ ')
        self.ros_msgs = OpcUaROSMessage(self.server, self.idx, self._idx_name).create_ros_data_types()
        rospy.loginfo(' ----- %s messages created------ ' % str(len(self.ros_msgs)))


if __name__ == '__main__':
    try:
        with ROSServer() as ua_server:
            ua_server.load_messages()
            ua_server.export_messages()
    except Exception as e:
        print(e.message)
