#!/usr/bin/python
import rospy

from opcua.common.ua_utils import get_nodes_of_namespace

from ros_global import BasicROSServer, MESSAGE_EXPORT_PATH
from ros_messages import OpcUaROSMessage


class ROSServer(BasicROSServer):

    def load_messages(self):
        rospy.loginfo(' ----- start creating messages ------ ')
        self.ros_msgs = OpcUaROSMessage(self.server, self.idx, self._idx_name).create_ros_data_types()
        rospy.loginfo(' ----- %s messages created------ ' % str(len(self.ros_msgs)))

    def export_messages(self):
        rospy.loginfo(' ----- start exporting node message to xml ------ ')
        node_to_export = get_nodes_of_namespace(self.server, [self.idx])
        self.server.export_xml(node_to_export, MESSAGE_EXPORT_PATH)
        rospy.loginfo(' ----- %s nodes are exported ------ ' % len(node_to_export))
        rospy.loginfo(' ----- node message exported to %s ------ ' % MESSAGE_EXPORT_PATH)


if __name__ == '__main__':
    try:
        with ROSServer() as ua_server:
            ua_server.load_messages()
            ua_server.export_messages()
    except Exception as e:
        print(e.message)
