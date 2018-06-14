#!/usr/bin/python

import rospy

from opcua.common.ua_utils import get_nodes_of_namespace

from ros_global import BasicROSServer, new_messageExportPath
from ros_messages import OpcUaROSMessage


class ROSServer(BasicROSServer):
    def _create_messages(self):
        rospy.loginfo(' ----- start creating messages ------ ')
        message_object = OpcUaROSMessage(self._server, self._idx)
        message_object.create_messages()
        rospy.loginfo(' ----- %s messages created------ ' % str(len(message_object.created_variable_types.keys())))

    def export_messages(self):
        self._create_messages()
        rospy.loginfo(' ----- start exporting node message to xml ------ ')
        node_to_export = get_nodes_of_namespace(self._server, [self._idx])
        self._server.export_xml(node_to_export, new_messageExportPath)
        rospy.loginfo(' ----- node message exported to %s ------ ' % new_messageExportPath)


if __name__ == '__main__':
    try:
        with ROSServer() as ua_server:
            ua_server._create_messages()
            rospy.spin()
    except Exception as e:
        print(e.message)
