#!/usr/bin/python
import rospy

from opcua.common.ua_utils import get_nodes_of_namespace

from ros_opc_ua_comm import BasicROSServer, message_export_path


class ROSServer(BasicROSServer):

    def export_messages(self):
        rospy.loginfo(' ----- Exporting node message to xml ------ ')
        node_to_export = get_nodes_of_namespace(self._server, [self._idx])
        self._server.export_xml(node_to_export, message_export_path)
        rospy.loginfo(' ----- {0} nodes exported to {1}------ '.format(len(node_to_export), message_export_path))


if __name__ == '__main__':
    try:
        with ROSServer() as ua_server:
            ua_server.load_messages()
            ua_server.export_messages()
    except Exception as e:
        print(e.message)
