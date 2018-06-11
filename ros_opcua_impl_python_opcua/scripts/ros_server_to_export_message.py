#!/usr/bin/python
import logging
import rospy

from opcua import Server
from opcua.common.ua_utils import get_nodes_of_namespace

import ros_global
from ros_messages import OpcUaROSMessage


class ROSServer:
    def __init__(self):
        self.server = Server()

        self.server.set_endpoint('opc.tcp://0.0.0.0:21554/')
        self.server.set_server_name('ROS ua Server')
        self._idx = self.server.register_namespace('http://ros.org/messages')

    def __enter__(self):
        self.server.start()
        rospy.init_node('rosopcua')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.server.stop()
        quit()

    def export_message(self):
        print(' ----- start parsing messages ------ ')
        OpcUaROSMessage(self.server, self._idx).create_messages()
        print(' ----- parsing messages end------ ')
        print(' ----- start exporting node message to xml ------ ')
        node_to_export = get_nodes_of_namespace(self.server, [self._idx])
        self.server.export_xml(node_to_export, ros_global.new_messageExportPath)
        print(' ----- End exporting node message to xml ------ ')


if __name__ == '__main__':
    logging.basicConfig(level=logging.WARNING)
    try:
        with ROSServer() as ua_server:
            ua_server.export_message()
    except Exception as e:
        print(e.message)
