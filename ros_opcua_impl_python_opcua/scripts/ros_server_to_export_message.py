#!/usr/bin/python
import logging
import rospy

from opcua import Server
from opcua.common.ua_utils import get_nodes_of_namespace

import ros_global
import ros_messages


class ROSServer:
    def __init__(self):
        self.server = Server()

        self.server.set_endpoint('opc.tcp://0.0.0.0:21554/')
        self.server.set_server_name('ROS ua Server')
        self._idx_messages = self.server.register_namespace('http://ros.org/messages')

        # get BaseDataVariableType as base node
        hierarchical_path = ['0:Types', '0:VariableTypes', '0:BaseVariableType', '0:BaseDataVariableType']
        base_node = self.server.get_root_node().get_child(hierarchical_path)
        # As can be seen in UAExpert, in BaseDataVariableType, all nodes are VariableType, only our
        # TODO: ROSMessageVariableTypes is a data type, maybe change to add_variable_type???
        # isAbstract should be set to true
        self._msg_type = base_node.add_data_type(self._idx_messages, 'ROSMessageVariableTypes')

    def __enter__(self):
        self.server.start()
        rospy.init_node('rosopcua')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.server.stop()
        quit()

    def export_message(self):
        print(' ----- start parsing messages ------ ')
        for message in ros_messages.get_ros_msg():
            package = message.split('/')[0]
            if package not in ros_global.packages:
                package_node = self._msg_type.add_folder(self._idx_messages, package)
                ros_global.packages.append(package)
                ros_global.package_node_created[package] = package_node
            else:
                package_node = ros_global.package_node_created.get(package)

            ros_messages.OpcUaROSMessage(self.server, package_node, self._idx_messages, message)
        print(' ----- parsing messages end------ ')
        print(' ----- start exporting node message to xml ------ ')
        node_to_export = get_nodes_of_namespace(self.server, [self._idx_messages])
        self.server.export_xml(node_to_export, ros_global.messageExportPath)
        print(' ----- End exporting node message to xml ------ ')


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    try:
        with ROSServer() as ua_server:
            ua_server.export_message()
    except Exception as e:
        print(e.message)
