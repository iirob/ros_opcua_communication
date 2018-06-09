#!/usr/bin/python
import logging
import rospy

from opcua import Server, ua
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
        data_variable_root_path = ['0:Types', '0:VariableTypes', '0:BaseVariableType', '0:BaseDataVariableType']
        dv_base_node = self.server.get_root_node().get_child(data_variable_root_path)
        # As can be seen in UAExpert, in BaseDataVariableType, all nodes are VariableType, only our
        # TODO: ROSMessageVariableTypes is a data type, maybe change to add_variable_type???
        # isAbstract should be set to true
        self._msg_type = dv_base_node.add_data_type(self._idx_messages, 'ROSMessageVariableTypes')
        data_type_root_path = ['0:Types', '0:DataTypes', '0:BaseDataType', '0:Structure']
        self._dt_base_node = self.server.get_root_node().get_child(data_type_root_path)

    def __enter__(self):
        self.server.start()
        rospy.init_node('rosopcua')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.server.stop()
        quit()

    def export_message(self):
        print(' ----- start parsing messages ------ ')
        for message in ros_global.get_ros_msg():
            package = message.split('/')[0]
            if package not in ros_global.packages:
                package_node = self._msg_type.add_folder(self._idx_messages, package)
                ros_global.packages.append(package)
                ros_global.package_node_created[package] = package_node
            else:
                package_node = ros_global.package_node_created.get(package)

            ros_messages.OpcUaROSMessage(self.server, package_node, self._idx_messages, self._dt_base_node,  message)
        print(' ----- parsing messages end------ ')
        print(' ----- start exporting node message to xml ------ ')
        node_to_export = get_nodes_of_namespace(self.server, [self._idx_messages])
        self.server.export_xml(node_to_export, ros_global.messageExportPath)
        print(' ----- End exporting node message to xml ------ ')

    def _traversal_messages(self):
        for msg in ros_global.get_ros_msg():
            self._recursively_create_message(msg)

    def _recursively_create_message(self, msg):
        message = ros_messages._get_message_class(msg)
        for variable_type, data_type in zip(message.__slots__, getattr(message, '_slot_types')):
            base_type_str, array_size = ros_messages._extract_array_info(data_type)
            if base_type_str not in ros_messages.ros_build_in_data_types.keys() and \
                    base_type_str not in ros_messages.created_data_types.keys():
                self._recursively_create_message(base_type_str)

            else:
                variable_type_name = base_type_str + 'Type'
                # first time, create a variable type
                if variable_type_name in ros_messages.created_variable_types.keys():
                    new_variable_type = self._msg_type.add_variable(
                        ua.NodeId(ros_global.get_counter(), self._idx_messages),
                        ua.QualifiedName(variable_type, self._idx_messages),
                        new_data_type.nodeid)
                    ros_messages.created_variable_types[base_type_str] = new_variable_type
                # not first time,
                # if basic type
                # add property
                # else add variable

            if base_type_str not in ros_messages.created_data_types.keys():
                new_data_type = self._dt_base_node.add_data_node(
                    ua.NodeId(ros_global.get_counter(), self._idx_messages),
                    ua.QualifiedName(base_type_str, self._idx_messages),
                    data_type)
                ros_messages.created_data_types[base_type_str] = new_data_type

                new_variable_type = self._msg_type.add_variable_type(
                    ua.NodeId(ros_global.get_counter(), self._idx_messages),
                    ua.QualifiedName(base_type_str + 'Type', self._idx_messages),
                    new_data_type.nodeid)
                ros_messages.created_variable_types[base_type_str] = new_variable_type
            # add data type, recursion, add variable type, then add variable


if __name__ == '__main__':
    logging.basicConfig(level=logging.WARNING)
    try:
        with ROSServer() as ua_server:
            ua_server.export_message()
    except Exception as e:
        print(e.message)
    # ROSServer().export_message()
    # ROSServer()._traversal_messages()
