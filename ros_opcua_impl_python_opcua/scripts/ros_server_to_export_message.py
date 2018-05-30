import rosgraph
import rosnode
import rospy

from opcua import Server
from opcua.common.ua_utils import get_nodes_of_namespace

import ros_messages
import ros_global


def next_name(hierarchy, index_of_last_processed):
    """
    Returns the hierarchy as one string from the first remaining part on.
    :param hierarchy: 
    :param index_of_last_processed: 
    :return: 
    """
    try:
        output = ""
        counter = index_of_last_processed + 1
        while counter < len(hierarchy):
            output += hierarchy[counter]
            counter += 1
        return output
    except Exception as ex:
        rospy.logerr("Error encountered ", ex)


def own_rosnode_cleanup():
    pinged, unpinged = rosnode.rosnode_ping_all()
    if unpinged:
        master = rosgraph.Master(rosnode.ID)
        # noinspection PyTypeChecker
        rosnode.cleanup_master_blacklist(master, unpinged)


class ROSServer:
    def __init__(self):
        self.namespace_ros = rospy.get_param("/rosopcua/namespace")
        self.topicsDict = {}
        self.servicesDict = {}
        self.actionsDict = {}
        rospy.init_node("rosopcua")
        self.server = Server()
        self.server.set_endpoint("opc.tcp://0.0.0.0:21554/")
        self.server.set_server_name("ROS ua Server")

        self.uri_messages = "http://ros.org/messages"
        self.idx_messages = self.server.register_namespace(self.uri_messages)

        # get variableType node, this is where we should put our custom stuff
        hierarchical_path = ["0:Types", "0:VariableTypes", "0:BaseVariableType", "0:BaseDataVariableType"]
        base_data_variable_type_node = self.server.get_root_node().get_child(hierarchical_path)
        # As can be seen in UAExpert, in BaseDataVariableType, all nodes are VariableType, only our
        # ROSMessageVariableTypes is a data type, maybe change to add_variable_type??? isAbstract should be set to true
        self.message_variable_type = base_data_variable_type_node.add_data_type(self.idx_messages,
                                                                                "ROSMessageVariableTypes")

    def __enter__(self):
        self.server.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.server.stop()
        quit()

    def export_message(self):
        # parse every ros messages
        print(" ----- start parsing messages ------ ")
        for message in ros_messages.get_ros_msg():
            package = message.split('/')[0]
            if package not in ros_global.packages:
                package_node = self.message_variable_type.add_folder(self.idx_messages, package)
                ros_global.packages.append(package)
                ros_global.package_node_created[package] = package_node
            else:
                package_node = ros_global.package_node_created.get(package)

            ros_messages.OpcUaROSMessage(self.server, package_node, self.idx_messages, message)
        print(" ----- parsing  messages end------ ")

        # export to xml
        print(" ----- start exporting node message to xml ------ ")
        node_to_export = get_nodes_of_namespace(self.server, [self.idx_messages])
        self.server.export_xml(node_to_export, ros_global.messageExportPath)
        print(" ----- End exporting node message to xml ------ ")


if __name__ == "__main__":
    try:
        with ROSServer() as ua_server:
            ua_server.export_message()
    except Exception as e:
        print(e.message)
