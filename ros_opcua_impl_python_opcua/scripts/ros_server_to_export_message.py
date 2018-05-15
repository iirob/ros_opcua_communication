#!/usr/bin/python
import time

import rosgraph
import rosnode
import rospy

from opcua import Server
from opcua.common.ua_utils import get_nodes_of_namespace

import ros_messages
import ros_global


# Returns the hierarchy as one string from the first remaining part on.
def nextname(hierarchy, index_of_last_processed):
    try:
        output = ""
        counter = index_of_last_processed + 1
        while counter < len(hierarchy):
            output += hierarchy[counter]
            counter += 1
        return output
    except Exception as e:
        rospy.logerr("Error encountered ", e)


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
        self.server.start()

        uri_messages = "http://ros.org/messages"
        idx_messages = self.server.register_namespace(uri_messages)

        # get variableType node, this is where we should put our custom stuff
        # objects = self.server.get_objects_node()
        variable_type = ["0:Types", "0:VariableTypes", "0:BaseVariableType", "0:BaseDataVariableType"]
        base_data_variable_type_node = self.server.get_root_node().get_child(variable_type)
        message_variable_type = base_data_variable_type_node.add_data_type(idx_messages, "ROSMessageVariableTypes")

        # parse every ros messages
        print " ----- start parsing  messages ------ "
        for message in ros_messages.get_ros_msg():
            package = message.split('/')[0]
            if package not in ros_global.packages:
                package_node = message_variable_type.add_folder(idx_messages, package)
                ros_global.packages.append(package)
                ros_global.package_node_created[package] = package_node
            else:
                package_node = ros_global.package_node_created.get(package)

            ros_messages.OpcUaROSMessage(self.server, package_node, idx_messages, message)
        print " ----- parsing  messages end------ "

        # export to xml
        print " ----- start exporting node message to xml ------ "
        node_to_export = get_nodes_of_namespace(self.server, [idx_messages])
        self.server.export_xml(node_to_export, ros_global.messageExportPath)
        print " ----- End exporting node message to xml ------ "
        while not rospy.is_shutdown():
            # ros_topics starts a lot of publisher/subscribers, might slow everything down quite a bit.
            # ros_services.refresh_services(self.namespace_ros, self, self.servicesDict, idx_services, services_object)
            # ros_topics.refresh_topics_and_actions(self.namespace_ros, self, self.topicsDict, self.actionsDict,
            #                                      idx_topics, idx_actions, topics_object, actions_object)
            # Don't clog cpu
            time.sleep(60)
        self.server.stop()
        quit()


def main():
    ROSServer()


if __name__ == "__main__":
    main()
