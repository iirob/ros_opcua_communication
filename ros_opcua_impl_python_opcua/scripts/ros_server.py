#!/usr/bin/python
import sys
import time

import rosgraph
import rosnode
import rospy
from opcua import Server, ua

import ros_services
import ros_topics
import ros_global
from opcua.common.ua_utils import get_nodes_of_namespace
from opcua.common.instantiate import instantiate


# Returns the hierarchy as one string from the first remaining part on.
def next_name(hierarchy, index_of_last_processed):
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
        self.subscription = None

        # setup our own namespaces, this is expected
        uri_topics = "http://ros.org/topics"

        # two different namespaces to make getting the correct node easier for get_node
        # otherwise had object for service and topic with same name
        uri_services = "http://ros.org/services"
        uri_actions = "http://ros.org/actions"
        uri_messages = "http://ros.org/messages"
        idx_topics = self.server.register_namespace(uri_topics)
        idx_services = self.server.register_namespace(uri_services)
        idx_actions = self.server.register_namespace(uri_actions)
        # idx_messages = self.server.register_namespace(uri_messages)

        # get Objects node, this is where we should put our custom stuff
        objects = self.server.get_objects_node()
        # one object per type we are watching
        topics_object = objects.add_object(idx_topics, "ROS-Topics")
        services_object = objects.add_object(idx_services, "ROS-Services")
        actions_object = objects.add_object(idx_actions, "ROS_Actions")
        messages_object = objects.add_object(idx_actions, "ROS_Messages")

        # import message in message space
        print "---- message import started ----"
        self.server.import_xml(ros_global.messageExportPath)
        print "---- message imported ----"

        idx_messages = self.server.get_namespace_index(uri_messages)
        imported_node = get_nodes_of_namespace(self.server, [idx_messages])

        for node in imported_node:
            identifier = str(node.nodeid.Identifier)
            # only for VariableType, check if the identifier ends with 'Type' (not 'DataType' ->  DataType )
            if node.get_node_class() == ua.NodeClass.Variable:
                instantiate(messages_object, node, idx=idx_messages)
                # save node in a List/Array
                message_typ = identifier.replace('Type', '')
                ros_global.messageNode[message_typ] = node

        # create subscription to handle write update event in namespace,
        # when a variable is edited in client for example and new message have to be published
        handler = SubHandler()
        self.subscription = self.server.create_subscription(100, handler)

        while not rospy.is_shutdown():
            # ros_topics starts a lot of publisher/subscribers, might slow everything down quite a bit.
            ros_services.refresh_services(self.namespace_ros, self, self.servicesDict, idx_services, services_object)
            ros_topics.refresh_topics_and_actions(self.namespace_ros, self, self.topicsDict, self.actionsDict,
                                                  idx_topics, idx_actions, topics_object, actions_object)
            # Don't clog cpu
            time.sleep(60)
        self.server.stop()
        quit()

    def find_service_node_with_same_name(self, name, idx):
        rospy.logdebug("Reached ServiceCheck for name " + name)
        for service in self.servicesDict:
            rospy.logdebug("Found name: " + str(self.servicesDict[service].parent.nodeid.Identifier))
            if self.servicesDict[service].parent.nodeid.Identifier == name:
                rospy.logdebug("Found match for name: " + name)
                return self.servicesDict[service].parent
        return None

    def find_topics_node_with_same_name(self, name, idx):
        rospy.logdebug("Reached TopicCheck for name " + name)
        for topic in self.topicsDict:
            rospy.logdebug("Found name: " + str(self.topicsDict[topic].parent.nodeid.Identifier))
            if self.topicsDict[topic].parent.nodeid.Identifier == name:
                rospy.logdebug("Found match for name: " + name)
                return self.topicsDict[topic].parent
        return None

    def find_action_node_with_same_name(self, name, idx):
        rospy.logdebug("Reached ActionCheck for name " + name)
        for topic in self.actionsDict:
            rospy.logdebug("Found name: " + str(self.actionsDict[topic].parent.nodeid.Identifier))
            if self.actionsDict[topic].parent.nodeid.Identifier == name:
                rospy.logdebug("Found match for name: " + name)
                return self.actionsDict[topic].parent

        return None


class SubHandler(object):
    """
        This Handle variable node (Topics) update/edit.
        When a variable node (Topics) is in client edited and in namespace edited,
        the changed topics has to be republished
    """

    def event_notification(self, event):
        print " a variable node has been edited/updated "
        print event
        # here most a topic published


if __name__ == "__main__":
    ROSServer()
