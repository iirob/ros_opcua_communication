#!/usr/bin/python
import time
import logging

import rospy

from opcua import Server, ua
from opcua.common.ua_utils import get_nodes_of_namespace

import ros_global
import ros_services
import ros_topics
# TODO: split ros_topics and ros_actions, the cross reference between them may be the reason of the bug in ppt


class ROSServer:
    def __init__(self):
        self._topicsDict = {}
        self._servicesDict = {}
        self._actionsDict = {}

        self.server = Server()

        self.server.set_endpoint('opc.tcp://0.0.0.0:21554/')
        self.server.set_server_name('ROS ua Server')
        self._namespace_ros = rospy.get_param('/rosopcua/namespace')

        # setup our own namespaces, this is expected
        self._idx_topics = self.server.register_namespace('http://ros.org/topics')
        self._idx_services = self.server.register_namespace('http://ros.org/service')
        self._idx_actions = self.server.register_namespace('http://ros.org/actions')
        # idx_messages = self.server.register_namespace(uri_messages)

        # get objects node
        objects = self.server.get_objects_node()
        # TODO: Maybe change the add_object to folders
        self._topics_object = objects.add_object(self._idx_topics, 'ROS-Topics')
        self._services_object = objects.add_object(self._idx_services, 'ROS-Services')
        self._actions_object = objects.add_object(self._idx_actions, 'ROS_Actions')
        # set the refresh cycle time
        self._cycle_time = 60

        self._import_messages()

    def __enter__(self):
        self.server.start()
        rospy.init_node('rosopcua')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.server.stop()
        quit()

    def _import_messages(self):
        print('---- message import started ----')
        self.server.import_xml(ros_global.messageExportPath)
        print('---- message imported ----')

        idx_messages = self.server.get_namespace_index('http://ros.org/messages')
        imported_node = get_nodes_of_namespace(self.server, [idx_messages])

        for node in imported_node:
            identifier = str(node.nodeid.Identifier)
            if node.get_node_class() == ua.NodeClass.Variable:
                message_typ = identifier.replace('Type', '')
                ros_global.messageNode[message_typ] = node

    @staticmethod
    def _find_node_with_same_name(name, node_dict):
        rospy.logdebug('Search for node with name: ' + name)
        for item in node_dict.keys():
            rospy.logdebug('Found name: ' + str(node_dict[item].parent.nodeid.Identifier))
            if node_dict[item].parent.nodeid.Identifier == name:
                rospy.logdebug('Found match for name: ' + name)
                return node_dict[item].parent
        return None

    def find_service_node_with_same_name(self, name):
        return self._find_node_with_same_name(name, self._servicesDict)

    def find_topics_node_with_same_name(self, name):
        return self._find_node_with_same_name(name, self._topicsDict)

    def find_action_node_with_same_name(self, name):
        return self._find_node_with_same_name(name, self._actionsDict)

    def cyclic_refresh(self):
        while not rospy.is_shutdown():
            # ros_topics starts a lot of publisher/subscribers, might slow everything down quite a bit.
            ros_services.refresh_services(self._namespace_ros, self, self._servicesDict,
                                          self._idx_services, self._services_object)
            ros_topics.refresh_topics_and_actions(self._namespace_ros, self, self._topicsDict, self._actionsDict,
                                                  self._idx_topics, self._idx_actions, self._topics_object,
                                                  self._actions_object)
            # Don't clog cpu
            time.sleep(self._cycle_time)


if __name__ == '__main__':
    logging.basicConfig(level=logging.WARNING)
    try:
        with ROSServer() as ua_server:
            ua_server.cyclic_refresh()
    except Exception as e:
        print(e.message)
