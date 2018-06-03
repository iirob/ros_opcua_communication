#!/usr/bin/python
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
        output = ''
        counter = index_of_last_processed + 1
        while counter < len(hierarchy):
            output += hierarchy[counter]
            counter += 1
        return output
    except Exception as ex:
        rospy.logerr('Error encountered ', ex)


def own_rosnode_cleanup():
    pinged, unpinged = rosnode.rosnode_ping_all()
    if unpinged:
        master = rosgraph.Master(rosnode.ID)
        # noinspection PyTypeChecker
        rosnode.cleanup_master_blacklist(master, unpinged)


class ROSServer:
    def __init__(self):
        self.namespace_ros = rospy.get_param('/rosopcua/namespace')
        self.topicsDict = {}
        self.servicesDict = {}
        self.actionsDict = {}
        self.server = Server()
        self.server.set_endpoint('opc.tcp://0.0.0.0:21554/')
        self.server.set_server_name('ROS ua Server')
        self.subscription = None

        # setup our own namespaces, this is expected
        self.idx_topics = self.server.register_namespace('http://ros.org/topics')
        self.idx_services = self.server.register_namespace('http://ros.org/service')
        self.idx_actions = self.server.register_namespace('http://ros.org/actions')
        # idx_messages = self.server.register_namespace(uri_messages)

        # get Objects node, this is where we should put our custom stuff
        objects = self.server.get_objects_node()
        # one object per type we are watching
        self.topics_object = objects.add_object(self.idx_topics, 'ROS-Topics')
        self.services_object = objects.add_object(self.idx_services, 'ROS-Services')
        self.actions_object = objects.add_object(self.idx_actions, 'ROS_Actions')
        self._import_messages(objects)
        self.cycle_time = 60

    def __enter__(self):
        self.server.start()
        rospy.init_node('rosopcua')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.server.stop()
        quit()

    def _import_messages(self, root):
        # import message in message space
        print('---- message import started ----')
        self.server.import_xml(ros_global.messageExportPath)
        print('---- message imported ----')

        idx_messages = self.server.get_namespace_index('http://ros.org/messages')
        messages_object = root.add_object(idx_messages, 'ROS_Messages')
        imported_node = get_nodes_of_namespace(self.server, [idx_messages])

        for node in imported_node:
            identifier = str(node.nodeid.Identifier)
            # only for VariableType, check if the identifier ends with 'Type' (not 'DataType' ->  DataType )
            # No VariableType is created at all in the xml file, I think you mean Variable
            if node.get_node_class() == ua.NodeClass.Variable:
                instantiate(messages_object, node, idx=idx_messages)
                # save node in a List/Array
                message_typ = identifier.replace('Type', '')
                ros_global.messageNode[message_typ] = node

    @staticmethod
    def _find_node_with_same_name(name, node_type, node_dict):
        rospy.logdebug('Reached %s Check for name %s' % (name, node_type))
        for service in node_dict:
            rospy.logdebug('Found name: ' + str(node_dict[service].parent.nodeid.Identifier))
            if node_dict[service].parent.nodeid.Identifier == name:
                rospy.logdebug('Found match for name: ' + name)
                return node_dict.parent
        return None

    def find_service_node_with_same_name(self, name):
        self._find_node_with_same_name(name, 'Service', self.servicesDict)

    def find_topics_node_with_same_name(self, name):
        self._find_node_with_same_name(name, 'Topic', self.topicsDict)

    def find_action_node_with_same_name(self, name):
        self._find_node_with_same_name(name, 'Action', self.actionsDict)

    def cyclic_refresh(self):
        # create subscription to handle write update event in namespace,
        # when a variable is edited in client for example and new message have to be published
        handler = SubHandler()
        self.subscription = self.server.create_subscription(100, handler)

        while not rospy.is_shutdown():
            # ros_topics starts a lot of publisher/subscribers, might slow everything down quite a bit.
            ros_services.refresh_services(self.namespace_ros, self, self.servicesDict,
                                          self.idx_services, self.services_object)
            ros_topics.refresh_topics_and_actions(self.namespace_ros, self, self.topicsDict, self.actionsDict,
                                                  self.idx_topics, self.idx_actions, self.topics_object,
                                                  self.actions_object)
            # Don't clog cpu
            time.sleep(self.cycle_time)


class SubHandler(object):
    """
        This Handle variable node (Topics) update/edit.
        When a variable node (Topics) is in client edited and in namespace edited,
        the changed topics has to be republished
    """

    def event_notification(self, event):
        print('a variable node at %s has been edited/updated' % hex(id(self)))
        print(event)
        # here most a topic published


if __name__ == '__main__':
    try:
        with ROSServer() as ua_server:
            ua_server.cyclic_refresh()
    except Exception as e:
        print(e.message)
