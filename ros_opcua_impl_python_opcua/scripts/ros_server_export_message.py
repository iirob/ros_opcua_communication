#!/usr/bin/python
import time
import rospy

from ros_global import BasicROSServer, rosnode_cleanup
from ros_opc_ua import nodeid_generator
from ros_messages import OpcUaROSMessage
from ros_services import OpcUaROSService
from ros_topics import OpcUaROSTopicPub, OpcUaROSTopicSub
from ros_actions import OpcUaROSActionClient, OpcUaROSActionServer


class ROSServer(BasicROSServer):

    def __init__(self):
        BasicROSServer.__init__(self)
        self._ros_nodes = {}
        self._ua_nodes = {}
        self._node_items = {}

    @staticmethod
    def _is_action(names):
        for name in names:
            if name.split('/')[-1] in ['cancel', 'goal']:
                return True
        return False

    def load_messages(self):
        rospy.loginfo(' ----- start creating messages ------ ')
        self.ros_msgs = OpcUaROSMessage(self.server, self.idx, self._idx_name).create_ros_data_types()
        # Generate python ua class according to ROSDictionary and register them
        self.server.load_type_definitions()
        rospy.loginfo(' ----- %s messages created------ ' % str(len(self.ros_msgs)))

    def _delete_node(self, node_name):
        for item in self._node_items[node_name]:
            item.delete_node()
        self._ua_nodes[node_name].delete()
        del self._ua_nodes[node_name]
        del self._node_items[node_name]

    def _create_node(self, node_name, node_content):
        ua_node = self.server.nodes.objects.add_object(nodeid_generator(self.idx), node_name)
        self._ua_nodes[node_name] = ua_node
        self._node_items[node_name] = []
        # services
        srv_node = ua_node.add_folder(nodeid_generator(self.idx), 'Services')
        for service in node_content['srvs']:
            self._node_items[node_name].append(OpcUaROSService(service, srv_node,
                                                               nodeid_generator(self.idx), self.ros_msgs))
        # action server
        if self._is_action(node_content['subs']):
            self._node_items[node_name].append(OpcUaROSActionServer(self.idx, node_name, ua_node, self.ros_msgs))
        # action client
        elif self._is_action(node_content['pubs']):
            self._node_items[node_name].append(OpcUaROSActionClient(self.idx, node_name, ua_node, self.ros_msgs))
        # normal topics
        else:
            pub_node = ua_node.add_folder(nodeid_generator(self.idx), 'Publications')
            for publish in node_content['pubs']:
                if publish == '/rosout':  # Take rosout away
                    continue
                self._node_items[node_name].append(OpcUaROSTopicPub(publish, pub_node,
                                                                    self.ros_msgs, nodeid_generator(self.idx)))
            sub_node = ua_node.add_folder(nodeid_generator(self.idx), 'Subscriptions')
            for subscribe in node_content['subs']:
                sub = OpcUaROSTopicSub(subscribe, sub_node, self.ros_msgs, nodeid_generator(self.idx),
                                       nodeid_generator(self.idx))
                self._node_items[node_name].append(sub)

    def create_nodes(self):
        ros_nodes = self.get_nodes_info(self.ros_node_name)
        self._ros_nodes = ros_nodes
        for name, content in ros_nodes.items():
            self._create_node(name, content)

    def refresh_nodes(self):
        rosnode_cleanup()
        current_nodes = self.get_nodes_info(self.ros_node_name)
        if current_nodes != self._ros_nodes:
            delete_dict = {name: content for name, content in self._ros_nodes.items() if name not in current_nodes}
            add_dict = {name: content for name, content in current_nodes.items() if name not in self._ros_nodes}
            for item in delete_dict:
                self._delete_node(item)
            for name, content in add_dict.items():
                self._create_node(name, content)
            self._ros_nodes = current_nodes


if __name__ == '__main__':
    try:
        with ROSServer() as ua_server:
            ua_server.load_messages()
            ua_server.create_nodes()
            ua_server.start_server()
            while not rospy.is_shutdown():
                ua_server.refresh_nodes()
                time.sleep(0.5)
    except Exception as e:
        print(e.message)
    # ua_server = ROSServer()
    # ua_server.load_messages()
    # ua_server.create_nodes()
    # ua_server.start_server()
    # rospy.init_node('rosopcua', log_level=rospy.INFO)
    # while not rospy.is_shutdown():
    #     ua_server.refresh_nodes()
    #     time.sleep(0.5)
