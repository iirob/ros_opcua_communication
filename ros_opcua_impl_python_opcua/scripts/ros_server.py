#!/usr/bin/python
import time

from ros_global import BasicROSServer, rosnode_cleanup
from ros_opc_ua_comm import *


class ROSServer(BasicROSServer):

    def __init__(self):
        BasicROSServer.__init__(self)
        self._ros_nodes = {}
        self._ua_nodes = {}
        self._node_items = {}

    @staticmethod
    def _is_action(names):
        for name in names:
            if name.split('/')[-1] in ['cancel', 'goal', 'result', 'feedback', 'status']:
                return True
        return False

    def nodeid_generator(self):
        return ua.NodeId(namespaceidx=self.idx)

    def _load_messages(self):
        if self.import_xml_msgs:
            # FIXME: bugs after import, the customized object can not be used.
            self.ros_msgs = self.import_messages()
        else:
            self.ros_msgs = OpcUaROSMessage(self.server, self.idx, self._idx_name).create_ros_data_types()

        self.server.load_type_definitions()

    def _delete_node(self, node_name):
        for node in self._node_items[node_name]:
            node.delete_node()
        self._ua_nodes[node_name].delete()
        del self._ua_nodes[node_name]
        del self._node_items[node_name]

    def _create_node(self, node_name, node_content):
        ua_node = self.server.nodes.objects.add_object(self.nodeid_generator(), node_name)
        self._ua_nodes[node_name] = ua_node
        self._node_items[node_name] = []
        # services
        srv_node = ua_node.add_folder(self.nodeid_generator(), 'Services')
        for service in node_content['srvs']:
            self._node_items[node_name].append(OpcUaROSService(service, srv_node,
                                                               self.nodeid_generator(), self.ros_msgs))
        # normal topics
        pub_node = ua_node.add_folder(self.nodeid_generator(), 'Publications')
        for publish in node_content['pubs']:
            if publish == '/rosout':  # Take rosout away
                continue
            self._node_items[node_name].append(OpcUaROSTopicPub(publish, pub_node,
                                                                self.ros_msgs, self.nodeid_generator()))
        sub_node = ua_node.add_folder(self.nodeid_generator(), 'Subscriptions')
        for subscribe in node_content['subs']:
            sub = OpcUaROSTopicSub(subscribe, sub_node, self.ros_msgs, self.nodeid_generator(),
                                   self.nodeid_generator())
            self._node_items[node_name].append(sub)
        # action
        if node_content['acts']:
            # action server
            if node_content['acts']['is_action_server']:
                self._node_items[node_name].append(OpcUaROSActionServer(self.idx, node_name, ua_node,
                                                                        self.ros_msgs, node_content['acts']))
            # action client
            else:
                self._node_items[node_name].append(OpcUaROSActionClient(self.idx, node_name, ua_node,
                                                                        self.ros_msgs, node_content['acts']))

    def _create_nodes(self):
        rosnode_cleanup()
        ros_nodes = self._get_ros_nodes()
        self._ros_nodes = ros_nodes
        for name, content in ros_nodes.items():
            self._create_node(name, content)

    def _refresh_nodes(self):
        rosnode_cleanup()
        current_nodes = self._get_ros_nodes()
        if current_nodes != self._ros_nodes:
            delete_dict = {name: content for name, content in self._ros_nodes.items() if name not in current_nodes}
            add_dict = {name: content for name, content in current_nodes.items() if name not in self._ros_nodes}
            for node in delete_dict:
                self._delete_node(node)
            for name, content in add_dict.items():
                self._create_node(name, content)
            self._ros_nodes = current_nodes

    def initialize_server(self):
        self._load_messages()
        self._create_nodes()
        self._start_server()

    def auto_refresh_nodes(self):
        while not rospy.is_shutdown():
            self._refresh_nodes()
            time.sleep(self.refresh_cycle_time)

    def manuel_refresh_nodes(self):
        """
        for manual refresh please call this method manually
        :return:
        """
        self._refresh_nodes()

    def refresh_nodes(self):
        if self.auto_refresh:
            self.auto_refresh_nodes()
        else:
            # here only refresh once, please take care!
            self.manuel_refresh_nodes()


if __name__ == '__main__':
    try:
        with ROSServer() as ua_server:
            ua_server.initialize_server()
            ua_server.refresh_nodes()
    except Exception as e:
        print(e.message)
    # For debugging
    # ua_server = ROSServer()
    # ua_server.initialize_server()
    # rospy.init_node('rosopcua', log_level=rospy.INFO)
    # ua_server.refresh_nodes()
