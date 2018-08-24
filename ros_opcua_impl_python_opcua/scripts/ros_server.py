#!/usr/bin/python
import time
import rospy
import rosnode
import rosgraph

from opcua import ua
from ros_opc_ua_comm import OpcUaROSService, OpcUaROSTopic, BasicROSServer


def _rosnode_cleanup():
    _, unpinged = rosnode.rosnode_ping_all()
    if unpinged:
        master = rosgraph.Master(rosnode.ID)
        rosnode.cleanup_master_blacklist(master, unpinged)


class ROSServer(BasicROSServer):

    def __init__(self):
        BasicROSServer.__init__(self)
        root = self._server.nodes.objects
        self._msg_folder = root.add_folder(self._nodeid_generator(), 'rosmsg')
        self._srv_folder = root.add_folder(self._nodeid_generator(), 'rossrv')
        self._service_folder = root.add_folder(self._nodeid_generator(), 'rosservice')
        self._topic_folder = root.add_folder(self._nodeid_generator(), 'rostopic')
        self._topic_pub_folder = self._topic_folder.add_folder(self._nodeid_generator(), 'topic publish')
        self._node_folder = root.add_folder(self._nodeid_generator(), 'rosnode')
        self._param_folder = root.add_folder(self._nodeid_generator(), 'rosparam')

        self._service_nodes = {}
        self._service_ua_nodes = {}
        self._topic_nodes = {}
        self._topics_status_ua_node = {}
        self._topics_publish_ua_node = {}
        self._ros_nodes = {}
        self._nodes_previous = None
        self._param_ua_nodes = {}
        self._params_previous = None

    def _load_messages(self):
        if self.import_xml_msgs:
            # FIXME: 1. bugs after import, extension object can not be used
            # FIXME: 2. should compare in system to get correct msgs and srvs
            self._type_list = self.import_messages()
        else:
            self.load_messages()

        self._server.load_type_definitions()

    def _create_static_info(self):
        """
        show rosmsg and rossrv in ua server
        :return:
        """
        for msg in self._ros_msgs:
            self._msg_folder.add_reference(self._ros_msgs[msg], ua.ObjectIds.Organizes)
        for srv in self._ros_srvs:
            self._srv_folder.add_reference(self._ros_srvs[srv], ua.ObjectIds.Organizes)

    def _create_service(self, name):
        service = OpcUaROSService(name, self._service_folder, self._nodeid_generator(), self._type_list)
        self._service_nodes[name] = service
        self._service_ua_nodes[name] = service.get_node()

    def _delete_service(self, name):
        self._service_nodes[name].delete_node()
        del self._service_nodes[name]
        del self._service_ua_nodes[name]

    def _create_topic(self, name):
        new_topic = OpcUaROSTopic(name, self._topic_folder, self._topic_pub_folder, self._type_list,
                                  self._nodeid_generator(), self._nodeid_generator())
        self._topic_nodes[name] = new_topic
        self._topics_status_ua_node[name] = new_topic.get_status_node()
        self._topics_publish_ua_node[name] = new_topic.get_publish_node()

    def _delete_topic(self, name):
        self._topic_nodes[name].delete_node()
        del self._topic_nodes[name]
        del self._topics_status_ua_node[name]
        del self._topics_publish_ua_node[name]

    def _link_services(self, ua_node, node_name, content):
        srv_node = ua_node.add_folder(self._nodeid_generator(), 'Services')
        for service in content:
            srv_node.add_reference(self._service_ua_nodes[service], ua.ObjectIds.Organizes)
        self._ros_nodes[node_name].append(srv_node)

    def _link_topics(self, ua_node, node_name, content_pub, content_sub):
        pub_node = ua_node.add_folder(self._nodeid_generator(), 'Publications')
        for publish in content_pub:
            pub_node.add_reference(self._topics_status_ua_node[publish], ua.ObjectIds.Organizes)
        sub_node = ua_node.add_folder(self._nodeid_generator(), 'Subscriptions')
        sub_node_publish = sub_node.add_folder(self._nodeid_generator(), 'topic publish')
        for subscribe in content_sub:
            sub_node.add_reference(self._topics_status_ua_node[subscribe], ua.ObjectIds.Organizes)
            sub_node_publish.add_reference(self._topics_publish_ua_node[subscribe], ua.ObjectIds.Organizes)
        self._ros_nodes[node_name].append(pub_node)
        self._ros_nodes[node_name].append(sub_node)
        self._ros_nodes[node_name].append(sub_node_publish)

    def _link_action(self, ua_node, node_name, content):
        action_node = ua_node.add_folder(self._nodeid_generator(), 'Action')
        self._ros_nodes[node_name].append(action_node)
        for topic in content['topics']:
            action_node.add_reference(self._topics_status_ua_node[topic], ua.ObjectIds.Organizes)
        # action server, with extra publish
        if content['is_action_server']:
            action_publish = action_node.add_folder(self._nodeid_generator(), 'topic publish')
            self._ros_nodes[node_name].append(action_publish)
            for topic in content['topics']:
                if 'goal' in topic or 'cancel' in topic:
                    action_publish.add_reference(self._topics_publish_ua_node[topic], ua.ObjectIds.Organizes)

    def _create_node(self, node_name, node_content):
        ua_node = self._node_folder.add_folder(self._nodeid_generator(), node_name)
        self._ros_nodes[node_name] = [ua_node]
        self._link_services(ua_node, node_name, node_content['srvs'])
        self._link_topics(ua_node, node_name, node_content['pubs'], node_content['subs'])
        if node_content['acts']:
            self._link_action(ua_node, node_name, node_content['acts'])
        rospy.loginfo('Created ROS Node: ' + node_name)

    def _delete_node(self, node_name):
        for node in self._ros_nodes[node_name]:
            node.delete()
        del self._ros_nodes[node_name]
        rospy.loginfo('Deleted ROS Node: ' + node_name)

    def _create_param(self, param_name, param_value, param_folder, ua_node_buffer):
        if isinstance(param_value, dict):
            param_folder = param_folder.add_folder(self._nodeid_generator(), param_name)
            ua_node_buffer.append(param_folder)
            for p, v in param_value.items():
                self._create_param(p, v, param_folder, ua_node_buffer)
        else:
            ua_node_buffer.append(param_folder.add_property(self._nodeid_generator(), param_name, param_value))
            rospy.loginfo('Created ROS Parameter: ' + param_name)

    def _delete_param(self, param_name):
        """
        Maybe this method will never be called.
        :param param_name:
        :return:
        """
        for node in self._param_ua_nodes[param_name]:
            node.delete()
        del self._param_ua_nodes[param_name]
        rospy.loginfo('Deleted ROS Parameter: ' + param_name)

    def _create_dynamic_info(self):
        _rosnode_cleanup()
        self._nodes_previous, ros_services, ros_topics, self._params_previous = self._get_ros_info()
        for service in ros_services:
            self._create_service(service)
        for topic in ros_topics:
            self._create_topic(topic)
        for name, content in self._nodes_previous.items():
            self._create_node(name, content)
        for name, content in self._params_previous.items():
            self._param_ua_nodes[name] = []
            self._create_param(name, content, self._param_folder, self._param_ua_nodes[name])

    def _refresh_services(self, service_list):
        if service_list != self._service_nodes.keys():
            delete_list = [name for name in self._service_nodes if name not in service_list]
            add_list = [name for name in service_list if name not in self._service_nodes]
            for node in delete_list:
                self._delete_service(node)
            for node in add_list:
                self._create_service(node)

    def _refresh_topics(self, topic_list):
        if topic_list != self._topic_nodes.keys():
            delete_list = [name for name in self._topic_nodes if name not in topic_list]
            add_list = [name for name in topic_list if name not in self._topic_nodes]
            for node in delete_list:
                self._delete_topic(node)
            for node in add_list:
                self._create_topic(node)

    def _refresh_nodes(self, current_nodes):
        if current_nodes != self._nodes_previous:
            delete_list = [name for name in self._nodes_previous if name not in current_nodes]
            add_dict = {name: content for name, content in current_nodes.items() if name not in self._nodes_previous}
            for node in delete_list:
                self._delete_node(node)
            for name, content in add_dict.items():
                self._create_node(name, content)
            self._nodes_previous = current_nodes

    def _refresh_param(self, param_list):
        if param_list != self._params_previous:
            delete_list = [name for name in self._param_ua_nodes if name not in param_list]
            add_dict = {name: content for name, content in param_list.items() if name not in self._param_ua_nodes}
            for node in delete_list:
                self._delete_param(node)
            for name, content in add_dict.items():
                self._param_ua_nodes[name] = []
                self._create_param(name, content, self._param_folder, self._param_ua_nodes[name])
            self._params_previous = param_list

    def _refresh_info(self):
        _rosnode_cleanup()
        current_nodes, current_services, current_topics, current_params = self._get_ros_info()
        self._refresh_services(current_services)
        self._refresh_topics(current_topics)
        self._refresh_nodes(current_nodes)
        self._refresh_param(current_params)

    def _auto_refresh_info(self):
        while not rospy.is_shutdown():
            self._refresh_info()
            time.sleep(self.refresh_cycle_time)

    def initialize_server(self):
        self._load_messages()
        self._create_static_info()
        self._create_dynamic_info()
        self._start_server()

    def refresh(self):
        """
        for manual refresh please call this method manually
        :return:
        """
        if self.auto_refresh:
            self._auto_refresh_info()
        else:
            self._refresh_info()


if __name__ == '__main__':
    try:
        with ROSServer() as ua_server:
            ua_server.initialize_server()
            ua_server.refresh()
    except Exception as e:
        print(e.message)
    # For debugging
    # ua_server = ROSServer()
    # ua_server.initialize_server()
    # rospy.init_node('rosopcua', log_level=rospy.INFO)
    # ua_server.refresh()
