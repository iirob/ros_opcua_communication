import rospy
import rosgraph
import rosnode

from opcua import ua, Server
from ros_opc_ua_comm import OpcUaROSMessage, OpcUaROSService, OpcUaROSTopic

message_export_path = 'message.xml'
_action_feature_list = ('cancel', 'goal', 'result', 'feedback', 'status')


def _nodeid_generator(idx):
    return ua.NodeId(namespaceidx=idx)


class ROSServiceManager:

    def __init__(self, idx, node_root, type_dict):
        self._idx = idx
        self._type_dict = type_dict
        self._root = node_root.add_folder(_nodeid_generator(self._idx), 'rosservice')
        self._services = {}
        self._ua_nodes = {}

    def _create_service(self, name):
        service = OpcUaROSService(name, self._root, _nodeid_generator(self._idx), self._type_dict)
        self._services[name] = service
        self._ua_nodes[name] = service.get_node()

    def _delete_service(self, name):
        self._services[name].delete_node()
        del self._services[name]
        del self._ua_nodes[name]

    def refresh_services(self, service_list):
        if service_list != self._services.keys():
            delete_list = [name for name in self._services if name not in service_list]
            add_list = [name for name in service_list if name not in self._services]
            for node in delete_list:
                self._delete_service(node)
            for node in add_list:
                self._create_service(node)

    def get_node_dict(self):
        return self._ua_nodes


class ROSTopicManager:

    def __init__(self, idx, node_root, type_dict):
        self._idx = idx
        self._type_dict = type_dict
        self._root = node_root.add_folder(_nodeid_generator(self._idx), 'rostopic')
        self._pub_root = self._root.add_folder(_nodeid_generator(self._idx), 'topic publish')

        self._topics = {}
        self._status_ua_node = {}
        self._publish_ua_node = {}

    def _create_topic(self, name):
        new_topic = OpcUaROSTopic(name, self._root, self._pub_root, self._type_dict,
                                  _nodeid_generator(self._idx), _nodeid_generator(self._idx))
        self._topics[name] = new_topic
        self._status_ua_node[name] = new_topic.get_status_node()
        self._publish_ua_node[name] = new_topic.get_publish_node()

    def _delete_topic(self, name):
        self._topics[name].delete_node()
        del self._topics[name]
        del self._status_ua_node[name]
        del self._publish_ua_node[name]

    def refresh_topics(self, topic_list):
        if topic_list != self._topics.keys():
            delete_list = [name for name in self._topics if name not in topic_list]
            add_list = [name for name in topic_list if name not in self._topics]
            for node in delete_list:
                self._delete_topic(node)
            for node in add_list:
                self._create_topic(node)

    def get_status_ua_node(self):
        return self._status_ua_node

    def get_publish_ua_node(self):
        return self._publish_ua_node


class ROSNodeManager:

    def __init__(self, idx, node_root, type_dict, service_nodes, topic_status_nodes, topic_publish_nodes):
        self._idx = idx
        self._type_dict = type_dict
        self._root = node_root.add_folder(_nodeid_generator(self._idx), 'rosnode')
        self._service_nodes = service_nodes
        self._topic_status_nodes = topic_status_nodes
        self._topic_publish_nodes = topic_publish_nodes
        self._ua_nodes = {}
        self._nodes_previous = {}

    def _link_services(self, ua_node, node_name, content):
        if content:
            srv_node = ua_node.add_folder(_nodeid_generator(self._idx), 'Services')
            for service in content:
                srv_node.add_reference(self._service_nodes[service], ua.ObjectIds.Organizes)
            self._ua_nodes[node_name].append(srv_node)

    def _link_topics(self, ua_node, node_name, content_pub, content_sub):
        if content_pub:
            pub_node = ua_node.add_folder(_nodeid_generator(self._idx), 'Publications')
            for publish in content_pub:
                pub_node.add_reference(self._topic_status_nodes[publish], ua.ObjectIds.Organizes)
            self._ua_nodes[node_name].append(pub_node)
        if content_sub:
            sub_node = ua_node.add_folder(_nodeid_generator(self._idx), 'Subscriptions')
            sub_node_publish = sub_node.add_folder(_nodeid_generator(self._idx), 'topic publish')
            for subscribe in content_sub:
                sub_node.add_reference(self._topic_status_nodes[subscribe], ua.ObjectIds.Organizes)
                sub_node_publish.add_reference(self._topic_publish_nodes[subscribe], ua.ObjectIds.Organizes)
            self._ua_nodes[node_name].append(sub_node)
            self._ua_nodes[node_name].append(sub_node_publish)

    def _link_action(self, ua_node, node_name, content):
        if content:
            action_node = ua_node.add_folder(_nodeid_generator(self._idx), 'Action')
            for topic in content['topics']:
                action_node.add_reference(self._topic_status_nodes[topic], ua.ObjectIds.Organizes)
            self._ua_nodes[node_name].append(action_node)
            # action server, with extra publish
            if content['is_action_server']:
                action_publish = action_node.add_folder(_nodeid_generator(self._idx), 'topic publish')
                for topic in content['topics']:
                    if 'goal' in topic or 'cancel' in topic:
                        action_publish.add_reference(self._topic_publish_nodes[topic], ua.ObjectIds.Organizes)
                self._ua_nodes[node_name].append(action_publish)

    def _create_node(self, node_name, node_content):
        ua_node = self._root.add_folder(_nodeid_generator(self._idx), node_name)
        self._ua_nodes[node_name] = [ua_node]
        self._link_services(ua_node, node_name, node_content['srvs'])
        self._link_topics(ua_node, node_name, node_content['pubs'], node_content['subs'])
        self._link_action(ua_node, node_name, node_content['acts'])
        rospy.loginfo('Created ROS Node: ' + node_name)

    def _delete_node(self, node_name):
        for node in self._ua_nodes[node_name]:
            node.delete()
        del self._ua_nodes[node_name]
        rospy.loginfo('Deleted ROS Node: ' + node_name)

    def refresh_nodes(self, current_nodes):
        if current_nodes != self._nodes_previous:
            delete_list = [name for name in self._nodes_previous if name not in current_nodes]
            add_dict = {name: content for name, content in current_nodes.items() if name not in self._nodes_previous}
            for node in delete_list:
                self._delete_node(node)
            for name, content in add_dict.items():
                self._create_node(name, content)
            self._nodes_previous = current_nodes


class ROSParamManager:

    def __init__(self, idx, node_root):
        self._idx = idx
        self._root = node_root.add_folder(_nodeid_generator(self._idx), 'rosparam')

        self._ua_nodes = {}
        self._previous = {}

    def _create_param(self, param_name, param_value, param_folder, ua_node_buffer):
        if isinstance(param_value, dict):
            param_folder = param_folder.add_folder(_nodeid_generator(self._idx), param_name)
            ua_node_buffer.append(param_folder)
            for p, v in param_value.items():
                self._create_param(p, v, param_folder, ua_node_buffer)
        else:
            ua_node_buffer.append(param_folder.add_property(_nodeid_generator(self._idx), param_name, param_value))
            rospy.loginfo('Created ROS Parameter: ' + param_name)

    def _delete_param(self, param_name):
        """
        Maybe this method will never be called without manual intervention.
        :param param_name:
        :return:
        """
        for node in self._ua_nodes[param_name]:
            node.delete()
        del self._ua_nodes[param_name]
        rospy.loginfo('Deleted ROS Parameter: ' + param_name)

    def refresh_params(self, param_list):
        if param_list != self._previous:
            delete_list = [name for name in self._ua_nodes if name not in param_list]
            add_dict = {name: content for name, content in param_list.items() if name not in self._ua_nodes}
            for node in delete_list:
                self._delete_param(node)
            for name, content in add_dict.items():
                self._ua_nodes[name] = []
                self._create_param(name, content, self._root, self._ua_nodes[name])
            self._previous = param_list


def _is_action(name_list, feature_list):
    # at least two features should match
    feature_counter = 0
    for name in name_list:
        if name.split('/')[-1] in feature_list:
            feature_counter = feature_counter + 1
    if feature_counter >= 2:
        return True
    return False


def _add_action(pub_list, sub_list):
    name_list = pub_list + sub_list
    action_list = {'is_action_server': True if _is_action(sub_list, ('goal', 'cancel')) else False,
                   'topics': [n for n in name_list if n.split('/')[-1] in _action_feature_list]}
    return action_list


def _del_action(name_list):
    return [n for n in name_list if n.split('/')[-1] not in _action_feature_list]


class ROSInfoAgent:

    def __init__(self, node_name, ros_namespace):
        self._ros_node_name = node_name
        self._exclude_list = ('rosout', '/rosout', self._ros_node_name, '/' + self._ros_node_name)
        self._namespace = ros_namespace
        self._ns = rosgraph.names.make_global_ns(self._namespace)
        self._master = rosgraph.Master(self._ros_node_name)

    def _extract_info(self, info):
        return sorted([t for t, l in info if (t == self._namespace or t.startswith(self._ns))
                       and t not in self._exclude_list and not (len(l) == 1 and l[0] in self._exclude_list)])

    def _extract_node_info(self, info, node):
        return sorted([t for t, l in info if t not in self._exclude_list
                       and (t == self._namespace or t.startswith(self._ns))
                       and node in l and not (len(l) == 1 and l[0] in self._exclude_list)])

    def get_ros_info(self):
        state = self._master.getSystemState()
        params = {key: value for key, value in self._master.getParam(self._namespace).items()
                  if key not in self._exclude_list}

        nodes = []
        for s in state:
            for t, l in s:
                nodes.extend(l)
        nodes = [node for node in list(set(nodes)) if node.split('/')[-1] not in self._exclude_list]

        topics = self._extract_info(state[0])
        topics += self._extract_info(state[1])
        services = self._extract_info(state[2])

        nodes_info_dict = {}
        for node in nodes:
            node_info = {'pubs': self._extract_node_info(state[0], node),
                         'subs': self._extract_node_info(state[1], node),
                         'srvs': self._extract_node_info(state[2], node),
                         'acts': {}}
            if _is_action(node_info['pubs'], _action_feature_list):
                node_info['acts'] = _add_action(node_info['pubs'], node_info['subs'])
                node_info['pubs'] = _del_action(node_info['pubs'])
                node_info['subs'] = _del_action(node_info['subs'])
            nodes_info_dict[node] = node_info
        return nodes_info_dict, services, list(set(topics)), params

    def node_cleanup(self):
        _, unpinged = rosnode.rosnode_ping_all()
        if unpinged:
            rosnode.cleanup_master_blacklist(self._master, unpinged)


class ROSBasicServer:

    def __init__(self):
        self._server = Server()

        self._server.set_endpoint('opc.tcp://0.0.0.0:21554/ROSServer')
        self._server.set_server_name('ROS UA Server')
        self._idx_name = 'http://ros.org/rosopcua'
        self._idx = self._server.register_namespace(self._idx_name)
        self._ros_node_name = 'rosopcua'
        self._message_path = message_export_path
        self._server_started = False

        self._namespace_ros = rospy.get_param('/rosopcua/namespace')
        self._auto_refresh = rospy.get_param('/rosopcua/automatic_refresh')
        self._refresh_cycle_time = rospy.get_param('/rosopcua/refresh_cycle_time')
        self._import_xml_msgs = rospy.get_param('/rosopcua/import_xml_msgs')

        self._type_dict = {}
        self._msgs_dict = {}
        self._srvs_dict = {}

    def __enter__(self):
        rospy.init_node(self._ros_node_name, log_level=rospy.INFO)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._server_started:
            self._server.stop()
            rospy.loginfo(' ----- Server stopped! ------ ')

    def _nodeid_generator(self):
        return _nodeid_generator(self._idx)

    def _start_server(self):
        self._server.start()
        self._server_started = True
        rospy.loginfo(' ----- Server started! ------ ')

    def import_messages(self):
        rospy.loginfo(' ----- start importing node message to xml ------ ')
        nodes = self._server.import_xml(self._message_path)
        rospy.loginfo(' ----- {} nodes are imported ------ '.format(len(nodes)))
        type_dict = {self._server.get_node(node).get_display_name().Text: node for node in nodes}
        return type_dict

    def load_messages(self):
        rospy.loginfo(' ----- Creating messages ------ ')
        ros_type_creator = OpcUaROSMessage(self._server, self._idx, self._idx_name)
        self._type_dict, self._msgs_dict, self._srvs_dict = ros_type_creator.create_ros_data_types()
        rospy.loginfo(' ----- {} messages created------ '.format(str(len(self._type_dict))))
