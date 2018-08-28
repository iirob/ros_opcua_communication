import rospy

from opcua import ua, Server, Client
from opcua.ua import UaError
from ros_opc_ua_comm import OpcUaROSMessage, expand_ua_class

message_export_path = 'message.xml'
server_end_point = 'opc.tcp://0.0.0.0:21554/ROSServer'
ros_idx_name = 'http://ros.org/rosopcua'


class ROSBasicServer:

    def __init__(self):
        self._server = Server()

        self._server.set_endpoint(server_end_point)
        self._server.set_server_name('ROS UA Server')
        self._idx_name = ros_idx_name
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
        return ua.NodeId(namespaceidx=self._idx)

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


def _traverse_param(param_node, level=0, show_value=False):
    if param_node.get_type_definition().Identifier == ua.ObjectIds.FolderType:
        buff = '\n{}{}'.format(level * '\t', param_node.get_display_name().Text)
        for node in param_node.get_children(refs=ua.ObjectIds.Organizes):
            buff += _traverse_param(node, level + 1, show_value)
        for node in param_node.get_children(refs=ua.ObjectIds.HasProperty):
            # No extension object in parameters
            buff += '\n{}{}'.format((level + 1)*'\t', node.get_display_name().Text)
            if show_value:
                buff += ': {}'.format(node.get_value())
        return buff
    else:
        buff = '{}{}'.format(level * '\t', param_node.get_display_name().Text)
    if show_value:
        buff += ': {}'.format(param_node.get_value())
    return buff


class ROSBasicClient:

    def __init__(self):
        self._client = Client(server_end_point)
        self._ros_node_name = 'opcuaclient'
        self._idx = None
        self._node_root = None
        self._msgs = {}
        self._srvs = {}
        self._topics = {}
        self._pub_topics = {}
        self._pub_parent = None
        self._services = {}
        self._service_parent = None
        self._params = {}
        self._nodes = {}

    def __enter__(self):
        rospy.init_node(self._ros_node_name, log_level=rospy.INFO)
        self._client.connect()
        self._idx = self._client.get_namespace_index(ros_idx_name)
        self._client.load_type_definitions()
        self._root = self._client.get_objects_node()
        rospy.loginfo(' ----- Client connected! ------ ')
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._client.disconnect()
        rospy.loginfo(' ----- Client disconnected! ------ ')

    @staticmethod
    def expand_value(obj):
        return expand_ua_class(obj)

    def _retrieve_ua_nodes(self, folder_name, refs=ua.ObjectIds.Organizes):
        temp_dict = {}
        msg_folder = self._root.get_child(folder_name)
        msgs = msg_folder.get_children(refs)
        for msg in msgs:
            msg_name = msg.get_display_name().Text
            temp_dict[msg_name] = msg
        return temp_dict

    def _refresh_msgs(self):
        self._msgs = self._retrieve_ua_nodes('rosmsg')

    def _refresh_svrs(self):
        self._srvs = self._retrieve_ua_nodes('rossrv')

    def _refresh_topics(self):
        topic_folder = 'rostopic'
        self._topics = self._retrieve_ua_nodes(topic_folder, refs=ua.ObjectIds.HasProperty)
        if not self._pub_parent:
            self._pub_parent = self._retrieve_ua_nodes(topic_folder).values()[0]
        pub_methods = self._pub_parent.get_children(refs=ua.ObjectIds.HasComponent)
        for method in pub_methods:
            msg_name = method.get_display_name().Text
            self._pub_topics[msg_name] = method

    def _refresh_services(self):
        services_folder_name = 'rosservice'
        if not self._service_parent:
            self._service_parent = self._root.get_child(services_folder_name)
        self._services = self._retrieve_ua_nodes(services_folder_name, refs=ua.ObjectIds.HasComponent)

    def _refresh_nodes(self):
        self._nodes = self._retrieve_ua_nodes('rosnode')

    def _refresh_params(self):
        self._params = self._retrieve_ua_nodes('rosparam')
        self._params.update(self._retrieve_ua_nodes('rosparam', refs=ua.ObjectIds.HasProperty))

    def list_msgs(self):
        if not self._msgs:
            self._refresh_msgs()
        for msg in self._msgs:
            rospy.loginfo(msg)

    def list_srvs(self):
        if not self._srvs:
            self._refresh_svrs()
        for srv in self._srvs:
            rospy.loginfo(srv)

    def list_services(self):
        self._refresh_services()
        for service in self._services:
            rospy.loginfo(service)

    def list_topics(self):
        self._refresh_topics()
        for topic in self._topics:
            rospy.loginfo(topic)

    def show_topic(self, topic_name):
        if topic_name not in self._topics:
            self._refresh_topics()
            if topic_name not in self._topics:
                raise Exception('Topic {} does not exist in server side!'.format(topic_name))
        try:
            topic_value = self._topics[topic_name].get_value()
            rospy.loginfo('{}:{}'.format(topic_name, self.expand_value(topic_value)))
        except UaError as e:
            rospy.logerr(e.message)
            del self._topics[topic_name]

    def show_topics(self):
        # In case the dict changed in iteration
        if not self._topics:
            self._refresh_topics()
        for topic in self._topics.keys():
            self.show_topic(topic)

    def list_params(self):
        self._refresh_params()
        for param in self._params:
            rospy.loginfo(_traverse_param(self._params[param]))

    def show_param(self, param_name):
        if not self._params:
            self._refresh_params()
        if param_name not in self._params:
            self._refresh_params()
            if param_name not in self._params:
                raise Exception('Parameter {} does not exist in server side!'.format(param_name))
        try:
            rospy.loginfo(_traverse_param(self._params[param_name], show_value=True))
        except UaError as e:
            rospy.logerr(e.message)
            del self._params[param_name]

    def show_params(self):
        if not self._params:
            self._refresh_params()
        # In case the dict changed in iteration
        for param in self._params.keys():
            self.show_param(param)

    def list_ros_nodes(self):
        self._refresh_nodes()
        for node in self._nodes:
            rospy.loginfo(node)

    def show_ros_node(self, node_name):
        if node_name not in self._nodes:
            self._refresh_nodes()
            if node_name not in self._nodes:
                raise Exception('Node {} does not exist in server side!'.format(node_name))
        try:
            rospy.loginfo('------ rosnode {} ------'.format(node_name))
            rospy.loginfo('\tPublications:')
            pubs = self._nodes[node_name].get_child('Publications').get_children(refs=ua.ObjectIds.Organizes)
            for pub in pubs:
                rospy.loginfo('\t\t{}:{}'.format(pub.get_display_name().Text, self.expand_value(pub.get_value())))
            rospy.loginfo('\tSubscriptions:')
            subs = self._nodes[node_name].get_child('Subscriptions').get_children(refs=ua.ObjectIds.Organizes)
            for sub in subs:
                sub_name = sub.get_display_name().Text
                if sub_name != 'topic publish':
                    rospy.loginfo('\t\t{}'.format(sub_name))
            rospy.loginfo('\tServices:')
            srvs = self._nodes[node_name].get_child('Services').get_children(refs=ua.ObjectIds.Organizes)
            for srv in srvs:
                rospy.loginfo('\t\t{}'.format(srv.get_display_name().Text))
        except UaError as e:
            rospy.logerr(e.message)
            del self._nodes[node_name]

    def show_ros_nodes(self):
        if not self._nodes:
            self._refresh_nodes()
        # In case the dict changed in iteration
        for node in self._nodes.keys():
            self.show_ros_node(node)

    def publish_topic(self, topic_name, ext_obj):
        if not self._pub_topics:
            self._refresh_topics()
        if topic_name not in self._pub_topics:
            raise Exception('Topic {} does not exist in server side!'.format(topic_name))
        topic_node = self._pub_topics[topic_name]
        result = self._pub_parent.call_method(topic_node, ext_obj)
        rospy.loginfo('Publish message of topic {} successful!'.format(topic_name))
        if result:
            rospy.loginfo(self.expand_value(result))

    def call_service(self, service_name, ext_obj=None):
        if not self._services:
            self._refresh_services()
        if service_name not in self._services:
            raise Exception('Topic {} does not exist in server side!'.format(service_name))
        service_node = self._services[service_name]
        if not ext_obj:
            ext_obj = []
        result = self._service_parent.call_method(service_node, ext_obj)
        rospy.loginfo('Call service {} successful!'.format(service_name))
        if result:
            rospy.loginfo(self.expand_value(result))
