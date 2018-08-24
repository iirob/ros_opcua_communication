import rospy

from opcua import ua, Server, Client
from ros_opc_ua_comm import OpcUaROSMessage

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


class SubHandler(object):

    def datachange_notification(self, node, val, data):
        print("Python: New data change event", node, val)


class ROSBasicClient:

    def __init__(self):
        self._client = Client(server_end_point)
        self._ros_node_name = 'opcua-client'
        self._idx = None
        self._node_root = None

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

    def _subscribe_topics(self):
        pass

    def list_topics(self):
        pass

    def show_topic(self, topic_name):
        pass

    def show_topics(self):
        pass

    def publish_topic(self, topic_name):
        pass

    def call_service(self, service_name):
        pass

    def list_params(self):
        pass

    def show_param(self, param_name):
        pass

    def show_params(self):
        pass

    def list_ros_nodes(self):
        pass

    def show_ros_node(self, node_name):
        pass

    def show_ros_nodes(self):
        pass
