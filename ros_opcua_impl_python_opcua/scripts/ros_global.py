import rosgraph
import rosnode
import rospy
import rosmsg
import rospkg

from opcua import ua, Server

message_export_path = 'message.xml'

_action_feature_list = ['cancel', 'goal', 'result', 'feedback', 'status']


def _get_ros_packages(mode):
    """
    same as the command line 'rosmsg packages'
    :return: ROS messages as a list
    """
    return sorted([x for x in rosmsg.iterate_packages(rospkg.RosPack(), mode)])


def _get_ros_msg(mode):
    ret = []
    if mode == rosmsg.MODE_MSG:
        suffix = 'msg'
    else:
        suffix = 'srv'
    ros_packages = _get_ros_packages(mode)
    for (p, directory) in ros_packages:
        for file_name in getattr(rosmsg, '_list_types')(directory, suffix, mode):
            ret.append(p + '/' + file_name)
    return ret


def get_ros_messages():
    """
    same as the command line 'rosmsg list'
    :return: list of ros package/message pairs
    """
    return _get_ros_msg(rosmsg.MODE_MSG)


def get_ros_services():
    """
    same as the command line 'rossrv list'
    :return: list of ros package/service pairs
    """
    return _get_ros_msg(rosmsg.MODE_SRV)


def rosnode_cleanup():
    _, unpinged = rosnode.rosnode_ping_all()
    if unpinged:
        master = rosgraph.Master(rosnode.ID)
        rosnode.cleanup_master_blacklist(master, unpinged)


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
    action_list = {n.split('/')[-1]: n for n in name_list if n.split('/')[-1] in _action_feature_list}
    action_list['is_action_server'] = True if _is_action(sub_list, ['goal', 'cancel']) else False
    return action_list


def _del_action(name_list):
    return [n for n in name_list if n.split('/')[-1] not in _action_feature_list]


class BasicROSServer:

    def __init__(self):
        self.server = Server()

        self.server.set_endpoint('opc.tcp://0.0.0.0:21554/RosServer')
        self.server.set_server_name('ROS UA Server')
        self._idx_name = 'http://ros.org/rosopcua'
        self.idx = self.server.register_namespace(self._idx_name)
        self.ros_node_name = 'rosopcua'
        self.ros_msgs = None
        self.server_started = False

        self.namespace_ros = rospy.get_param('/rosopcua/namespace')
        self.auto_refresh = rospy.get_param('/rosopcua/automatic_refresh')
        self.refresh_cycle_time = rospy.get_param('/rosopcua/refresh_cycle_time')
        self.import_xml_msgs = rospy.get_param('/rosopcua/import_xml_msgs')
        self.g_ns = rosgraph.names.make_global_ns(self.namespace_ros)

    def __enter__(self):
        rospy.init_node(self.ros_node_name, log_level=rospy.INFO)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.server_started:
            self.server.stop()
        quit()

    def _nodeid_generator(self):
        return ua.NodeId(namespaceidx=self.idx)

    def _start_server(self):
        self.server.start()
        self.server_started = True
        rospy.loginfo(' ----- Server started! ------ ')

    def import_messages(self):
        rospy.loginfo(' ----- start importing node message to xml ------ ')
        nodes = self.server.import_xml(message_export_path)
        rospy.loginfo(' ----- %s nodes are imported ------ ' % len(nodes))
        type_dict = {self.server.get_node(node).get_display_name().Text: node for node in nodes}
        return type_dict

    def _extract_content(self, info, node):
        return sorted([t for t, l in info if node in l and (t == self.namespace_ros or t.startswith(self.g_ns))])

    def _get_ros_nodes(self):

        master = rosgraph.Master(self.ros_node_name)
        state = master.getSystemState()

        nodes = []
        for s in state:
            for t, l in s:
                nodes.extend(l)
        # Take rosout and self away from display
        nodes = [node for node in list(set(nodes)) if node not in ['/rosout', '/' + self.ros_node_name]]
        nodes_info_dict = {}
        for node in nodes:
            node_info = {'pubs': self._extract_content(state[0], node),
                         'subs': self._extract_content(state[1], node),
                         'srvs': self._extract_content(state[2], node),
                         'acts': {}}
            if _is_action(node_info['pubs'], _action_feature_list):
                node_info['acts'] = _add_action(node_info['pubs'], node_info['subs'])
                node_info['pubs'] = _del_action(node_info['pubs'])
                node_info['subs'] = _del_action(node_info['subs'])
            nodes_info_dict[node] = node_info
        return nodes_info_dict
