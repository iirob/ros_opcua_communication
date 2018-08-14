import rosgraph
import rosnode
import rospy
import rosmsg
from rosmsg import *

from opcua import ua, Server
from opcua.common.ua_utils import get_nodes_of_namespace

MESSAGE_EXPORT_PATH = 'message.xml'


def _get_ros_packages(mode):
    """
    same as the command line 'rosmsg packages'
    :return: ROS messages as a list
    """
    return sorted([x for x in iterate_packages(rospkg.RosPack(), mode)])


def _get_ros_msg(mode):
    ret = []
    if mode == MODE_MSG:
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
    return _get_ros_msg(MODE_MSG)


def get_ros_services():
    """
    same as the command line 'rossrv list'
    :return: list of ros package/service pairs
    """
    return _get_ros_msg(MODE_SRV)


def get_ros_package(package_name):
    return list_types(package_name, mode=MODE_MSG)


def rosnode_cleanup():
    _, unpinged = rosnode.rosnode_ping_all()
    if unpinged:
        master = rosgraph.Master(rosnode.ID)
        rosnode.cleanup_master_blacklist(master, unpinged)


def correct_type(node, type_message):
    data_value = node.get_data_value()
    result = node.get_value()
    if isinstance(data_value, ua.DataValue):
        if type_message.__name__ in ('float', 'double'):
            return float(result)
        if type_message.__name__ == 'int':
            return int(result) & 0xff
        if type_message.__name__ in ('Time', 'Duration'):
            return rospy.Time(result)
    else:
        rospy.logerr("can't convert: " + str(node.get_data_value.Value))
        return None


class BasicROSServer:
    def __init__(self):
        self.server = Server()

        self.server.set_endpoint('opc.tcp://0.0.0.0:21554/RosServer')
        self.server.set_server_name('ROS UA Server')
        self._idx_name = 'http://ros.org/rosopcua'
        self.idx = self.server.register_namespace(self._idx_name)
        self.ros_node_name = 'rosopcua'
        self.ros_msgs = None

    def __enter__(self):
        rospy.init_node(self.ros_node_name, log_level=rospy.INFO)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.server.stop()
        quit()

    def start_server(self):
        self.server.start()

    def export_messages(self):
        rospy.logwarn(' ----- check if Extension Object fully supported! ------ ')
        rospy.loginfo(' ----- start exporting node message to xml ------ ')
        node_to_export = get_nodes_of_namespace(self.server, [self.idx])
        rospy.loginfo(' ----- %s nodes are to be exported ------ ' % len(node_to_export))
        self.server.export_xml(node_to_export, MESSAGE_EXPORT_PATH)
        rospy.loginfo(' ----- node message exported to %s ------ ' % MESSAGE_EXPORT_PATH)

    def get_nodes_info(self, node_name):
        master = rosgraph.Master(node_name)
        state = master.getSystemState()

        nodes = []
        for s in state:
            for t, l in s:
                nodes.extend(l)
        nodes = list(set(nodes))
        nodes_info_dict = {}
        for node in nodes:
            node_info = {'pubs': sorted([t for t, l in state[0] if node in l]),
                         'subs': sorted([t for t, l in state[1] if node in l]),
                         'srvs': sorted([t for t, l in state[2] if node in l])}
            # Take rosout and self away from display
            if node not in ['/rosout', '/' + self.ros_node_name]:
                nodes_info_dict[node] = node_info
        return nodes_info_dict
