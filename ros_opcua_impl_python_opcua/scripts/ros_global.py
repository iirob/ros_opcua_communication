# GLOBAL VARIABLES AND FUNCTIONS
import rosgraph
import rosnode
import rospy
import rosmsg
from rosmsg import *

from opcua import ua, Server

object_id_dict = {'bool': ua.ObjectIds.Boolean,
                  'byte': ua.ObjectIds.Byte,
                  'int': ua.ObjectIds.Int16,
                  'int8': ua.ObjectIds.SByte,
                  'uint8': ua.ObjectIds.Byte,
                  'int16': ua.ObjectIds.Int16,
                  'uint16': ua.ObjectIds.UInt16,
                  'int32': ua.ObjectIds.Int32,
                  'uint32': ua.ObjectIds.UInt32,
                  'int64': ua.ObjectIds.Int64,
                  'uint64': ua.ObjectIds.UInt64,
                  'float': ua.ObjectIds.Float,
                  'float32': ua.ObjectIds.Float,
                  'float64': ua.ObjectIds.Float,
                  'double': ua.ObjectIds.Double,
                  'string': ua.ObjectIds.String,
                  'str': ua.ObjectIds.String,
                  'array': ua.ObjectIds.Enumeration,
                  'Time': ua.ObjectIds.Time,
                  'time': ua.ObjectIds.Time}


def get_object_ids(type_name):
    if type_name == 'int16':
        rospy.logwarn('Int16??')
    dv = object_id_dict.get(type_name, None)
    if dv is None:
        rospy.logerr('Can not create type with name ' + type_name)
    return dv


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


def next_name(hierarchy, index_of_last_processed):
    """
    Returns the hierarchy as one string from the first remaining part on.
    :param hierarchy:
    :param index_of_last_processed:
    :return:
    """
    try:
        output = ''
        counter = index_of_last_processed + 1
        while counter < len(hierarchy):
            output += hierarchy[counter]
            counter += 1
        return output
    except Exception as ex:
        rospy.logerr('Error encountered ', ex)


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
        self._server = Server()

        self._server.set_endpoint('opc.tcp://0.0.0.0:21554/')
        self._server.set_server_name('ROS UA Server')

    def __enter__(self):
        self._server.start()
        rospy.init_node('rosopcua', log_level=rospy.INFO)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._server.stop()
        quit()


# created UA nodes in UA Server, only the ROS related nodes
package_node_created = {}
# retrieved ROS package names
packages = []

# ros messages  'message' --> nodeVariableType
messageExportPath = 'message.xml'
new_messageExportPath = 'new_message.xml'
messageNode = {}

# ros_messages 'message' --> nodeDataType
dataTypeNode = {}

# ros Topics  'topic_name' --> 'topic_node'
topicNode = {}

# BaseDataType
# BaseDataVariableType

# baseDataVariableType_node = ;

if __name__ == '__main__':
    print(get_ros_services())
