# GLOBAL VARIABLES AND FUNCTIONS

# Global functions
from opcua import ua
import rospy

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


# created UA nodes in UA Server, only the ROS related nodes
package_node_created = {}
# retrieved ROS package names
packages = []

# ros messages  'message' --> nodeVariableType
messageExportPath = 'message.xml'
messageNode = {}

# ros_messages 'message' --> nodeDataType
dataTypeNode = {}

# ros Topics  'topic_name' --> 'topic_node'
topicNode = {}

# BaseDataType
# BaseDataVariableType

# baseDataVariableType_node = ;
