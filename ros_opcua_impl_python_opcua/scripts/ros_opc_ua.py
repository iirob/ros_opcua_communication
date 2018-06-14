from datetime import datetime
from opcua import ua

ROS_BUILD_IN_DATA_TYPES = {'bool': [False, ua.ObjectIds.Boolean],
                           'int8': [0, ua.ObjectIds.SByte],
                           'byte': [0, ua.ObjectIds.SByte],  # deprecated int8
                           'uint8': [0, ua.ObjectIds.Byte],
                           'char': [0, ua.ObjectIds.Byte],  # deprecated uint8
                           'int16': [0, ua.ObjectIds.Int16],
                           'uint16': [0, ua.ObjectIds.UInt16],
                           'int32': [0, ua.ObjectIds.Int32],
                           'uint32': [0, ua.ObjectIds.UInt32],
                           'int64': [0, ua.ObjectIds.Int64],
                           'uint64': [0, ua.ObjectIds.UInt64],
                           'float32': [0, ua.ObjectIds.Float],
                           'float64': [0, ua.ObjectIds.Float],
                           'string': ['', ua.ObjectIds.String],
                           'time': [datetime.utcnow(), ua.ObjectIds.DateTime],
                           'duration': [0, ua.ObjectIds.Duration]}

ID_COUNTER = 0


def _get_counter():
    global ID_COUNTER
    ID_COUNTER += 1
    return ID_COUNTER


def _process_ros_array(array_length, ua_node):
    """
    NOTE: ROS only support 1 dimensional array, therefore the array dimension list has only
     one member, array_length = 0 indicates an array with variable length
    :param array_length: None represents a scalar
    :param ua_node: node to be processed
    :return
    """
    if array_length is not None:
        ua_node.set_value_rank(ua.ValueRank.OneDimension)
        ua_node.set_array_dimensions([array_length])
    else:
        ua_node.set_value_rank(ua.ValueRank.Scalar)


def create_ros_variable(parent, nodeid, bname, variable_type_id, data_type_id, array_length):
    val = ua.Variant(ua.VariantType.Null)
    new_node = parent.add_variable(nodeid, bname, val, datatype=data_type_id, custom_variable_type=variable_type_id)
    new_node.set_modelling_rule(True)
    return new_node


def create_ros_property(parent, nodeid, bname, array_length, val, data_type):
    new_node = parent.add_property(nodeid, bname, val, data_type, data_type)
    new_node.set_modelling_rule(True)
    return new_node


def create_ros_data_type(parent, nodeid, bname, is_abstract=False):
    new_node = parent.add_data_type(nodeid, bname, is_abstract=is_abstract)
    return new_node


def create_ros_variable_type(parent, nodeid, bname, data_type, is_abstract=False):
    val = ua.Variant(ua.VariantType.Null)
    new_node = parent.add_variable_type(nodeid, bname, data_type, val, is_abstract)
    return new_node


def create_ros_object_type(parent, nodeid, bname, is_abstract=False):
    new_node = parent.add_object_type(nodeid, bname, is_abstract)
    return new_node


def nodeid_generator(idx):
    return ua.NodeId(_get_counter(), idx)
