from datetime import datetime

from opcua import ua
from opcua.common import node

ID_COUNTER = 0

ROS_BUILD_IN_DATA_TYPES = {'bool': [False, ua.ObjectIds.Boolean],
                           'int8': [0, ua.ObjectIds.SByte],
                           'byte': [0, ua.ObjectIds.SByte],  # deprecated int8
                           'uint8': [0, ua.ObjectIds.Byte],
                           'char': [0, ua.ObjectIds.SByte],  # deprecated uint8
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


def _get_counter():
    global ID_COUNTER
    ID_COUNTER += 1
    return ID_COUNTER


def _process_ros_array(array_length, attributes):
    """
    NOTE: ROS only support 1 dimensional array, therefore the array dimension list has only
     one member, array_length = 0 indicates a variable length of the arr
    :param array_length: None represents a scalar, 0 represents an array variable length
    :param attributes:
    :return:
    """
    if array_length is not None:
        attributes.ValueRank = ua.ValueRank.OneDimension
        attributes.ArrayDimensions = [array_length]
    else:
        attributes.ValueRank = ua.ValueRank.Scalar


def _create_variable(parent, nodeid, bname, variable_type_id, data_type_id, array_length, is_property=False):
    addnode = ua.AddNodesItem()
    addnode.RequestedNewNodeId = nodeid
    addnode.BrowseName = bname
    addnode.NodeClass = ua.NodeClass.Variable
    addnode.ParentNodeId = parent.nodeid
    if is_property:
        addnode.ReferenceTypeId = ua.NodeId(ua.ObjectIds.HasProperty)
        addnode.TypeDefinition = ua.NodeId(ua.ObjectIds.PropertyType)
    else:
        addnode.ReferenceTypeId = ua.NodeId(ua.ObjectIds.HasComponent)
        addnode.TypeDefinition = variable_type_id
    attrs = ua.VariableAttributes()
    attrs.Description = ua.LocalizedText(bname.Name)
    attrs.DisplayName = ua.LocalizedText(bname.Name)
    attrs.DataType = data_type_id
    _process_ros_array(array_length, attrs)
    attrs.WriteMask = 0
    attrs.UserWriteMask = 0
    attrs.Historizing = 0
    attrs.AccessLevel = ua.AccessLevel.CurrentRead.mask
    attrs.UserAccessLevel = ua.AccessLevel.CurrentRead.mask
    addnode.NodeAttributes = attrs
    results = parent.server.add_nodes([addnode])
    results[0].StatusCode.check()
    return node.Node(parent.server, results[0].AddedNodeId)


def create_variable(parent, nodeid, bname, variable_type_id, data_type_id, array_length):
    new_node = _create_variable(parent, nodeid, bname, variable_type_id, data_type_id, array_length)
    new_node.set_modelling_rule(True)
    return new_node


def create_property(parent, nodeid, bname, array_length, var, data_type):
    data_type = ua.NodeId(data_type, 0)
    new_node = _create_variable(parent, nodeid, bname, None, data_type, array_length, True)
    new_node.set_value(var)
    return new_node


def create_data_type(parent, nodeid, bname, is_abstract=False):
    addnode = ua.AddNodesItem()
    addnode.RequestedNewNodeId = nodeid
    addnode.BrowseName = bname
    addnode.NodeClass = ua.NodeClass.DataType
    addnode.ParentNodeId = parent.nodeid
    addnode.ReferenceTypeId = ua.NodeId(ua.ObjectIds.HasSubtype)
    attrs = ua.DataTypeAttributes()
    attrs.Description = ua.LocalizedText(bname.Name)
    attrs.DisplayName = ua.LocalizedText(bname.Name)
    attrs.IsAbstract = is_abstract

    attrs.WriteMask = 0
    attrs.UserWriteMask = 0

    addnode.NodeAttributes = attrs
    results = parent.server.add_nodes([addnode])
    results[0].StatusCode.check()
    return node.Node(parent.server, results[0].AddedNodeId)


def create_variable_type(parent, nodeid, bname, data_type_id):
    addnode = ua.AddNodesItem()
    addnode.RequestedNewNodeId = nodeid
    addnode.BrowseName = bname
    addnode.NodeClass = ua.NodeClass.VariableType
    addnode.ParentNodeId = parent.nodeid
    addnode.ReferenceTypeId = ua.NodeId(ua.ObjectIds.HasSubtype)
    attrs = ua.VariableTypeAttributes()
    attrs.Description = ua.LocalizedText(bname.Name)
    attrs.DisplayName = ua.LocalizedText(bname.Name)

    attrs.DataType = data_type_id
    attrs.IsAbstract = True
    _process_ros_array(None, attrs)
    attrs.WriteMask = 0
    attrs.UserWriteMask = 0

    addnode.NodeAttributes = attrs
    results = parent.server.add_nodes([addnode])
    results[0].StatusCode.check()
    return node.Node(parent.server, results[0].AddedNodeId)


def nodeid_generator(idx):
    return ua.NodeId(_get_counter(), idx)
