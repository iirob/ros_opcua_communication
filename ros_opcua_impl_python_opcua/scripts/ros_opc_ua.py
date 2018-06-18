from opcua import ua
from opcua.common import node

import xml.etree.ElementTree as Et

ROS_BUILD_IN_DATA_TYPES = {'bool': ua.VariantType.Boolean,
                           'int8': ua.VariantType.SByte,
                           'byte': ua.VariantType.SByte,  # deprecated int8
                           'uint8': ua.VariantType.Byte,
                           'char': ua.VariantType.Byte,  # deprecated uint8
                           'int16': ua.VariantType.Int16,
                           'uint16': ua.VariantType.UInt16,
                           'int32': ua.VariantType.Int32,
                           'uint32': ua.VariantType.UInt32,
                           'int64': ua.VariantType.Int64,
                           'uint64': ua.VariantType.UInt64,
                           'float32': ua.VariantType.Float,
                           'float64': ua.VariantType.Float,
                           'string': ua.VariantType.String,
                           'time': ua.VariantType.DateTime,
                           'duration': ua.VariantType.DateTime}


def process_ros_array(array_length, attributes):
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
    process_ros_array(array_length, attrs)
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
    # data_type = ua.NodeId(data_type, 0)
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
    process_ros_array(None, attrs)
    attrs.WriteMask = 0
    attrs.UserWriteMask = 0

    addnode.NodeAttributes = attrs
    results = parent.server.add_nodes([addnode])
    results[0].StatusCode.check()
    return node.Node(parent.server, results[0].AddedNodeId)


def nodeid_generator(idx):
    return ua.NodeId(namespaceidx=idx)


class TypeBinaryDictionary:

    def __init__(self, idx_name):
        head_attributes = {'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance', 'xmlns:tns': idx_name,
                           'DefaultByteOrder': 'LittleEndian', 'xmlns:opc': 'http://opcfoundation.org/BinarySchema/',
                           'xmlns:ua': 'http://opcfoundation.org/UA/', 'TargetNamespace': idx_name}

        self.etree = Et.ElementTree(Et.Element('opc:TypeDictionary', head_attributes))

        name_space = Et.SubElement(self.etree.getroot(), 'opc:Import')
        name_space.attrib['Namespace'] = 'http://opcfoundation.org/UA/'

    def append_struct(self, name):
        appended_struct = Et.SubElement(self.etree.getroot(), 'opc:StructuredType')
        appended_struct.attrib["BaseType"] = 'ua:ExtensionObject'
        appended_struct.attrib["Name"] = name
        return appended_struct

    @staticmethod
    def add_field(type_name, variable_name, struct_node):
        if type_name in ROS_BUILD_IN_DATA_TYPES:
            type_name = 'opc:' + getattr(ROS_BUILD_IN_DATA_TYPES[type_name], '_name_')
        else:
            type_name = 'tns:' + type_name
        field = Et.SubElement(struct_node, 'opc:Field')
        field.attrib["TypeName"] = type_name
        field.attrib["Name"] = variable_name

    def get_dict_value(self):
        indent(self.etree.getroot())
        # Et.dump(self.etree.getroot())
        return Et.tostring(self.etree.getroot(), encoding='utf-8')


def indent(elem, level=0):
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i
