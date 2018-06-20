from opcua import ua
from opcua.ua import *

import xml.etree.ElementTree as Et
import re

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
        appended_struct.attrib["Name"] = to_camel_case(name)
        return appended_struct

    @staticmethod
    def add_field(type_name, variable_name, struct_node):
        if type_name in ROS_BUILD_IN_DATA_TYPES:
            if type_name == 'string':
                type_name = 'opc:CharArray'
            else:
                type_name = 'opc:' + getattr(ROS_BUILD_IN_DATA_TYPES[type_name], '_name_')
        else:
            type_name = 'tns:' + to_camel_case(type_name)
        field = Et.SubElement(struct_node, 'opc:Field')
        field.attrib["Name"] = variable_name
        field.attrib["TypeName"] = type_name

    def get_dict_value(self):
        indent(self.etree.getroot())
        Et.dump(self.etree.getroot())
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


class DictionaryBuilder:
    def __init__(self, server, idx):
        self.server = server
        self.idx = idx
        self.dict = None

    def add_dictionary(self):
        node = ua.AddNodesItem()
        node.RequestedNewNodeId = NumericNodeId(7617, 0)
        node.BrowseName = QualifiedName('ROSDictionary', 0)
        node.NodeClass = NodeClass.Variable
        node.ParentNodeId = NumericNodeId(93, 0)
        node.ReferenceTypeId = NumericNodeId(47, 0)
        node.TypeDefinition = NumericNodeId(72, 0)
        attrs = ua.VariableAttributes()
        attrs.DisplayName = LocalizedText("Opc.Ua")
        attrs.DataType = ua.NodeId(ua.ObjectIds.ByteString)
        attrs.ValueRank = -1
        node.NodeAttributes = attrs
        self.server.add_nodes([node])

    def add_dict_description(self):
        node = ua.AddNodesItem()
        node.RequestedNewNodeId = NumericNodeId(7692, 0)
        node.BrowseName = QualifiedName('BuildInfo', 0)
        node.NodeClass = NodeClass.Variable
        node.ParentNodeId = NumericNodeId(7617, 0)
        node.ReferenceTypeId = NumericNodeId(47, 0)
        node.TypeDefinition = NumericNodeId(69, 0)
        attrs = ua.VariableAttributes()
        attrs.DisplayName = LocalizedText("BuildInfo")
        attrs.DataType = ua.NodeId(ua.ObjectIds.String)
        attrs.Value = ua.Variant('BuildInfo', ua.VariantType.String)
        attrs.ValueRank = -1
        node.NodeAttributes = attrs
        self.server.add_nodes([node])
        refs = []
        ref = ua.AddReferencesItem()
        ref.IsForward = True
        ref.ReferenceTypeId = NumericNodeId(40, 0)
        ref.SourceNodeId = NumericNodeId(7692, 0)
        ref.TargetNodeClass = NodeClass.DataType
        ref.TargetNodeId = NumericNodeId(69, 0)
        refs.append(ref)
        ref = ua.AddReferencesItem()
        ref.IsForward = False
        ref.ReferenceTypeId = NumericNodeId(47, 0)
        ref.SourceNodeId = NumericNodeId(7692, 0)
        ref.TargetNodeClass = NodeClass.DataType
        ref.TargetNodeId = NumericNodeId(7617, 0)
        refs.append(ref)
        self.server.add_references(refs)



def repl_func(m):
    """
    process regular expression match groups for word upper-casing problem taken from
     https://stackoverflow.com/questions/1549641/how-to-capitalize-the-first-letter-of-each-word-in-a-string-python
     """
    return m.group(1) + m.group(2).upper()


def to_camel_case(name):
    """
    Create python class name from ROS message/service strings with package name, class name is in CamelCase
    e.g.                 actionlib/TestAction -> ActionlibTestAction
         turtle_actionlib/ShapeActionFeedback -> TurtleActionlibShapeActionFeedback
    """
    name = re.sub(r'[^a-zA-Z0-9]+', ' ', name)
    name = re.sub('(^|\s)(\S)', repl_func, name)
    name = re.sub(r'[^a-zA-Z0-9]+', '', name)
    return name
