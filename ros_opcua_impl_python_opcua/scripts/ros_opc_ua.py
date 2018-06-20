from opcua import ua

import xml.etree.ElementTree as Et
import re

IDENTIFIER_COUNTER = 1

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
    global IDENTIFIER_COUNTER
    IDENTIFIER_COUNTER += 1
    return ua.NodeId(IDENTIFIER_COUNTER, namespaceidx=idx)


class TypeDictionaryBinary:

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
    def __init__(self, server, idx, dict_name):
        self.server = server.get_root_node().server
        self.idx = idx
        self.dict_id = self._add_dictionary(dict_name)

    def _add_dictionary(self, name):
        dictionary_node_id = nodeid_generator(self.idx)
        node = ua.AddNodesItem()
        node.RequestedNewNodeId = dictionary_node_id
        node.BrowseName = ua.QualifiedName(name, self.idx)
        node.NodeClass = ua.NodeClass.Variable
        node.ParentNodeId = ua.NodeId(ua.ObjectIds.OPCBinarySchema_TypeSystem, 0)
        node.ReferenceTypeId = ua.NodeId(ua.ObjectIds.HasComponent, 0)
        node.TypeDefinition = ua.NodeId(ua.ObjectIds.DataTypeDictionaryType, 0)
        attrs = ua.VariableAttributes()
        attrs.DisplayName = ua.LocalizedText(name)
        attrs.DataType = ua.NodeId(ua.ObjectIds.ByteString)
        # Value should be set after all data types created
        attrs.Value = ua.Variant(None, ua.VariantType.Null)
        attrs.ValueRank = -1
        node.NodeAttributes = attrs
        self.server.add_nodes([node])

        return dictionary_node_id

    @staticmethod
    def _reference_generator(source_id, target_id, reference_type, is_forward=True):
        ref = ua.AddReferencesItem()
        ref.IsForward = is_forward
        ref.ReferenceTypeId = reference_type
        ref.SourceNodeId = source_id
        ref.TargetNodeClass = ua.NodeClass.DataType
        ref.TargetNodeId = target_id
        return ref

    def _link_nodes(self, linked_obj_node_id, data_type_node_id, description_node_id):
        refs = [
                # add reverse reference to BaseDataType -> Structure
                self._reference_generator(data_type_node_id, ua.NodeId(ua.ObjectIds.Structure, 0),
                                          ua.NodeId(ua.ObjectIds.HasSubtype, 0), False),
                # add reverse reference to created data type
                self._reference_generator(linked_obj_node_id, data_type_node_id,
                                          ua.NodeId(ua.ObjectIds.HasEncoding, 0), False),
                # add HasDescription link to dictionary description
                self._reference_generator(linked_obj_node_id, description_node_id,
                                          ua.NodeId(ua.ObjectIds.HasDescription, 0)),
                # add reverse HasDescription link
                self._reference_generator(description_node_id, linked_obj_node_id,
                                          ua.NodeId(ua.ObjectIds.HasDescription, 0), False),
                # add link to the type definition node
                self._reference_generator(linked_obj_node_id, ua.NodeId(ua.ObjectIds.DataTypeEncodingType, 0),
                                          ua.NodeId(ua.ObjectIds.HasTypeDefinition, 0)),
                # add has type definition link
                self._reference_generator(description_node_id, ua.NodeId(ua.ObjectIds.DataTypeDescriptionType, 0),
                                          ua.NodeId(ua.ObjectIds.HasTypeDefinition, 0)),
                # forward link of dict to description item
                self._reference_generator(self.dict_id, description_node_id,
                                          ua.NodeId(ua.ObjectIds.HasComponent, 0)),
                # add reverse link to dictionary
                self._reference_generator(description_node_id, self.dict_id,
                                          ua.NodeId(ua.ObjectIds.HasComponent, 0), False)]
        self.server.add_references(refs)

    def create_data_type(self, name):
        # apply for new node id
        data_type_node_id = nodeid_generator(self.idx)
        description_node_id = nodeid_generator(self.idx)
        bind_obj_node_id = nodeid_generator(self.idx)

        # create data type node
        dt_node = ua.AddNodesItem()
        dt_node.RequestedNewNodeId = data_type_node_id
        dt_node.BrowseName = ua.QualifiedName(name, self.idx)
        dt_node.NodeClass = ua.NodeClass.DataType
        dt_node.ParentNodeId = ua.NodeId(ua.ObjectIds.Structure, 0)
        dt_node.ReferenceTypeId = ua.NodeId(ua.ObjectIds.HasSubtype, 0)
        dt_attributes = ua.DataTypeAttributes()
        dt_attributes.DisplayName = ua.LocalizedText(name)
        dt_node.NodeAttributes = dt_attributes

        # create description node
        desc_node = ua.AddNodesItem()
        desc_node.RequestedNewNodeId = description_node_id
        desc_node.BrowseName = ua.QualifiedName(name, self.idx)
        desc_node.NodeClass = ua.NodeClass.Variable
        desc_node.ParentNodeId = self.dict_id
        desc_node.ReferenceTypeId = ua.NodeId(ua.ObjectIds.HasComponent, 0)
        desc_node.TypeDefinition = ua.NodeId(ua.ObjectIds.DataTypeDescriptionType, 0)
        desc_attributes = ua.VariableAttributes()
        desc_attributes.DisplayName = ua.LocalizedText(name)
        desc_attributes.DataType = ua.NodeId(ua.ObjectIds.String)
        desc_attributes.Value = ua.Variant(name, ua.VariantType.String)
        desc_attributes.ValueRank = -1
        desc_node.NodeAttributes = desc_attributes

        # create object node python class should link to
        obj_node = ua.AddNodesItem()
        obj_node.RequestedNewNodeId = bind_obj_node_id
        obj_node.BrowseName = ua.QualifiedName('Default Binary', 0)
        obj_node.NodeClass = ua.NodeClass.Object
        obj_node.ParentNodeId = data_type_node_id
        obj_node.ReferenceTypeId = ua.NodeId(ua.ObjectIds.HasEncoding, 0)
        obj_node.TypeDefinition = ua.NodeId(ua.ObjectIds.DataTypeEncodingType, 0)
        obj_attributes = ua.ObjectAttributes()
        obj_attributes.DisplayName = ua.LocalizedText("Default Binary")
        obj_attributes.EventNotifier = 0
        obj_node.NodeAttributes = obj_attributes

        self.server.add_nodes([dt_node, desc_node, obj_node])
        # link the three node by there node id according to UA standard
        self._link_nodes(bind_obj_node_id, data_type_node_id, description_node_id)

        return data_type_node_id


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
