from opcua.common.instantiate import instantiate

from ros_global import *
from ros_opc_ua import *


class OpcUaROSMessage:
    def __init__(self, server, idx, idx_name):
        self._server = server
        self._idx = idx

        self.created_data_types = {}
        self.created_variable_types = {}
        self._dict_structs = {}

        vt_base_node = self._server.nodes.base_variable_type.get_child('0:BaseDataVariableType')
        self._vt_root = create_variable_type(vt_base_node, nodeid_generator(self._idx),
                                             ua.QualifiedName('ROSMessageType', self._idx),
                                             vt_base_node.get_data_type())

        dt_base_node = self._server.nodes.base_structure_type
        self._dt_root = create_data_type(dt_base_node, nodeid_generator(self._idx),
                                         ua.QualifiedName('ROSMessage', self._idx), is_abstract=True)
        self._dict_base = self._server.nodes.opc_binary.add_variable(nodeid_generator(self._idx), 'ROSDictionary',
                                                                     ua.get_default_value(ua.VariantType.ByteString),
                                                                     varianttype=ua.VariantType.ByteString)
        self._dict_base.delete_reference(ua.ObjectIds.BaseDataVariableType, ua.ObjectIds.HasTypeDefinition)
        self._dict_base.add_reference(ua.ObjectIds.DataTypeDictionaryType, ua.ObjectIds.HasTypeDefinition)
        self._dict_base.add_property(nodeid_generator(self._idx), 'NameSpaceUri', idx_name)
        self._dict_base.add_property(nodeid_generator(self._idx), 'Deprecated', True)

        self._type_dictionary = TypeBinaryDictionary(idx_name)

    def _is_new_type(self, message):
        return message not in ROS_BUILD_IN_DATA_TYPES.keys() and message not in self.created_data_types.keys()

    def _create_data_and_variable_type(self, type_name):
        new_dt = self._dt_root.add_data_type(nodeid_generator(self._idx), type_name)
        # new_dt = create_data_type(self._dt_root, nodeid_generator(self._idx),
        #                           ua.QualifiedName(type_name, self._idx))
        self.created_data_types[type_name] = new_dt
        description_node = self._dict_base.add_variable(nodeid_generator(self._idx), type_name,
                                                        str(type_name), varianttype=ua.VariantType.String)
        description_node.delete_reference(ua.ObjectIds.BaseDataVariableType, ua.ObjectIds.HasTypeDefinition)
        description_node.add_reference(ua.ObjectIds.DataTypeDescriptionType, ua.ObjectIds.HasTypeDefinition)
        encoding_node = new_dt.add_object(nodeid_generator(self._idx), 'Default Binary',
                                          ua.ObjectIds.DataTypeEncodingType)
        encoding_node.add_reference(description_node, ua.ObjectIds.HasDescription)

        # According to convention, variable type should end with Type
        # vt_name = type_name + 'Type'
        # new_vt = create_variable_type(self._vt_root, nodeid_generator(self._idx),
        #                               ua.QualifiedName(vt_name, self._idx), new_dt.nodeid)
        # self.created_variable_types[vt_name] = new_vt

        new_struct = self._type_dictionary.append_struct(type_name)
        self._dict_structs[type_name] = new_struct

    def _recursively_create_message(self, msg):
        # Add current message to data and variable type list
        if self._is_new_type(msg):
            self._create_data_and_variable_type(msg)
        # handle subtypes
        message = get_message_class(msg)
        if not message:
            return
        for variable_type, data_type in zip(message.__slots__, getattr(message, '_slot_types')):
            base_type_str, array_size = extract_array_info(data_type)
            if self._is_new_type(base_type_str):
                self._create_data_and_variable_type(base_type_str)
                self._recursively_create_message(base_type_str)

            self._type_dictionary.add_field(base_type_str, variable_type, self._dict_structs[msg])
            # current_variable_type = self.created_variable_types[msg + 'Type']
            # if base_type_str in ROS_BUILD_IN_DATA_TYPES.keys():
            #     create_property(current_variable_type, nodeid_generator(self._idx),
            #                     ua.QualifiedName(variable_type, self._idx), array_size,
            #                     ua.get_default_value(ROS_BUILD_IN_DATA_TYPES[base_type_str]),
            #                     ROS_BUILD_IN_DATA_TYPES[base_type_str])
            # else:
            #     create_variable(current_variable_type, nodeid_generator(self._idx),
            #                     ua.QualifiedName(variable_type, self._idx),
            #                     self.created_variable_types[base_type_str + 'Type'].nodeid,
            #                     self.created_data_types[base_type_str].nodeid, array_size)

    def create_messages(self):
        messages = get_ros_messages()
        for msg in messages:
            if msg not in self.created_data_types.keys():
                self._recursively_create_message(msg)

        self._dict_base.set_value(self._type_dictionary.get_dict_value(), ua.VariantType.ByteString)


def update_node_with_message(node_name, message, idx):
    """
    the method update all variable of a node or copy all values from message to node
    IMPORT: the variable browser name of the variable node must be the same as the attribute of the message object
    """
    value = message
    # set value if exists
    if value is not None:
        if not (hasattr(value, '__slots__') or type(value) in (list, tuple)):  # PRIMITIVE TYPE
            node_name.set_value(value)
        elif type(value) in (list, tuple):  # handle array
            if len(value) > 0:
                node_name.set_value(value)
        else:  # complex type
            if type(message).__name__ in ('Time', 'Duration'):
                value = message.secs
                node_name.set_value(value)
                # node.set_value(str(value))
            else:
                node_name.set_value(str(value))

    node_children = node_name.get_children()
    for child in node_children:
        # if child a variable
        if child.get_node_class() == ua.NodeClass.Variable:
            # get attr_name
            if hasattr(value, child.get_browse_name().Name):
                update_node_with_message(child,  getattr(value, child.get_browse_name().Name), idx)


def instantiate_customized(parent, node_type, node_id=None, bname=None, idx=0):
    """
    Please take care that in the new version of python opcua, the dname, ie. the DisplayName is deleted from the
     parameter list in instantiate function
    :param parent: 
    :param node_type: 
    :param node_id: 
    :param bname: BrowseName
    :param idx: 
    :return: 
    """
    nodes = instantiate(parent, node_type, nodeid=node_id, bname=bname, idx=idx)
    new_node = nodes[0]
    _init_node_recursively(new_node, idx)
    return new_node


def _init_node_recursively(node_name, idx):
    """ 
    This function initiate all the sub variable with complex type of customized type of the node
    TODO: the instantiate function itself is a recursive realization, try to use the original one.
    :param node_name: opc ua node
    :param idx:
    """
    children = node_name.get_children()
    if not (len(children) > 0):
        return
    for child in children:
        if _is_variable_and_string_type(child):
            variable_type_name = child.get_type_definition().Identifier.replace('Type', '')
            if variable_type_name in messageNode.keys():
                variable_type = messageNode[variable_type_name]
                created_node = instantiate(node_name, variable_type, bname=child.get_browse_name(), idx=idx)[0]
                _init_node_recursively(created_node, idx)
                child.delete()


def _is_variable_and_string_type(node_name):
    return node_name.get_node_class() == ua.NodeClass.Variable and \
           node_name.get_type_definition().NodeIdType == ua.NodeIdType.String  # complex type or customized type


def update_message_instance_with_node(message, node_name):
    """ the function try to update all message attribute with the help of node info.
    NB: all node variable browse name must be the same name as the the message attribute """
    variables = node_name.get_children(nodeclassmask=ua.NodeClass.Variable)

    if len(variables) > 0:
        for var in variables:
            attr_name = var.get_browse_name().Name
            if hasattr(message, attr_name):
                if len(var.get_children(nodeclassmask=ua.NodeClass.Variable)) == 0:  # primitive type
                    setattr(message, attr_name, correct_type(var, type(getattr(message, attr_name))))
                    # var.get_value())
                    # setattr(message, attr_name, var.get_value())
                else:     # complex type
                    update_message_instance_with_node(getattr(message, attr_name), var)

    return message
    # if has variables recursion
    # else update value
