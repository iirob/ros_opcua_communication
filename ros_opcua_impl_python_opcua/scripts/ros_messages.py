import roslib
import roslib.message
import rospy

from subprocess import Popen, PIPE
from datetime import datetime

from opcua import ua
from opcua.common.instantiate import instantiate

import ros_global


class OpcUaROSMessage:
    def __init__(self, server, parent, idx, message):
        self._server = server
        self._parent = parent
        self._idx = idx
        self._message = message
        # This _node has only value assignment, no other usage, make sure and delete it
        self._nodes = {}
        hierarchical_path = ['0:Types', '0:DataTypes', '0:BaseDataType', '0:Structure']
        self._base_data_type = self._server.get_root_node().get_child(hierarchical_path)

        self._set_message_class(message)
        self._recursive_create_items(self._parent, self._idx, message.split('/')[0], message.split('/')[-1],
                                     self._message_instance)

    def _set_message_class(self, message):
        try:
            message_class = roslib.message.get_message_class(message)
            if not message_class:
                raise rospy.ROSException
            self._message_instance = message_class()
        except rospy.ROSException:
            rospy.logfatal("Couldn't find message class for type " + message)
            return

    def _recursive_create_items(self, parent, idx, package, message_type, message):
        # message can be an object, a complex data types or an array
        # Here is a 'complex data type'
        # TODO: For complex type, the correct hierarchy should be built up, Not always BasicVariableDataType!!!
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            # get node if it exists or create a new node

            node_identifier = package + '/' + message_type
            # if node_identifier in ros_global.messageNode.keys():
            #     node = ros_global.messageNode[node_identifier]
            if node_identifier not in ros_global.messageNode.keys():
                # add new object type node
                data_type_id = ua.NodeId(package + '/' + message_type + 'DataType',
                                         parent.nodeid.NamespaceIndex, ua.NodeIdType.String)
                data_type_b_name = ua.QualifiedName(message_type + 'DataType', parent.nodeid.NamespaceIndex)
                data_type_node = self._base_data_type.add_data_type(data_type_id, data_type_b_name, description=None)
                # not add VariableType? Then why check VariableType in ros_server.py ?
                new_node = parent.add_variable(ua.NodeId(package + '/' + message_type + 'Type',
                                                         parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                               ua.QualifiedName(message_type+'Type', parent.nodeid.NamespaceIndex),
                                               data_type_node.nodeid)

                ros_global.messageNode[node_identifier] = new_node
                ros_global.dataTypeNode[node_identifier] = data_type_node

                # get over all message attribute and add a variable node to the new object type-node
                for slot_name, type_name_child in zip(message.__slots__, getattr(message, '_slot_types')):
                    base_type_str, array_size = _extract_array_info(type_name_child)

                    # if the slot is a simple type, add a variable node, why not directly use if????
                    attr_node = _create_node_with_type(new_node, package, message_type + '/' + slot_name,
                                                       type_name_child, array_size)

                    if attr_node is None:
                        # slot is a complex type, create new message node
                        # (new instance of the class RosMessage(if it doesn't exit)
                        # and call the function recursive_create_item in constructor)
                        attr_node = _process_node_variable_type(base_type_str, self._server, idx)
                        var_node = new_node.add_variable(ua.NodeId(package + '/' + message_type + '/' + slot_name, idx),
                                                         ua.QualifiedName(slot_name, idx),
                                                         ua.Variant(None, ua.VariantType.Null),
                                                         attr_node.nodeid,
                                                         ros_global.dataTypeNode[base_type_str].nodeid)

                        # if is a array
                        if array_size is not None:
                            # print(package + '/' + message_type)
                            var_node.set_value_rank(1)  # is a array
                            if array_size > 0:  # or zero  [] or [n]
                                var_node.set_array_dimensions([array_size])

        else:
            # arrays
            base_type_str, array_size = _extract_array_info(message)

            try:
                base_instance = roslib.message.get_message_class(base_type_str)()
            except (ValueError, TypeError):
                base_instance = None

            if array_size is not None and hasattr(base_instance, '__slots__'):  # for array of complex objects
                # What does this loop do?
                for index in range(array_size):
                    print('')
            else:   # for base type and array of base types
                new_node = _create_node_with_type(parent, package, message_type, message, array_size)
                # if new_node is None:
                #     create new node (check it exists)

                self._nodes[message_type] = new_node


def _extract_array_info(type_str):
    # TODO: correct set the array_size of a scala, i.e. -1 according to the documentation.
    array_size = None
    if '[' in type_str and type_str[-1] == ']':
        type_str, array_size_str = type_str.split('[', 1)
        array_size_str = array_size_str[:-1]
        if len(array_size_str) > 0:
            array_size = int(array_size_str)
        else:
            array_size = 0

    return type_str, array_size


def get_ros_msg():
    """
    Same as execute 'rosmsg list' in terminal
    :return: a list with msgs
    """
    out, _ = Popen(['rosmsg', 'list'], stdin=PIPE, stdout=PIPE, stderr=PIPE).communicate()
    return out.split()


def _create_node_with_type(parent, package, message_name, type_name, array_size, prop=True):
    """
    This function must be refactored, since it contains two functions, one is to check if it
    is a simple type, another is that if it is a simple type, then create a node, confusing
    on first sight. Violated least astonishment principle and single responsibility principle.
    :param parent:
    :param package:
    :param message_name:
    :param type_name:
    :param array_size:
    :param prop: Property
    :return:
    """
    if '[' in type_name:
        type_name = type_name[:type_name.index('[')]

    if type_name == 'bool':
        dv = ua.Variant(False, ua.VariantType.Boolean)
    elif type_name == 'byte':
        dv = ua.Variant(0, ua.VariantType.Byte)
    elif type_name == 'int':
        dv = ua.Variant(0, ua.VariantType.Int32)
    elif type_name == 'int8':
        dv = ua.Variant(0, ua.VariantType.SByte)
    elif type_name == 'uint8' or type_name == 'char':
        dv = ua.Variant(0, ua.VariantType.Byte)
    elif type_name == 'int16':
        dv = ua.Variant(0, ua.VariantType.Int16)
    elif type_name == 'uint16':
        dv = ua.Variant(0, ua.VariantType.UInt16)
    elif type_name == 'int32':
        dv = ua.Variant(0, ua.VariantType.Int32)
    elif type_name == 'uint32':
        dv = ua.Variant(0, ua.VariantType.UInt32)
    elif type_name == 'int64':
        dv = ua.Variant(0, ua.VariantType.Int64)
    elif type_name == 'uint64':
        dv = ua.Variant(0, ua.VariantType.UInt64)
    elif type_name == 'float' or type_name == 'float32' or type_name == 'float64':
        dv = ua.Variant(0.0, ua.VariantType.Float)
    elif type_name == 'double':
        dv = ua.Variant(0.0, ua.VariantType.Double)
    elif type_name == 'string':
        dv = ua.Variant('', ua.VariantType.String)
    elif type_name == 'time' or type_name == 'duration':
        dv = ua.Variant(datetime.utcnow(), ua.VariantType.DateTime)
    else:
        # check if node exists, create node and add reference to the new node
        return None

    # create new node
    if prop:  # as property
        node = parent.add_property(ua.NodeId(package + '/' + message_name, parent.nodeid.NamespaceIndex),
                                   ua.QualifiedName(message_name.split('/')[-1], parent.nodeid.NamespaceIndex),
                                   dv.Value)
    else:  # Base variable
        node = parent.add_variable(ua.NodeId(package + '/' + message_name, parent.nodeid.NamespaceIndex),
                                   ua.QualifiedName(message_name.split('/')[-1], parent.nodeid.NamespaceIndex),
                                   dv.Value)

    # if is a array
    if array_size is not None:
        node.set_value_rank(1)  # is a array
        if array_size > 1:   # or zero  [] or [n]
            node.set_array_dimensions([array_size])

    return node


def _process_node_variable_type(message, server, idx):
    """
    This function check if a node related to a message has been added and return the node if it exists,
     if node doesn't exit create a new object RosMessage and call the function _recursive_create_item 
     in constructor to create the related node object type and add all attribute of the message 
    :param message: 
    :param server: 
    :param idx: 
    :return: 
    """
    hierarchical_path = ['0:Types', '0:VariableTypes', '0:BaseVariableType', '0:BaseDataVariableType',
                         str(idx) + ':ROSMessageVariableTypes']
    message_variable_type = server.get_root_node().get_child(hierarchical_path)

    # if node is a complex type, check if node message node exits -> add reference to the current message node
    # if not create message node and add reference
    if message in ros_global.messageNode.keys():
        node = ros_global.messageNode[message]
    else:
        package = message.split('/')[0]
        if package not in ros_global.package_node_created.keys():
            package_node = message_variable_type.add_folder(idx, package)
            ros_global.packages.append(package)
            ros_global.package_node_created[package] = package_node
        else:
            package_node = ros_global.package_node_created.get(package)
        OpcUaROSMessage(server, package_node, idx, message)
        node = ros_global.messageNode[message]

    return node


def update_node_with_message(node, message, idx):
    """
    the method update all variable of a node or copy all values from message to node
    IMPORT: the variable browser name of the variable node must be the same as the attribute of the message object
    """
    value = message
    # set value if exists
    if value is not None:
        if not (hasattr(value, '__slots__') or type(value) in (list, tuple)):  # PRIMITIVE TYPE
            node.set_value(value)
        elif type(value) in (list, tuple):  # handle array
            if len(value) > 0:
                node.set_value(value)
        else:  # complex type
            if type(message).__name__ in ('Time', 'Duration'):
                value = message.secs
                node.set_value(value)
                # node.set_value(str(value))
            else:
                node.set_value(str(value))

    node_children = node.get_children()
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


def _init_node_recursively(node, idx):
    """ 
    This function initiate all the sub variable with complex type of customized type of the node
    TODO: the instantiate function itself is a recursive realization, try to use the original one.
    :param node: opc ua node
    :param idx:
    """
    children = node.get_children()
    if not (len(children) > 0):
        return
    for child in children:
        if _is_variable_and_string_type(child):
            variable_type_name = child.get_type_definition().Identifier.replace('Type', '')
            if variable_type_name in ros_global.messageNode.keys():
                variable_type = ros_global.messageNode[variable_type_name]
                created_node = instantiate(node, variable_type, bname=child.get_browse_name(), idx=idx)[0]
                _init_node_recursively(created_node, idx)
                child.delete()


def _is_variable_and_string_type(node):
    return node.get_node_class() == ua.NodeClass.Variable and \
           node.get_type_definition().NodeIdType == ua.NodeIdType.String  # complex type or customized type


def update_message_instance_with_node(message, node):
    """ the function try to update all message attribute with the help of node info.
    NB: all node variable browse name must be the same name as the the message attribute """
    variables = node.get_children(nodeclassmask=ua.NodeClass.Variable)

    if len(variables) > 0:
        for var in variables:
            attr_name = var.get_browse_name().Name
            if hasattr(message, attr_name):
                if len(var.get_children(nodeclassmask=ua.NodeClass.Variable)) == 0:  # primitive type
                    setattr(message, attr_name, ros_global.correct_type(var, type(getattr(message, attr_name))))
                    # var.get_value())
                    # setattr(message, attr_name, var.get_value())
                else:     # complex type
                    update_message_instance_with_node(getattr(message, attr_name), var)

    return message
    # if has variables recursion
    # else update value
