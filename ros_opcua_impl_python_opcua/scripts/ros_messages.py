import roslib
import roslib.message
import rospy

from opcua.common.instantiate import instantiate

import ros_global
from ros_opc_ua import *


def _get_message_class(message):
    try:
        message_class = roslib.message.get_message_class(message)
        if not message_class:
            raise rospy.ROSException
        return message_class()
    except rospy.ROSException:
        rospy.logfatal('Could not find message class for type ' + message)
        return None


def _extract_array_info(type_str):
    array_size = None
    if '[' in type_str and type_str[-1] == ']':
        type_str, array_size_str = type_str.split('[', 1)
        array_size_str = array_size_str[:-1]
        if len(array_size_str) > 0:
            array_size = int(array_size_str)
        else:
            array_size = 0

    return type_str, array_size


class OpcUaROSMessage:
    def __init__(self, server, idx):
        self.server = server
        self._idx = idx

        self.created_data_types = {}
        self.created_variable_types = {}

        data_variable_root_path = ['0:Types', '0:VariableTypes', '0:BaseVariableType', '0:BaseDataVariableType']
        vt_base_node = self.server.get_root_node().get_child(data_variable_root_path)
        self._vt_root = create_variable_type(vt_base_node, nodeid_generator(self._idx),
                                             ua.QualifiedName('ROSMessageType', self._idx),
                                             vt_base_node.get_data_type())

        data_type_root_path = ['0:Types', '0:DataTypes', '0:BaseDataType', '0:Structure']
        dt_base_node = self.server.get_root_node().get_child(data_type_root_path)
        self._dt_root = create_data_type(dt_base_node, nodeid_generator(self._idx),
                                         ua.QualifiedName('ROSMessage', self._idx), is_abstract=True)

    def _is_new_type(self, message):
        return message not in ROS_BUILD_IN_DATA_TYPES.keys() and message not in self.created_data_types.keys()

    def _add_data_and_variable_type(self, type_name):
        new_dt = create_data_type(self._dt_root, nodeid_generator(self._idx), ua.QualifiedName(type_name, self._idx))
        self.created_data_types[type_name] = new_dt

        # According to the convention, variable type should end with Type
        vt_name = type_name + 'Type'
        new_vt = create_variable_type(self._vt_root, nodeid_generator(self._idx),
                                      ua.QualifiedName(vt_name, self._idx), new_dt.nodeid)
        self.created_variable_types[vt_name] = new_vt

    def _recursively_create_message(self, msg):
        # Add current message to data and variable type list
        msg_type_str, _ = _extract_array_info(msg)
        if msg_type_str not in self.created_data_types.keys():
            self._add_data_and_variable_type(msg_type_str)
        # handle subtypes
        message = _get_message_class(msg)
        for variable_type, data_type in zip(message.__slots__, getattr(message, '_slot_types')):
            base_type_str, array_size = _extract_array_info(data_type)
            if self._is_new_type(base_type_str):
                self._add_data_and_variable_type(base_type_str)
                self._recursively_create_message(base_type_str)

            current_variable_type = self.created_variable_types[msg + 'Type']
            if base_type_str in ROS_BUILD_IN_DATA_TYPES.keys():
                create_property(current_variable_type, nodeid_generator(self._idx),
                                ua.QualifiedName(variable_type, self._idx), array_size,
                                *ROS_BUILD_IN_DATA_TYPES[base_type_str])
            else:
                create_variable(current_variable_type, nodeid_generator(self._idx),
                                ua.QualifiedName(variable_type, self._idx),
                                self.created_variable_types[base_type_str + 'Type'].nodeid,
                                self.created_data_types[base_type_str].nodeid, array_size)

    def create_messages(self):
        messages = ros_global.get_ros_msg()
        for msg in messages:
            if msg not in self.created_data_types.keys():
                self._recursively_create_message(msg)


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
            if variable_type_name in ros_global.messageNode.keys():
                variable_type = ros_global.messageNode[variable_type_name]
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
                    setattr(message, attr_name, ros_global.correct_type(var, type(getattr(message, attr_name))))
                    # var.get_value())
                    # setattr(message, attr_name, var.get_value())
                else:     # complex type
                    update_message_instance_with_node(getattr(message, attr_name), var)

    return message
    # if has variables recursion
    # else update value
