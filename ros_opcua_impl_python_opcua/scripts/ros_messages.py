import roslib.message
from opcua import ua

from ros_global import get_ros_messages, get_ros_services
from ros_opc_ua import DataTypeDictionaryBuilder, ROS_BUILD_IN_DATA_TYPES


def extract_array_info(type_str):
    """ROS only support 1 dimensional array"""
    is_array = False
    if '[' in type_str and type_str[-1] == ']':
        type_str = type_str.split('[', 1)[0]
        is_array = True

    return type_str, is_array


class OpcUaROSMessage:
    def __init__(self, server, idx, idx_name):
        self._server = server
        self._idx = idx

        self._created_data_types = {}

        self._dict_builder = DataTypeDictionaryBuilder(server, idx, idx_name, 'ROSDictionary')

    def _is_new_type(self, message):
        return message not in ROS_BUILD_IN_DATA_TYPES and message not in self._created_data_types

    def _create_data_type(self, type_name):
        new_dt_id = self._dict_builder.create_data_type(type_name)
        self._created_data_types[type_name] = new_dt_id

    def _recursively_create_message(self, msg):
        if self._is_new_type(msg):
            self._create_data_type(msg)
        message = roslib.message.get_message_class(msg)
        if not message:  # Broken packages
            return
        for variable_type, data_type in zip(message.__slots__, getattr(message, '_slot_types')):
            base_type_str, is_array = extract_array_info(data_type)
            if self._is_new_type(base_type_str):
                self._create_data_type(base_type_str)
                self._recursively_create_message(base_type_str)

            self._dict_builder.add_field(base_type_str, variable_type, msg, is_array)

    def _create_messages(self):
        messages = get_ros_messages()
        for msg in messages:
            if msg not in self._created_data_types:
                self._recursively_create_message(msg)

    def _process_service_classes(self, srv):
        msg_name = getattr(srv, '_type')
        self._create_data_type(msg_name)
        for variable_type, data_type in zip(srv.__slots__, getattr(srv, '_slot_types')):
            base_type_str, is_array = extract_array_info(data_type)
            self._dict_builder.add_field(base_type_str, variable_type, msg_name, is_array)

    def _create_services(self):
        """since srv can not embed another .srv, no recursion is needed"""
        services = get_ros_services()
        for srv in services:
            service = roslib.message.get_service_class(srv)
            if not service:  # Broken packages
                continue
            self._process_service_classes(getattr(service, '_request_class'))
            self._process_service_classes(getattr(service, '_response_class'))

    def create_ros_data_types(self):
        self._create_messages()
        self._create_services()

        self._dict_builder.set_dict_byte_string()

        return self._created_data_types


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
