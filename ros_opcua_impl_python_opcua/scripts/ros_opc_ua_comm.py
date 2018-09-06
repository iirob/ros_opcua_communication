import rospy
import roslib.message
import rostopic
import rosservice
import rosmsg
import rospkg

from opcua import ua, uamethod
from opcua.common.type_dictionary_buider import DataTypeDictionaryBuilder, get_ua_class

ros_build_in_types = {'bool': ua.VariantType.Boolean,
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
                      'string': ua.VariantType.String}

ua_basic_types = [item.name for item in ros_build_in_types.values()]


def expand_ua_class(obj, level=0):
    """expand ua class to plain text"""
    if not obj:
        return None
    buff = '\n{}ua_object, type: {}'.format(level * '\t', obj.__class__.__name__)
    for var, data_type in obj.ua_types:
        if data_type in ua_basic_types:
            buff += '\n\t{}:{}'.format(level * '\t' + var, getattr(obj, var))
        else:
            if isinstance(getattr(obj, var), list):
                for member in getattr(obj, var):
                    if hasattr(member, 'ua_types'):
                        buff += expand_ua_class(member, level=level + 1)
                    else:
                        buff += '\n\t{}:{}'.format(level * '\t' + type(member).__name__, member)
            else:
                buff += expand_ua_class(getattr(obj, var), level=level + 1)
    return buff


def _lookup_type(type_name):
    if type_name in ros_build_in_types:
        return ros_build_in_types[type_name]
    return type_name


def _get_ros_packages(mode):
    """
    same as the command line 'rosmsg packages'
    :return: ROS messages as a list
    """
    return sorted([x for x in rosmsg.iterate_packages(rospkg.RosPack(), mode)])


def _get_ros_msg(mode):
    ret = []
    if mode == rosmsg.MODE_MSG:
        suffix = 'msg'
    else:
        suffix = 'srv'
    ros_packages = _get_ros_packages(mode)
    for (p, directory) in ros_packages:
        for file_name in getattr(rosmsg, '_list_types')(directory, suffix, mode):
            ret.append(p + '/' + file_name)
    return ret


def _extract_ua_array_info(type_str):
    is_array = False
    if type_str.startswith('ListOf'):
        type_str = type_str.split('ListOf', 1)[-1]
        is_array = True

    return type_str, is_array


def _extract_ros_array_info(type_str):
    """ROS only support 1 dimensional array"""
    is_array = False
    if '[' in type_str:
        type_str = type_str.split('[', 1)[0]
        is_array = True

    return type_str, is_array


def to_ros_msg(ua_obj, ros_msg, msg_dict):
    for attr in ua_obj.ua_types:
        base_type_str, is_array = _extract_ua_array_info(attr[1])
        if base_type_str in ua_basic_types:
            setattr(ros_msg, attr[0], getattr(ua_obj, attr[0]))
        else:
            if is_array:
                for member in getattr(ua_obj, attr[0]):
                    new_ros_msg = roslib.message.get_message_class(msg_dict[base_type_str])()
                    getattr(ros_msg, attr[0]).append(new_ros_msg)
                    to_ros_msg(member, new_ros_msg, msg_dict)
            else:
                to_ros_msg(getattr(ua_obj, attr[0]), getattr(ros_msg, attr[0]), msg_dict)
    return ros_msg


def to_ua_class(ros_msg, ua_obj):
    for attr, data_type in zip(ros_msg.__slots__, getattr(ros_msg, '_slot_types')):
        base_type_str, is_array = _extract_ros_array_info(data_type)
        if base_type_str in ros_build_in_types:
            setattr(ua_obj, attr, getattr(ros_msg, attr))
        else:
            if is_array:
                for member in getattr(ros_msg, attr):
                    new_ua_class = get_ua_class(getattr(member, '_type'))()
                    getattr(ua_obj, attr).append(new_ua_class)
                    to_ua_class(member, new_ua_class)
            else:
                to_ua_class(getattr(ros_msg, attr), getattr(ua_obj, attr))
    return ua_obj


def _create_args(msg_class, data_type):
    """one extension object contains all info"""
    if not len(getattr(msg_class, '__slots__')):
        return []
    msg_class_name = getattr(msg_class, '_type')
    arg = ua.Argument()
    arg.Name = msg_class_name
    arg.DataType = data_type
    arg.ValueRank = -1
    arg.ArrayDimensions = []
    arg.Description = ua.LocalizedText(msg_class_name)
    return [arg]


class OpcUaROSMessage:
    def __init__(self, server, idx, idx_name):
        self._server = server
        self._idx = idx

        self._created_struct_nodes = {}
        self._dict_builder = DataTypeDictionaryBuilder(server, idx, idx_name, 'ROSDictionary')

    def _is_new_type(self, message):
        if not isinstance(message, str):
            message = message.name
        return message not in ros_build_in_types and message not in self._created_struct_nodes

    def _create_data_type(self, type_name):
        new_dt = self._dict_builder.create_data_type(type_name)
        self._created_struct_nodes[type_name] = new_dt
        return new_dt

    def _recursively_create_message(self, msg):
        msg = self._create_data_type(msg) if self._is_new_type(msg) else self._created_struct_nodes[msg]
        message = roslib.message.get_message_class(msg.name)
        if not message:  # Broken packages
            return
        for variable_type, data_type in zip(message.__slots__, getattr(message, '_slot_types')):
            base_type_str, is_array = _extract_ros_array_info(data_type)
            if self._is_new_type(base_type_str):
                self._create_data_type(base_type_str)
                self._recursively_create_message(base_type_str)

            msg.add_field(variable_type, _lookup_type(base_type_str), is_array)

    def _create_messages(self):
        messages = _get_ros_msg(rosmsg.MODE_MSG)
        for msg in messages:
            if msg not in self._created_struct_nodes:
                self._recursively_create_message(msg)

    def _process_service_classes(self, srv):
        msg_name = getattr(srv, '_type')
        new_struct = self._create_data_type(msg_name)
        for variable_type, data_type in zip(srv.__slots__, getattr(srv, '_slot_types')):
            base_type_str, is_array = _extract_ros_array_info(data_type)
            new_struct.add_field(variable_type, _lookup_type(base_type_str), is_array)

    def _create_services(self):
        """since srv can not embed another .srv, no recursion is needed"""
        services = _get_ros_msg(rosmsg.MODE_SRV)
        for srv in services:
            service = roslib.message.get_service_class(srv)
            if not service:  # Broken packages
                continue
            self._process_service_classes(getattr(service, '_request_class'))
            self._process_service_classes(getattr(service, '_response_class'))

    def create_ros_data_types(self):
        self._create_messages()
        created_msgs = {key: value.data_type for key, value in self._created_struct_nodes.items()}
        self._create_services()
        self._dict_builder.set_dict_byte_string()
        data_types = {key: value.data_type for key, value in self._created_struct_nodes.items()}
        created_srvs = {key: value for key, value in data_types.items() if key not in created_msgs}
        return data_types, created_msgs, created_srvs


class OpcUaROSService:

    def __init__(self, service_name, node_root, service_node_id, msg_dict, reverse_dict):
        self._service_class = rosservice.get_service_class_by_name(service_name)
        self._service_name = service_name
        self._proxy = rospy.ServiceProxy(service_name, self._service_class)
        self._node_root = node_root

        self._ros_service_req = getattr(self._service_class, '_request_class')
        self._req_name = getattr(self._ros_service_req, '_type')
        self._ros_service_resp = getattr(self._service_class, '_response_class')
        self._resp_name = getattr(self._ros_service_resp, '_type')

        in_dt_node_id = msg_dict[self._req_name]
        out_dt_node_id = msg_dict[self._resp_name]
        input_arg = _create_args(self._ros_service_req, in_dt_node_id)
        output_arg = _create_args(self._ros_service_resp, out_dt_node_id)

        self._method = node_root.add_method(service_node_id, service_name, self._call_service, input_arg, output_arg)
        self._msg_dict = reverse_dict
        rospy.loginfo('Created ROS Service: ' + service_name)

    @uamethod
    def _call_service(self, _, *inputs):
        rospy.loginfo('Calling service {0} under ROS node: {1}'.format(self._service_name,
                                                                       self._node_root.get_display_name().Text))
        try:
            ua_obj = inputs[0] if inputs else get_ua_class(self._req_name)()
            response = self._proxy(to_ros_msg(ua_obj, self._ros_service_req(), self._msg_dict))
            rospy.loginfo('Calling service succeeded!')
            if response.__slots__:
                resp_ua_class = to_ua_class(response, get_ua_class(self._resp_name)())
                return ua.Variant(resp_ua_class)
        except Exception as e:
            rospy.logerr('Error when calling service ' + self._service_name, e)

    def get_node(self):
        return self._method

    def delete_node(self):
        self._proxy.close()
        self._method.delete()
        rospy.loginfo('Deleted ROS Service: ' + self._service_name)


class OpcUaROSTopic:

    def __init__(self, topic_name, status_root, pub_root, msg_dict, reverse_dict, *node_id):
        self._topic_name = topic_name
        self._topic_class, _, _ = rostopic.get_topic_class(self._topic_name)
        self._msg_name = getattr(self._topic_class, '_type')
        self._topic = status_root.add_property(node_id[0], self._topic_name, ua.Variant(None, ua.VariantType.Null),
                                               datatype=msg_dict[self._msg_name])
        self._pub_handler = rospy.Publisher(self._topic_name, self._topic_class, queue_size=10)
        self._sub_handler = rospy.Subscriber(self._topic_name, self._topic_class, self._message_callback)

        input_arg = _create_args(self._topic_class, msg_dict[self._msg_name])
        self._method = pub_root.add_method(node_id[1], self._topic_name, self._call_publish, input_arg, [])
        self._msg_dict = reverse_dict
        rospy.loginfo('Created ROS Topic: ' + self._topic_name)

    def _message_callback(self, message):
        self._topic.set_value(to_ua_class(message, get_ua_class(self._msg_name)()))

    @uamethod
    def _call_publish(self, _, *inputs):
        rospy.loginfo('Publishing data under ROS Topic: ' + self._topic_name)
        try:
            ua_obj = inputs[0] if inputs else get_ua_class(self._topic_class)()
            self._pub_handler.publish(to_ros_msg(ua_obj, self._topic_class(), self._msg_dict))
            rospy.loginfo('Topic publishing succeeded!')
        except Exception as e:
            rospy.logerr('Error when publishing topic ' + self._topic, e)

    def get_status_node(self):
        return self._topic

    def get_publish_node(self):
        return self._method

    def delete_node(self):
        self._sub_handler.unregister()
        self._method.delete()
        self._topic.delete()
        self._pub_handler.unregister()
        rospy.loginfo('Deleted ROS Topic: ' + self._topic_name)
