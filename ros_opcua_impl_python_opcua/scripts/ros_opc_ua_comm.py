# Thanks to https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel
import rospy
import roslib.message
import rostopic
import rosservice
import actionlib

from opcua import ua, uamethod

from ros_opc_ua import DataTypeDictionaryBuilder, get_ua_class
from ros_global import get_ros_messages, get_ros_services

_ros_build_in_types = {'bool': ua.VariantType.Boolean,
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

_ua_basic_types = [item.name for item in _ros_build_in_types.values()]


def _nodeid_generator(idx):
    return ua.NodeId(namespaceidx=idx)


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


def _to_ros_msg(ua_class, ros_msg):
    for attr in ua_class.ua_types:
        base_type_str, is_array = _extract_ros_array_info(attr[1])
        if base_type_str in _ua_basic_types:
            setattr(ros_msg, attr[0], getattr(ua_class, attr[0]))
        else:
            if is_array:
                for member in getattr(ua_class, attr):
                    new_ros_msg = roslib.message.get_message_class(base_type_str)
                    getattr(ros_msg, attr).append(new_ros_msg)
                    _to_ros_msg(member, new_ros_msg())
            else:
                _to_ros_msg(getattr(ua_class, attr[0]), getattr(ros_msg, attr[0]))
    return ros_msg


def _to_ua_class(ros_msg, ua_class):
    for attr, data_type in zip(ros_msg.__slots__, getattr(ros_msg, '_slot_types')):
        base_type_str, is_array = _extract_ros_array_info(data_type)
        if base_type_str in _ros_build_in_types:
            setattr(ua_class, attr, getattr(ros_msg, attr))
        else:
            if is_array:
                for member in getattr(ros_msg, attr):
                    new_ua_class = get_ua_class(getattr(member, '_type'))()
                    getattr(ua_class, attr).append(new_ua_class)
                    _to_ua_class(member, new_ua_class)
            else:
                _to_ua_class(getattr(ros_msg, attr), getattr(ua_class, attr))
    return ua_class


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


def _process_type(type_name):
    if type_name in _ros_build_in_types:
        return _ros_build_in_types[type_name].name
    return type_name


class OpcUaROSMessage:
    def __init__(self, server, idx, idx_name):
        self._server = server
        self._idx = idx

        self._created_data_types = {}
        self._dict_builder = DataTypeDictionaryBuilder(server, idx, idx_name, 'ROSDictionary')

    def _is_new_type(self, message):
        return message not in _ros_build_in_types and message not in self._created_data_types

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
            base_type_str, is_array = _extract_ros_array_info(data_type)
            if self._is_new_type(base_type_str):
                self._create_data_type(base_type_str)
                self._recursively_create_message(base_type_str)

            self._dict_builder.add_field(_process_type(base_type_str), variable_type, msg, is_array)

    def _create_messages(self):
        messages = get_ros_messages()
        for msg in messages:
            if msg not in self._created_data_types:
                self._recursively_create_message(msg)

    def _process_service_classes(self, srv):
        msg_name = getattr(srv, '_type')
        self._create_data_type(msg_name)
        for variable_type, data_type in zip(srv.__slots__, getattr(srv, '_slot_types')):
            base_type_str, is_array = _extract_ros_array_info(data_type)
            self._dict_builder.add_field(_process_type(base_type_str), variable_type, msg_name, is_array)

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


class OpcUaROSService:

    def __init__(self, service_name, node_root, service_node_id, msg_dict):
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
        rospy.loginfo('Created ROS Service with name: ' + service_name)

    @uamethod
    def _call_service(self, _, *inputs):
        rospy.loginfo('Calling service %s under ROS node: %s'
                      % (self._service_name, self._node_root.get_display_name().Text))
        try:
            ua_class = inputs[0] if inputs else get_ua_class(self._req_name)()
            response = self._proxy(_to_ros_msg(ua_class, self._ros_service_req()))
            rospy.loginfo('Calling service succeeded!')
            # BUG: To deal with bug (BadEndOfStream) in calling methods with empty extension objects
            if response.__slots__:
                resp_ua_class = _to_ua_class(response, get_ua_class(self._resp_name)())
                return ua.Variant(resp_ua_class)
        except Exception as e:
            rospy.logerr('Error when calling service ' + self._service_name, e)

    def delete_node(self):
        self._proxy.close()
        self._method.delete()
        rospy.loginfo('Deleted ROS Service with name: ' + self._service_name)


class OpcUaROSTopic:

    def __init__(self, topic_name, node_root, msg_dict, *node_id):
        self._topic_name = topic_name
        self._topic_class, _, _ = rostopic.get_topic_class(self._topic_name)
        self._msg_name = getattr(self._topic_class, '_type')
        self._topic = node_root.add_variable(node_id[0], self._topic_name, ua.Variant(None, ua.VariantType.Null),
                                             datatype=msg_dict[self._msg_name])
        self._handler = None

    def delete_node(self):
        self._handler.unregister()
        self._topic.delete()
        rospy.loginfo('Deleted ROS Topic publication with name: ' + self._topic_name)


class OpcUaROSTopicPub(OpcUaROSTopic):

    def __init__(self, topic_name, node_root, msg_dict, *node_id):
        OpcUaROSTopic.__init__(self, topic_name, node_root, msg_dict, *node_id)
        rospy.loginfo('Created UA variable for ROS Publication under topic: ' + topic_name)
        self._handler = rospy.Subscriber(topic_name, self._topic_class, self._message_callback)

    def _message_callback(self, message):
        self._topic.set_value(_to_ua_class(message, get_ua_class(self._msg_name)()))


class OpcUaROSTopicSub(OpcUaROSTopic):

    def __init__(self, topic_name, node_root, msg_dict, *node_id):
        OpcUaROSTopic.__init__(self, topic_name, node_root, msg_dict, *node_id)
        self._handler = rospy.Publisher(self._topic_name, self._topic_class, queue_size=1)

        input_arg = _create_args(self._topic_class, msg_dict[self._msg_name])
        self._method = self._topic.add_method(node_id[1], 'publish', self._call_publish, input_arg, [])
        rospy.loginfo('Created UA variable for ROS Subscription under ROS topic: ' + topic_name)

    @uamethod
    def _call_publish(self, _, *inputs):
        rospy.loginfo('Publishing data under ROS topic: %s, under node:' + self._topic_name)
        try:
            ua_class = inputs[0] if inputs else get_ua_class(self._topic_class)()
            self._handler.publish(_to_ros_msg(ua_class, self._topic_class()))
            rospy.loginfo('Topic publishing succeeded!')
            return
        except Exception as e:
            rospy.logerr('Error when publishing topic ' + self._topic, e)

    def delete_node(self):
        OpcUaROSTopic.delete_node(self)
        self._method.delete()


class OpcUaROSAction:

    def __init__(self, idx, action_name, node_root, msg_dict):
        self._idx = idx
        self._action_name = action_name
        self._root = node_root.add_folder(_nodeid_generator(self._idx), 'ROSAction')
        self._dict = msg_dict

        self._goal_class, self._goal = self._get_action_class(self._action_name + '/goal', _nodeid_generator(self._idx))
        self._cancel_class, self._cancel = self._get_action_class(self._action_name + '/cancel',
                                                                  _nodeid_generator(self._idx))
        self._status_class, self._status = self._get_action_class(self._action_name + '/status',
                                                                  _nodeid_generator(self._idx))
        self._result_class, self._result = self._get_action_class(self._action_name + '/result',
                                                                  _nodeid_generator(self._idx))
        self._feedback_class, self._feedback = self._get_action_class(self._action_name + '/feedback',
                                                                      _nodeid_generator(self._idx))

        action_class_name = getattr(self._goal_class, '_type').replace("Goal", "")
        self._action_class = roslib.message.get_message_class(action_class_name)

        rospy.loginfo('Created ROS Action with name: ' + self._action_name)

    def _get_action_class(self, name, node_id):
        action_class, _, _ = rostopic.get_topic_class(name)
        class_name = getattr(action_class, '_type')
        ua_node = self._root.add_variable(node_id, name, ua.Variant(None, ua.VariantType.Null),
                                          datatype=self._dict[class_name])
        return action_class, ua_node

    def delete_node(self):
        self._goal.delete()
        self._cancel.delete()
        self._status.delete()
        self._result.delete()
        self._feedback.delete()
        rospy.loginfo('Deleted ROS Action with name: ' + self._action_name)


class OpcUaROSActionClient(OpcUaROSAction):

    def __init__(self, idx, action_name, node_root, msg_dict):
        OpcUaROSAction.__init__(self, idx, action_name, node_root, msg_dict)


class OpcUaROSActionServer(OpcUaROSAction):

    def __init__(self, idx, action_name, node_root, msg_dict):
        OpcUaROSAction.__init__(self, idx, action_name, node_root, msg_dict)
        try:
            self.client = actionlib.SimpleActionClient(self._action_name, self._action_class)
        except actionlib.ActionException as e:
            rospy.logerr("Error when creating ActionClient for action " + self._action_name, e)

        goal_data_id = self._dict[getattr(self._goal_class, '_type')]
        self._man_send = self._goal.add_method(_nodeid_generator(self._idx), 'send goal', self._send_goal,
                                               _create_args(self._goal_class, goal_data_id), [])

        self._man_cancel = self._cancel.add_method(_nodeid_generator(self._idx), 'cancel goal',
                                                   self._cancel_goal, [], [])

    @uamethod
    def _send_goal(self, _, *inputs):
        rospy.loginfo('Calling method %s under ROS node: %s' % ('"send goal"', self._root.get_display_name().Text))
        try:
            ua_class = inputs[0] if inputs else get_ua_class(self._goal_class)()
            goal_msg = _to_ros_msg(ua_class, self._goal_class())
            rospy.loginfo("Created Message Instances for goal-send: " + goal_msg)
            self.client.send_goal(goal_msg, done_cb=self._update_result, feedback_cb=self._update_feedback,
                                  active_cb=self._update_state)
        except Exception as e:
            rospy.logerr("Error occurred during goal sending for Action " + self._action_name, e)

    @uamethod
    def _cancel_goal(self, *_):
        rospy.loginfo('Calling method %s under ROS node: %s' % (self._action_name, self._root.get_display_name().Text))
        try:
            self.client.cancel_all_goals()
            self._update_state()
        except Exception as e:
            rospy.logerr("Error when cancelling a goal for " + self._action_name, e)

    def _update_result(self, state, result):
        rospy.logdebug("updated result cb reached")
        self._status.set_value(_to_ua_class(state, self._status_class()))
        self._result.set_value(_to_ua_class(result, self._result_class()))

    def _update_state(self):
        self._status.set_value(_to_ua_class(self.client.get_state(), self._status_class()))

    def _update_feedback(self, feedback):
        rospy.logdebug("updated feedback cb reached")
        self._feedback.set_value(_to_ua_class(feedback, self._feedback_class()))

    def delete_node(self):
        self.client.cancel_all_goals()
        self._man_send.delete()
        self._man_cancel.delete()
        OpcUaROSAction.delete_node(self)
