#!/usr/bin/python
import io
import codecs

import rospy
import roslib.message
from ros_opcua_msgs.msg import *
from ros_opcua_srvs.srv import *

from opcua import ua, Client
from opcua.common.type_dictionary_buider import get_ua_class

from ros_opc_ua_comm import ros_build_in_types, to_ua_class, to_ros_msg


def _get_value(data):
    if data.type in ros_build_in_types:
        return getattr(data, data.type + '_d')
    elif not data.type:
        return
    else:
        data_class = roslib.message.get_message_class(data.type)
        if not data_class:
            data_class = roslib.message.get_service_class(data.type)
        escaped_value = codecs.escape_decode(data.string_d)[0]
        ros_obj = data_class().deserialize(escaped_value)
        return to_ua_class(ros_obj, get_ua_class(data.type)())


def _set_value(data, msg_dict, ros_msg_name=None):
    value = TypeValue()
    if hasattr(data, 'ua_types'):
        ros_class = roslib.message.get_message_class(ros_msg_name)
        if not ros_class:
            ros_class = roslib.message.get_service_class(ros_msg_name)
        ros_msg = to_ros_msg(data, ros_class(), msg_dict)
        value.type = ros_msg_name
        serializer = io.BytesIO()
        ros_msg.serialize(serializer)
        value.string_d = serializer.getvalue()
    else:
        var_type = ua.Variant(data).VariantType
        for k, v in ros_build_in_types.items():
            if v == var_type:
                value.type = k
                setattr(value, k + '_d', data)
                break
    return value


def _get_name_in_argument(node):
    for child in node.get_children():
        if child.get_browse_name().Name == 'OutputArguments':
            return child.get_value()[0].Name


class SubHandler:
    def __init__(self, pub, ros_msg_name, msg_dict):
        self._pub = pub
        self._ros_msg_name = ros_msg_name
        self._msg_dict = msg_dict

    def datachange_notification(self, *val):
        self._pub.publish(_set_value(val[1], self._msg_dict, self._ros_msg_name))


class ROSOPCViewer:
    """
    Note that the name opcuaclient is in exclude list, launch this ros node makes no change on ua server.
    """
    def __init__(self):
        self._client = None
        self._connected = False
        self._node_name = 'opcuaclient'
        self._sub_handlers = {}
        self._topics = {}
        self._ros_type_dict = {}

    def __enter__(self):
        rospy.init_node(self._node_name, log_level=rospy.INFO)
        self._setup_services()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._disconnect(None)

    def _retrieve_data_types(self, node):
        for msg_node in node.get_children(refs=ua.ObjectIds.Organizes):
            for msg_folder in msg_node.get_children(refs=ua.ObjectIds.Organizes):
                for msg in msg_folder.get_children(refs=ua.ObjectIds.Organizes):
                    self._ros_type_dict[msg.get_browse_name().Name] = msg.get_display_name().Text

    def _setup_ros_server(self):
        data_types = self._client.get_node(ua.TwoByteNodeId(ua.ObjectIds.DataTypesFolder))
        for node in data_types.get_children():
            if node.get_browse_name().Name == 'ROSDataType':
                rospy.loginfo(' ----- ROS UA server detected! ------ ')
                self._retrieve_data_types(node)
                break

    def _setup_services(self):
        rospy.Service(self._node_name + '/connect', Connect, self._connect)
        rospy.Service(self._node_name + '/disconnect', Disconnect, self._disconnect)
        rospy.Service(self._node_name + '/list_node', ListNode, self._list_node)
        rospy.Service(self._node_name + '/call_method', CallMethod, self._call_method)
        rospy.Service(self._node_name + '/read', Read, self._read)
        rospy.Service(self._node_name + '/write', Write, self._write)
        rospy.Service(self._node_name + '/subscribe', Subscribe, self._subscribe)
        rospy.Service(self._node_name + '/unsubscribe', Unsubscribe, self._unsubscribe)

    def _connect(self, request):
        response = ConnectResponse()
        try:
            self._client = Client(request.endpoint)
            self._client.connect()
            self._client.load_type_definitions()
            self._root = self._client.get_objects_node()
            self._setup_ros_server()
            rospy.loginfo(' ----- Client {} connected! ------ '.format(request.endpoint))
            self._connected = True
            response.success = True
        except Exception as error:
            rospy.logerr(' ----- Connection failed! ------ ')
            response.success = False
            response.error_message = str(error)
        finally:
            return response

    def _disconnect(self, _):
        response = DisconnectResponse()
        if self._connected:
            try:
                self._client.disconnect()
                rospy.loginfo(' ----- Client disconnected! ------ ')
                response.success = True
            except Exception as error:
                rospy.logerr(' ----- Connection failed! ------ ')
                response.success = False
                response.error_message = str(error)
        else:
            response.success = False
            response.error_message = 'Client is not connected yet!'
        return response

    def _add_addition_info(self, node, address):
        node_type_id = node.get_type_definition()
        if node_type_id:
            node_type = self._client.get_node(node.get_type_definition()).get_browse_name()
            address.nodeInfo.append('Type:' + node_type.Name.replace('Type', ''))
            if node.get_node_class() == ua.NodeClass.Variable:
                data_type = self._client.get_node(node.get_data_type()).get_display_name().Text
                address.nodeInfo.append('DataType:' + data_type)
        elif node.get_node_class() == ua.NodeClass.Method:
            address.nodeInfo.append('Type:Method')
            for child in node.get_children():
                if child.get_browse_name().Name == 'InputArguments':
                    input = child.get_value()[0].Name
                    if input:
                        address.nodeInfo.append('InputDataType:' + input)

    def _list_node(self, request):
        response = ListNodeResponse()
        try:
            if request.node.nodeId == '':
                root = self._root
            else:
                root = self._client.get_node(request.node.nodeId)
            for node in root.get_children():
                address = Address()
                address.nodeId = str(node.nodeid)
                address.qualifiedName = node.get_browse_name().Name
                self._add_addition_info(node, address)
                response.children.append(address)
            rospy.loginfo(' ----- List Node "{}" successful! ------ '.format(root.get_browse_name().Name))
            response.success = True
        except Exception as error:
            rospy.logerr(' ----- List Node failed! ------ ')
            response.success = False
            response.error_message = str(error)
        finally:
            return response

    def _call_method(self, request):
        response = CallMethodResponse()
        try:
            node = self._client.get_node(request.node.nodeId)
            method_node = self._client.get_node(request.method.nodeId)
            ros_msg_name = _get_name_in_argument(method_node)
            inputs = []
            for data in request.data:
                data = _get_value(data)
                if data:
                    inputs.append(data)
            outputs = node.call_method(method_node, *inputs)
            if outputs:
                if isinstance(outputs, list):
                    for output in outputs:
                        response.data.append(_set_value(output, self._ros_type_dict, ros_msg_name))
                else:
                    response.data.append(_set_value(outputs, self._ros_type_dict, ros_msg_name))
            rospy.loginfo(' ----- Call Method {} successful! ------ '.format(node.get_browse_name().Name))
            response.success = True
        except Exception as error:
            rospy.logerr(' ----- Call Method failed! ------ ')
            response.success = False
            response.error_message = str(error)
        finally:
            return response

    def _get_name_in_data_type(self, node):
        return self._client.get_node(node.get_data_type()).get_display_name().Text

    def _read(self, request):
        response = ReadResponse()
        try:
            node = self._client.get_node(request.node.nodeId)
            response.data = _set_value(node.get_value(), self._ros_type_dict, self._get_name_in_data_type(node))
            rospy.loginfo(' ----- Read value of Node {} successful! ------ '.format(node.get_browse_name().Name))
            response.success = True
        except Exception as error:
            rospy.logerr(' ----- Read value failed! ------ ')
            response.success = False
            response.error_message = str(error)
        finally:
            return response

    def _write(self, request):
        response = WriteResponse()
        try:
            node = self._client.get_node(request.node.nodeId)
            node.set_value(_get_value(request.data))
            rospy.loginfo(' ----- Write value of Node {} successful! ------ '.format(node.get_browse_name().Name))
            response.success = True
        except Exception as error:
            rospy.logerr(' ----- Write value failed! ------ ')
            response.success = False
            response.error_message = str(error)
        finally:
            return response

    def _subscribe(self, request):
        response = SubscribeResponse()
        try:
            node = self._client.get_node(request.node.nodeId)
            pub = rospy.Publisher(request.callback_topic, TypeValue, queue_size=10)
            self._topics[request.node.nodeId] = pub
            sub_handler = self._client.create_subscription(500, SubHandler(pub, self._get_name_in_data_type(node),
                                                                           self._ros_type_dict))
            sub_handler.subscribe_data_change(node)
            self._sub_handlers[request.node.nodeId] = sub_handler
            rospy.loginfo(' ----- Subscribe to Node {} successful! ------ '.format(node.get_browse_name().Name))
            response.success = True
        except Exception as error:
            rospy.logerr(' ----- Subscribe failed! ------ ')
            response.success = False
            response.error_message = str(error)
        finally:
            return response

    def _unsubscribe(self, request):
        response = UnsubscribeResponse()
        if request.node.nodeId in self._topics and request.node.nodeId in self._sub_handlers:
            try:
                self._topics[request.node.nodeId].unregister()
                self._sub_handlers[request.node.nodeId].delete()
                del self._topics[request.node.nodeId]
                del self._sub_handlers[request.node.nodeId]
                rospy.loginfo(' ----- Unsubscribe successful! ------ ')
                response.success = True
            except Exception as error:
                rospy.logerr(' ----- Subscribe failed! ------ ')
                response.success = False
                response.error_message = str(error)
        else:
            response.success = False
            response.error_message = 'No such a subscription!'
        return response


if __name__ == '__main__':
    try:
        with ROSOPCViewer():
            rospy.loginfo(' ----- ROS OPC UA Viewer started! ------ ')
            rospy.spin()
    except Exception as e:
        print(e.message)
