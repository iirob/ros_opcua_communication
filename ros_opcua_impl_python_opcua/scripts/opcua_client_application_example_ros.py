import time
from io import BytesIO
import rospy
import roslib.message
from ros_opcua_msgs.msg import *
from ros_opcua_srvs.srv import *


def serialize(ros_obj):
    buff = BytesIO()
    ros_obj.serialize(buff)
    return str(buff.getvalue())


def deserialize(ros_obj, data):
    return ros_obj.deserialize(data)


def connect_server():
    # Call connect
    proxy = rospy.ServiceProxy('/opcuaclient/connect', Connect)
    address = ConnectRequest()
    address.endpoint = 'opc.tcp://0.0.0.0:21554/ROSServer'
    result = proxy(address)
    if result.success:
        print('\nConnected!')
    else:
        raise Exception(result.error_message)
    proxy.close()


def disconnect_server():
    # call disconnect ua server
    proxy = rospy.ServiceProxy('/opcuaclient/disconnect', Disconnect)
    result = proxy()
    if result.success:
        print('\nDisconnected!')
    else:
        raise Exception(result.error_message)
    proxy.close()


def list_node():
    proxy = rospy.ServiceProxy('/opcuaclient/list_node', ListNode)
    node_to_list = ListNodeRequest()
    node_to_list.node.nodeId = 'ns=2;i=53'
    response = proxy(node_to_list)
    if response.success:
        for node in response.children:
            print('\nNode ID: ' + node.nodeId)
            print('Node Name: ' + node.qualifiedName)
            print('Additional Information:')
            for info in node.nodeInfo:
                print(info)
    else:
        raise Exception(response.error_message)
    proxy.close()


def generate_spawn_request():
    # Make sure the turtlesim node is started.
    # Sample of calling spawn and kill.
    # Get service class
    req_type = 'turtlesim/SpawnRequest'
    spawn_req = roslib.message.get_service_class(req_type)()
    # Set value
    spawn_req.name = 'hello_world'
    spawn_req.x = 1
    spawn_req.y = 1
    spawn_req.theta = 1

    # Make ua service request
    req = CallMethodRequest()
    # make sure the node id here are correct
    # list node service may help here.
    req.node.nodeId = 'ns=2;i=53'
    req.method.nodeId = 'ns=2;i=61'
    # here is our special msg class to handle any kinds of data types
    data = TypeValue()
    # set the value and serialized object
    data.type = req_type
    data.string_d = serialize(spawn_req)
    req.data.append(data)
    return req


def generate_kill_request():
    req_type = 'turtlesim/KillRequest'
    kill_req = roslib.message.get_service_class(req_type)()
    kill_req.name = 'hello_world'

    # Make ua service request
    req = CallMethodRequest()
    req.node.nodeId = 'ns=2;i=53'
    req.method.nodeId = 'ns=2;i=58'
    data = TypeValue()
    data.type = req_type
    data.string_d = serialize(kill_req)
    req.data.append(data)
    return req


def execute_method_call(req):
    proxy = rospy.ServiceProxy('/opcuaclient/call_method', CallMethod)
    response = proxy(req)
    if response.success:
        # deserialize the response
        if response.data:
            result = roslib.message.get_service_class(response.data[0].type)()
            result = deserialize(result, response.data[0].string_d)
        else:
            result = []
        print('\nCall method succeeded, result is:' + str(result))
    else:
        raise Exception(response.error_message)
    proxy.close()


def read():
    """
    read one node with extension object
    :return:
    """
    node = ReadRequest()
    node.node.nodeId = 'ns=2;i=81'

    proxy = rospy.ServiceProxy('/opcuaclient/read', Read)
    response = proxy(node)
    if response.success:
        method_class = roslib.message.get_service_class(response.data.type)
        if not method_class:
            method_class = roslib.message.get_message_class(response.data.type)
        result = deserialize(method_class(), response.data.string_d)
        print('\nValue of the node is:\n' + str(result))
    else:
        raise Exception(response.error_message)
    proxy.close()


def write():
    """
    normally the node with extension objects in server are not writeable
    if one node with extension object should be written, the serialize method above should be called.
    :return:
    """
    node = WriteRequest()
    node.node.nodeId = 'ns=2;i=95'
    node.data.type = 'int64'
    node.data.int64_d = 200

    proxy = rospy.ServiceProxy('/opcuaclient/write', Write)
    response = proxy(node)
    if response.success:
        print('Write in successful')
    else:
        raise Exception(response.error_message)
    proxy.close()


def subscribe():
    """
    for parsing extension object, the serialize method above should be called.
    :return:
    """
    node = SubscribeRequest()
    node.node.nodeId = 'ns=2;i=81'
    node.callback_topic = 'test'

    proxy = rospy.ServiceProxy('/opcuaclient/subscribe', Subscribe)
    response = proxy(node)
    if response.success:
        print('\nSubscription successful, data change will be published under ROS topic ' + node.callback_topic)
    else:
        raise Exception(response.error_message)
    proxy.close()


def unsubscribe():
    node = UnsubscribeRequest()
    node.node.nodeId = 'ns=2;i=81'

    proxy = rospy.ServiceProxy('/opcuaclient/unsubscribe', Unsubscribe)
    response = proxy(node)
    if response.success:
        print('\nUnsubscribe successful')
    else:
        raise Exception(response.error_message)
    proxy.close()


if __name__ == '__main__':
    """
    make sure the node ids are correct
    make sure the turtlesim node are started
    make sure the ros_opc_ua_client is running
    """
    try:
        connect_server()
        time.sleep(1)

        list_node()
        time.sleep(1)

        # call spawn service
        ua_req = generate_spawn_request()
        execute_method_call(ua_req)
        time.sleep(1)
        # call kill service
        ua_req = generate_kill_request()
        execute_method_call(ua_req)
        time.sleep(1)

        read()
        time.sleep(1)
        write()
        time.sleep(1)

        subscribe()
        time.sleep(10)
        unsubscribe()

        disconnect_server()

    except Exception as e:
        print(str(e))
