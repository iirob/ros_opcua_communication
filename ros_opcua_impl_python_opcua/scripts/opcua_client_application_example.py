import time
import rospy
from ros_opcua_msgs.msg import *
from ros_opcua_srvs.srv import *


def connect_server():
    # Call connect
    proxy = rospy.ServiceProxy('/opcuaclient/connect', Connect)
    address = ConnectRequest()
    address.endpoint = 'opc.tcp://0.0.0.0:4840/freeopcua/server/'
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


def generate_multiply_request():
    # Make ua service request
    req = CallMethodRequest()
    # make sure the node id here are correct
    # list node service may help here.
    req.node.nodeId = 'ns=2;i=7'
    req.method.nodeId = 'ns=2;i=18'
    # here is our special msg class to handle any kinds of data types
    data1 = TypeValue()
    data1.type = 'int64'
    data1.int64_d = 2
    data2 = TypeValue()
    data2.type = 'int64'
    data2.int64_d = 3
    req.data.append(data1)
    req.data.append(data2)

    return req


def list_node():
    proxy = rospy.ServiceProxy('/opcuaclient/list_node', ListNode)
    node_to_list = ListNodeRequest()
    node_to_list.node.nodeId = 'ns=2;i=7'
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


def execute_method_call(req):
    # call method
    proxy = rospy.ServiceProxy('/opcuaclient/call_method', CallMethod)
    response = proxy(req)
    if response.success:
        print('\nThe result of the multiplication is: ' + str(response.data[0].int64_d))
    else:
        raise Exception(response.error_message)
    proxy.close()


def read():
    node = ReadRequest()
    node.node.nodeId = 'ns=2;i=10'

    proxy = rospy.ServiceProxy('/opcuaclient/read', Read)
    response = proxy(node)
    if response.success:
        print('\nValue of the node is: ' + str(getattr(response.data, response.data.type + '_d')))
    else:
        raise Exception(response.error_message)
    proxy.close()


def write():
    node = WriteRequest()
    node.node.nodeId = 'ns=2;i=10'
    node.data.type = 'string'
    node.data.string_d = 'new string'

    proxy = rospy.ServiceProxy('/opcuaclient/write', Write)
    response = proxy(node)
    if response.success:
        print('Write in successful')
    else:
        raise Exception(response.error_message)
    proxy.close()


def subscribe():
    node = SubscribeRequest()
    node.node.nodeId = 'ns=2;i=8'
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
    node.node.nodeId = 'ns=2;i=8'

    proxy = rospy.ServiceProxy('/opcuaclient/unsubscribe', Unsubscribe)
    response = proxy(node)
    if response.success:
        print('\nUnsubscribe successful')
    else:
        raise Exception(response.error_message)
    proxy.close()


if __name__ == '__main__':
    try:
        connect_server()
        time.sleep(1)

        list_node()
        time.sleep(1)

        ua_req = generate_multiply_request()
        execute_method_call(ua_req)
        time.sleep(1)

        read()
        time.sleep(1)
        write()
        time.sleep(1)
        read()
        time.sleep(1)

        subscribe()
        time.sleep(10)
        unsubscribe()

        disconnect_server()
    except Exception as e:
        print(str(e))
