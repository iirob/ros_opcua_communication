import time
from io import BytesIO
import rospy
import roslib.message
from ros_opcua_msgs.msg import *
from ros_opcua_srvs.srv import *


def test_ros_ua_server():
    """
    make sure the node ids are correct
    make sure the turtlesim node are started
    make sure the ros_opcua_viewer is running
    :return:
    """
    # Call connect
    connect = rospy.ServiceProxy('/opcuaclient/connect', Connect)
    address = ConnectRequest()
    address.endpoint = 'opc.tcp://0.0.0.0:21554/ROSServer'
    connect(address)
    print('Connected!')
    time.sleep(1)

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
    # Serialize the request data
    buff = BytesIO()
    spawn_req.serialize(buff)
    serialized_obj = str(buff.getvalue())

    # Make ua service request
    ua_req = CallMethodRequest()
    ua_req.node = Address()
    # make sure the node id here are correct
    # list node service may help here.
    ua_req.node.nodeId = 'ns=2;i=53'
    ua_req.method = Address()
    ua_req.method.nodeId = 'ns=2;i=74'
    # here is our special msg class to handle any kinds of data types
    data = TypeValue()
    # set the value and serialized object
    data.type = req_type
    data.string_d = serialized_obj
    ua_req.data.append(data)
    # call method
    proxy = rospy.ServiceProxy('/opcuaclient/call_method', CallMethod)
    response = proxy(ua_req)
    # deserialize the response
    spawn_response = roslib.message.get_service_class(response.data[0].type)()
    result = spawn_response.deserialize(response.data[0].string_d)
    print('Spawn turtle, Success:{}'.format(response.success))
    print(result)
    time.sleep(3)

    # call kill service
    req_type = 'turtlesim/KillRequest'
    kill_req = roslib.message.get_service_class(req_type)()
    kill_req.name = spawn_req.name
    # serialize
    buff_k = BytesIO()
    kill_req.serialize(buff_k)
    # set value
    ua_req.method.nodeId = 'ns=2;i=71'
    ua_req.data.pop()
    data = TypeValue()
    data.type = req_type
    data.string_d = str(buff_k.getvalue())
    ua_req.data.append(data)
    # call method
    response = proxy(ua_req)
    print('Kill the spawn, Success:{}'.format(response.success))
    time.sleep(1)

    # call disconnect ua server
    disconnect = rospy.ServiceProxy('/opcuaclient/disconnect', Disconnect)
    disconnect()
    print('Disconnected!')


def test_normal_ua_server():
    """
    the to be tested server is written in python_opcua/examples/server-example.py
    :return:
    """
    # Call connect
    connect = rospy.ServiceProxy('/opcuaclient/connect', Connect)
    address = ConnectRequest()
    address.endpoint = 'opc.tcp://0.0.0.0:4840/freeopcua/server/'
    connect(address)
    print('Connected!')
    time.sleep(1)

    # Make ua service request
    ua_req = CallMethodRequest()
    ua_req.node = Address()
    # make sure the node id here are correct
    # list node service may help here.
    ua_req.node.nodeId = 'ns=2;i=7'
    ua_req.method = Address()
    ua_req.method.nodeId = 'ns=2;i=18'
    # here is our special msg class to handle any kinds of data types
    data1 = TypeValue()
    data1.type = 'int64'
    data1.int64_d = 2

    data2 = TypeValue()
    data2.type = 'int64'
    data2.int64_d = 3
    ua_req.data.append(data1)
    ua_req.data.append(data2)
    # call method
    proxy = rospy.ServiceProxy('/opcuaclient/call_method', CallMethod)
    response = proxy(ua_req)
    print(response.data[0].int64_d)

    # call disconnect ua server
    disconnect = rospy.ServiceProxy('/opcuaclient/disconnect', Disconnect)
    disconnect()
    print('Disconnected!')


if __name__ == '__main__':
    try:
        # test_ros_ua_server()
        test_normal_ua_server()
    except Exception as e:
        print(str(e))
