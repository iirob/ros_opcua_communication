# ros_opcua_impl_freeopcua

Ths package provides OPC UA client implementation for ROS. The client can connect to an OPC UA server and access simple data from it.

## Getting started with ros_opcua_impl_freeopcua

To get start with OPC UA client implementation from this repository first an OPC UA server should be started. For testing one can use example server from `python_opcua` libraray. You can start it using following commands:

```
roscd ros_opcua_impl_python_opcua/python-opcua/examples/
python server-example.py
```
Then start your client using:
```
roslaunch ros_opcua_impl_freeopcua client.launch
```
Use following services to connect to a server, read and write the data:
```
rosservice call /opcua/opcua_client/connect "endpoint: 'opc.tcp://localhost:4840'"
rosservice call /opcua/opcua_client/read "node:
  nodeId: 'ns=2;i=8'
  qualifiedName: 'MyVariable'"
rosservice call /opcua/opcua_client/write "node:
  nodeId: 'ns=2;i=8'
  qualifiedName: 'MyVariable'
data: {type: 'float64', bool_d: false, int8_d: 0, uint8_d: 0, int16_d: 0, uint16_d: 0, int32_d: 0,
  uint32_d: 0, int64_d: 0, uint64_d: 0, float_d: 0.0, double_d: 3.1, string_d: ''}"
```
After this lines oyou should see value changes in OPC UA server of the variable `Objects->MyObject->MyVariable` to `3.1`.

**NOTE**: Here is server run on the same computer as the server. If they are running on diffrent computers, instead of `localhost` insert DNS name or IP address of the computer where server is running. It is also recommended to cross check all values and server using *UaExpert OPC UA client*.

### Compiling of freeopcua libraray (Not needed - it is done automatically)

This package implements bindings for freeopcua - Open Source C++ OPC-UA Server and Client Library.

First you need to build FreeOpcUa with following commands (asume that you are in FreeOpcUa folder):

```sh
mkdir build
cd build
cmake ..
make
```

And change `FreeOpcUa_LIBRARIES` variable to be compatible with your environment.

Enjoy!
