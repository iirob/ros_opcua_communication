# ros_opcua_impl_python_opcua

This package provides a python implementation of OPC UA server, which models ROS in OPC UA protocol. The server provides access to all ROS topics, services, actions, parameters and nodes, besides an OPC UA client is provided to explore the contents of the server.

## Getting started with ros_opcua_impl_python_opcua package

### Start up

1. After cloning the package locally, an initial build is needed. As a standard ROS package, of course `catkin_make` is the ideal tool to do this, if you are new to ROS, please check [here](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) to figure out how to build the package. After building, please do not forget to **source the package**.

2. Start the server (the starting will take some time, wait until you see the output `server started`) using:

```
roslaunch ros_opcua_impl_python_opcua rosopcua.launch
```

Then a ROS node named `rosopcua` could be seen in ROS, this node takes related ROS contents and models them in OPC UA protocol.

3. Run some ROS Nodes to test, they should afterward visible in the server. For example one can start `turtlesim_node`:

```
rosrun turtlesim turtlesim_node 
```

4. Start the UaExpert as OPC UA client and connect to the server, if you have not installed it locally, please check [here](https://www.unified-automation.com/products/development-tools/uaexpert.html) to download it. The initial address you should type in UaExpert is `opc.tcp://0.0.0.0:21554/ROSServer`, afterward if you want to test with server and client located in different computers, this address can be found in `ros_opcua_impl_python_opcua/basic_server_client.py` under the variant `server_end_point`.

After a successful connection, all ROS topics, services, actions, parameters and nodes mapped to the OPC UA can be seen. 

### Basic operations

1. Call a service. To move the turtle from example, choose `Objects->Services->turtle1/teleport_relative`, right click and choose call. Enter the new position (for instance 1, 1) of the turtle and then the turtle should move.

2. Interact with topics. In `Objects->Topics->turtle1/pose` one can follow the position of the turtle in real time. To check the full effect of this try to move turtle using [Robot Steering](https://wiki.ros.org/rqt_robot_steering) rqt-Plugin, or run:

```
rosrun turtlesim turtle_teleop_key 
```

and move the turtle there. To publish messages under a ROS topic, you may choose `Objects->Topics->Topic publish->/turtle1/color_sensor`, call the method and the background color should be changed.

3. ROS msgs and srvs. They are modeled internally and linked under `Types->DataTypes->ROSDataType`.

## Rough project structure

Below is an overview of the folders and files in this project:

+ config, include the configuration file, will be loaded while launching the server node, a more specific explanation can be found in chapter "Configurations".
+ export, OPC UA nodes could be exported there, not interested for using
+ launch, includes 3 launch files
+ python-opcua, the foundation framework of this project
+ scripts, the main code folder of this project
+ known_issues.md, records the solved and unsolved problems of this project

...

## Interacting with the OPC UA server without `UaExpert`

`scripts/ros_opc_ua_client.py` is a ROS implementation of OPC UA client, which allows to connect with any OPC UA server, to start it, run:

```
roslaunch ros_opcua_impl_python_opcua opcuaclient.launch
```

After this 8 different services available under the ROS node `opcuaclient` can be seen, they can be called with `rosservice` (please **first call connect** service before calling other services) or more leisurely, with python scripts, there are two examples in file `scripts/opcua_client_application_example.py` and `scripts/opcua_client_application_example_ros.py` respectively.

## Configurations

Some parameters are designed in the file `config/params.yaml`, which supports advanced functions, below is a list of them:

| Parameter name | Default value| Comment|
| :---: | :---: |:----------------|
|namespace|'/'|filters of the ros topics, services, actions, parameters and nodes, default '/' means no filter|
|automatic_refresh|true|True will start automatic refresh of ros nodes, otherwise the user should call refresh manually|
|refresh_cycle_time|0.5|The cycle time of the automatic refresh, the time unit is second|
|show_nodes|true|Switch for masking the display of ROS nodes|
|show_topics|true|Switch for masking the display of ROS topics|
|show_services|true|Switch for masking the display of ROS services|
|show_params|true|Switch for masking the display of ROS parameters|
|import_xml_msgs|false|Controls if data types will be imported from an xml file or generate on-the-fly, its address is written in `/scripts/ros_opc_ua_comm.py` under the variable `message_export_path`, the nodes to be imported should be generated with the script `scripts/ros_server_export_message.py`, do not set it to true until the bug mentioned there is fixed|
|parameter_writable|true|Controls if the variables in parameters are writable|

## For developers

### Project hierarchy

The project is organized in a 5 layer hierarchy:

| Layer | Correspondent files | Comment|
| :---: | :---: |:----------------|
|Application|`scripts/opcua_client_application_example.py`, `scripts/opcua_client_application_example_ros.py`|possible application examples|
|Top|`scripts/ros_server.py`, `scripts/ros_server_export_message.py`, `scripts/ros_opc_ua_client.py`|built OPC UA server, manage ROS information with different information managers|
|High|`scripts/ros_info_manage.py`|created ROS information managers to manage the ROS services set, topic set, and ROS node set|
|Middle|`scripts/ros_opc_ua_comm.py` other classes|build the communication proxy of ROS topics, services based on the UA objects|
|Low|`scripts/ros_opc_ua_comm.py` class `OpcUaROSMessage`|created correspondent OPC UA extension object of rosmsg, rossrv|
|Bottom|`python-opcua/opcua/common/type_dictionary_buider.py`| extension object support in python-opcua|

### Msgs and Srvs used in `scripts/ros_opc_ua_client.py`

They can be found under the root folder under `ros_opcua_communication/ros_opcua_msgs` and `ros_opcua_communication/ros_opcua_srvs`.

The msg `TypeValue.msg` is designed for transferring arbitrary data structure between ROS and OPC UA in ROS world, to transfer an ROS object as data, the name of the object should be written in `type` string, the complete object should be serialized and written in the string field `string_d`. Please check `scripts/opcua_client_application_example_ros.py` to figure out detailed usage.

### Tricks that may help

Inheriting the two classes in `scripts/basic_server_client.py` can save time for further developing.

To implement filters in static modeling, it would be better to modify the low layer; in dynamic modeling, it would be better to modify the class `ROSInfoAgent` in the high layer, so that the server does not "see" a specific node, topic, service or parameter.

