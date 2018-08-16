# ros_opcua_impl_python_opcua

This package provides OPC UA server implementation for connection to ROS. Server provides access to all ROS topics, services and actions from an  OPC UA Client. 

## Getting started with ros_opcua_impl_python_opcua package

First start some ROS Nodes for something to be visalized in the server. For example one can start ``turtlesim_node`:
```
rosrun turtlesim turtlesim_node 
```

Then start the server (the starting will take some time, wait until you see the output) using:
```
roslaunch ros_opcua_impl_python_opcua rosopcua.launch
```

Then start the *UaExpert OPC UA* Client and connect to the server. Server can be accessed per default from wholw local network on port `21554`. If you are running everything localy use `opc.tcp://localhost:21554` as server URL.

After successful connection you can see all ROS Services, Topics and Actions mapped to the OPC UA. To move the turtle from exaple choose `Objects->ROS-Services->turtle1/teleport_absolute` with right clieck and choose call. Enter the new possitionof the turtle and see how turtle moves.

In `Objects->ROS-Topics->turtle1->pose` one can follow the position of the turtle in real time. To check the full effect of this try to move turtle using [Robot Steering](https://wiki.ros.org/rqt_robot_steering) rqt-Plugin.
