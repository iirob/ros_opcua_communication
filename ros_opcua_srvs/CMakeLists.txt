cmake_minimum_required(VERSION 2.8.3)
project(ros_opcua_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ros_opcua_msgs
  std_msgs
  std_srvs
)

add_service_files(
  FILES
  CallMethod.srv
  Connect.srv
  Disconnect.srv
  ListNode.srv
  Read.srv
  Subscribe.srv
  Unsubscribe.srv
  Write.srv
)

generate_messages(
  DEPENDENCIES ros_opcua_msgs std_msgs std_srvs
)

catkin_package(
 CATKIN_DEPENDS message_runtime ros_opcua_msgs std_msgs std_srvs
)
