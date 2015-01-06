cmake_minimum_required(VERSION 2.8.3)
project(opcua_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  opcua_msgs
  std_msgs
  std_srvs
)

add_service_files(
  FILES
  CallMethod.srv
  Connect.srv
  ListNode.srv
  Read.srv
  Subscribe.srv
  Unsubscribe.srv
  Write.srv
)

generate_messages(
  DEPENDENCIES opcua_msgs std_msgs std_srvs
)

catkin_package(
 CATKIN_DEPENDS message_runtime opcua_msgs std_msgs std_srvs
)
