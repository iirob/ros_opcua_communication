cmake_minimum_required(VERSION 2.8.3)
project(opcua_impl_open62541)

set(open62541_LIBRARIES /home/stogl/workspace/ros_devel/src/opcua_communication/opcua_impl_open62541/open62541/build/libopen62541.so
)

find_package(catkin REQUIRED COMPONENTS
  opcua_msgs
  opcua_srvs
)

catkin_package(
 INCLUDE_DIRS open62541/include open62541/src open62541/examples open62541/build/src_generated
#  LIBRARIES opcua_impl_open62541
 CATKIN_DEPENDS opcua_msgs opcua_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  open62541/include
  open62541/src
  open62541/examples
  open62541/build/src_generated
  ${catkin_INCLUDE_DIRS}
)

# add_executable(server_node src/opcua_server.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(server_node
#   ${catkin_LIBRARIES}
#   ${open62541_LIBRARIES}
# )

#############
## Install ##
#############

# install(TARGETS server_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
