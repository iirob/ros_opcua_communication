cmake_minimum_required(VERSION 2.8.3)
project(opcua_impl_ua_sdk)

set(UA_SDK_BASE_DIR /opt/unifiedautomation/sdk)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  opcua_msgs
  opcua_srvs
)

catkin_package(
 INCLUDE_DIRS include common/include
 LIBRARIES ua_client
 CATKIN_DEPENDS roscpp opcua_msgs opcua_srvs
 DEPENDS uastack
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -g -m32")

link_directories(
  ${UA_SDK_BASE_DIR}/lib
)

include_directories(
  include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${UA_SDK_BASE_DIR}/include/uaclient
  ${UA_SDK_BASE_DIR}/include/uastack
  ${UA_SDK_BASE_DIR}/include/uabase
  ${UA_SDK_BASE_DIR}/include/uapki
  ${UA_SDK_BASE_DIR}/include/uamodels
)


# add_library(ua_client
#   common/src/SPOPCUAClient.cpp
#   common/src/SPOPCUATypes.cpp
#   common/src/SPOPCUASubscription.cpp
#   common/src/SPOPCUAAux.cpp
# )
# target_link_libraries(ua_client
#   uaclient
#   uabase
#   xmlparser
#   uastack
#   uapki
#   crypto
#   xml2
# )
# 
# add_executable(client_node src/opcua_client.cpp)
# target_link_libraries(client_node
#     ua_client
#     ${catkin_LIBRARIES}
# )
# 
# #############
# ## Install ##
# #############
# 
# install(TARGETS ua_client client_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# 
# install(DIRECTORY common/include
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
# 
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
