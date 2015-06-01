cmake_minimum_required(VERSION 2.8.3)
project(opcua_impl_freeopcua)

set(BUILD_CLIENT ON)
set(BUILD_SERVER ON)
set(BUILD_PYTHON ON)
set(BUILD_TESTING OFF)

add_subdirectory(freeopcua)

set(FreeOpcUa_LIBRARIES opcua)

find_package(catkin REQUIRED
  roscpp
  opcua_msgs
  opcua_srvs
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
 INCLUDE_DIRS include freeopcua/include
#  LIBRARIES opcua_impl_freeopcua
 CATKIN_DEPENDS roscpp opcua_msgs opcua_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  include
  freeopcua/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(client_node src/opcua_client.cpp)

add_dependencies(client_node opcua_msgs_generate_messages_cpp opcua_srvs_generate_messages_cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(opcua_impl_freeopcua_node opcua_impl_freeopcua_generate_messages_cpp)

target_link_libraries(client_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${FreeOpcUa_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS client_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)