cmake_minimum_required(VERSION 2.8.3)
project(opcua_impl_freeopcua)

set(FreeOpcUa_LIBRARIES /home/stogl/workspace/ros_devel/src/opcua_communication/opcua_impl_freeopcua/freeopcua/build/bin/opcua.so)

find_package(catkin REQUIRED
  roscpp
  opcua_msgs
  opcua_srvs
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
 INCLUDE_DIRS freeopcua/include
#  LIBRARIES opcua_impl_freeopcua
 CATKIN_DEPENDS roscpp opcua_msgs opcua_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  freeopcua/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(client_node src/opcua_client.cpp)

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

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS opcua_impl_freeopcua opcua_impl_freeopcua_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_opcua_impl_freeopcua.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
