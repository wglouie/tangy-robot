cmake_minimum_required(VERSION 2.8.3)
project(server_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib genmsg actionlib_msgs roscpp rospy std_msgs message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
#   Message1.msg
#   Message2.msg
   Plan.msg
   PlanRequest.msg
   CurrentStateRequest.msg
 )


## Generate services in the 'srv' folder
 add_service_files(
   FILES
#   Service1.srv
#   Service2.srv
    InitialStateRequest.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
#   actionlib_msgs
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES server_msgs
#  CATKIN_DEPENDS actionlib actionlib_msgs roscpp rospy std_msgs
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(include
#  ${catkin_INCLUDE_DIRS}
#)

## Declare a cpp library
# add_library(server_msgs
#   src/${PROJECT_NAME}/server_msgs.cpp
# )


## Declare a cpp executable
# add_executable(server_msgs_node src/server_msgs_node.cpp)
#add_executable(server_msgs_server src/server_msgs_server.cpp)
#add_executable(server_msgs_client src/server_msgs_client_test.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(server_msgs_node server_msgs_generate_messages_cpp)
#add_dependencies(server_msgs_server server_msgs_generate_messages_cpp)
#add_dependencies(server_msgs_client server_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(server_msgs_server
#   ${catkin_LIBRARIES}
# )
#target_link_libraries(server_msgs_client
#   ${catkin_LIBRARIES}
# )


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
# install(TARGETS server_msgs server_msgs_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_server_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
