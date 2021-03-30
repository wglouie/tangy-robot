cmake_minimum_required(VERSION 2.8.3)
project(world_state_identifier)

find_package(catkin REQUIRED COMPONENTS  
    roscpp
    roslib
    rospy
    std_msgs
    message_generation
    actionlib
    actionlib_msgs
    bingo_detection
    face_detection
    help_indicators
    tf
    geometry_msgs
    demonstration_learning_msgs
    tangy_bringup
)

#add_definitions(-std=c++11) ##http://answers.ros.org/question/216842/ros-using-c-11-how-to-use-with-catkin/


include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)


#add_message_files (
#        DIRECTORY msg
#        FILES
#)


#add_service_files(
#	 FILES
#)

add_action_files(
   FILES
   world_state_identifier.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
   actionlib_msgs std_msgs geometry_msgs help_indicators demonstration_learning_msgs
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include include/trivia
#  LIBRARIES trivia
#  CATKIN_DEPENDS actionlib actionlib_msgs drrobot_h20_arm_player face_detection message_generation music_player robot_gui roscpp roslib rospy std_msgs tangy_bringup tangy_move
#  DEPENDS system_lib
#)

catkin_package(
   INCLUDE_DIRS include/world_state_identifier
)


# ADD ALL CPP's YOU USE
#add_library(library_name src/cpp_name.cpp)
#target_link_libraries(library_name ${catkin_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
#add_dependencies(library_name example_package_gencpp library_you_depend_on)

# ADD ALL CPP's YOU USE
add_executable(world_state_identifier_server src/world_state_identifier_server.cpp)
target_link_libraries(world_state_identifier_server tangy_robot_lib FaceDetectionLibrary HelpIndicatorsLibrary ${catkin_LIBRARIES} ${Boost_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
add_dependencies(world_state_identifier_server
    world_state_identifier_server_gencpp
    bingo_detection_gencpp
    face_detection_gencpp
    help_indicators_gencpp
    demonstration_learning_msgs_gencpp)

# Create ActionExecutorClientLibrary
add_library(WorldStateIdentifierClientLibrary src/world_state_identifier_client.cpp)
add_dependencies(WorldStateIdentifierClientLibrary 
world_state_identifier_gencpp 
demonstration_learning_msgs_gencpp)
target_link_libraries(WorldStateIdentifierClientLibrary ${catkin_LIBRARIES} ${OpenCV_LIBS})


# ADD ALL CPP's YOU USE
add_executable(world_state_identifier_client_test src/world_state_identifier_client_test.cpp)
target_link_libraries(world_state_identifier_client_test WorldStateIdentifierClientLibrary ${catkin_LIBRARIES} ${Boost_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
add_dependencies(world_state_identifier_client_test 
world_state_identifier_gencpp 
demonstration_learning_msgs_gencpp)


