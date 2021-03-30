cmake_minimum_required(VERSION 2.8.3)
project(action_executor)

find_package(catkin REQUIRED COMPONENTS  
    roscpp
    roslib
    rospy
    std_msgs
    message_generation
    actionlib
    actionlib_msgs
    tangy_bringup
    demonstration_learning_msgs
    robot_inverse_kinematics
    trajectory_executor
    text_to_speech
) 

add_definitions(-std=c++11) ##http://answers.ros.org/question/216842/ros-using-c-11-how-to-use-with-catkin/


include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)


#add_service_files(
#        FILES
#	tts.srv
#)

add_action_files(
   FILES
   action_executor.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
   actionlib_msgs   std_msgs    demonstration_learning_msgs
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
   INCLUDE_DIRS include/action_executor
)


# ADD ALL CPP's YOU USE
#add_library(library_name src/cpp_name.cpp)
#target_link_libraries(library_name ${catkin_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
#add_dependencies(library_name example_package_gencpp library_you_depend_on)

# ADD ALL CPP's YOU USE
add_executable(action_executor_server
  src/action_executor_server.cpp
  src/pugixml.cpp
  src/bingo_number_handler.cpp)
target_link_libraries(action_executor_server
    tangy_robot_lib
    TrajectoryExecutorClientLibrary
    TextToSpeechClientLibrary
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
add_dependencies(action_executor_server 
action_executor_gencpp  
demonstration_learning_msgs_gencpp
robot_inverse_kinematics_gencpp
trajectory_executor_gencpp)

# Create ActionExecutorClientLibrary
add_library(ActionExecutorClientLibrary src/action_executor_client.cpp)
add_dependencies(ActionExecutorClientLibrary
action_executor_gencpp
demonstration_learning_msgs_gencpp
robot_inverse_kinematics_gencpp
trajectory_executor_gencpp)
target_link_libraries(ActionExecutorClientLibrary ${catkin_LIBRARIES} ${OpenCV_LIBS})


# ADD ALL CPP's YOU USE
add_executable(action_executor_client_test src/action_executor_client_test.cpp)
target_link_libraries(action_executor_client_test ActionExecutorClientLibrary ${catkin_LIBRARIES} ${Boost_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
add_dependencies(action_executor_client_test 
action_executor_gencpp
demonstration_learning_msgs_gencpp)
