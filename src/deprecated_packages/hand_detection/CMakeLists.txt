cmake_minimum_required(VERSION 2.8.3)
project(hand_detection)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf message_generation openni_camera actionlib)

find_package(openni_tracker)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES 
  HandDetection.action
)

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(HandDetectionServer src/HandDetectionServer.cpp)
add_dependencies(HandDetectionServer hand_detection_gencpp)
target_link_libraries(HandDetectionServer ${catkin_LIBRARIES})


add_executable(HandDetectionClient src/HandDetectionServer.cpp)
add_dependencies(HandDetectionClient hand_detection_gencpp)
target_link_libraries(HandDetectionClient ${catkin_LIBRARIES})


