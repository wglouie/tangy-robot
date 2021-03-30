cmake_minimum_required(VERSION 2.8.3)
project(robot_inverse_kinematics)

find_package(catkin REQUIRED COMPONENTS skeletonmsgs_nu tf geometry_msgs message_generation std_msgs collision_avoider)

add_service_files(DIRECTORY srv FILES execute_gesture_file.srv)

#add_action_files(
#   DIRECTORY action
#   FILES execute_file.action
#)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(CATKIN_DEPENDS message_runtime std_msgs skeletonmsgs_nu geometry_msgs collision_avoider)

catkin_install_python(PROGRAMS
  scripts/kinect_ik.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robot_inverse_kinematics
)

#cmake_minimum_required(VERSION 2.8.3)
#project(action_executor)

#find_package(catkin REQUIRED COMPONENTS
#    roscpp
#    roslib
#    rospy
#    std_msgs
#    message_generation
#    actionlib
#    actionlib_msgs
#)

#include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#)


#add_message_files(
#   FILES
#   message_name.msg
#)


#add_service_files(
#        FILES
#	activity_learner_service.srv
#	AddTwoInts.srv
#)

# Generate added messages and services with any dependencies listed here
#generate_messages(
# DEPENDENCIES
#   actionlib_msgs   std_msgs
#)
