cmake_minimum_required(VERSION 2.8.3)
project(activity_learner)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs geometry_msgs message_generation std_msgs demonstration_learning_msgs)

#add_message_files(DIRECTORY msg FILES Floats.msg HeaderString.msg)
add_service_files(DIRECTORY srv FILES capture_state_action.srv
                                        predict_action.srv train_policy.srv
                                        create_new_training_set.srv
                                        probabilistic_predict_action.srv
                                        save_trajectory.srv
					get_current_trajectory.srv)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs demonstration_learning_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs actionlib actionlib_msgs geometry_msgs demonstration_learning_msgs)

catkin_install_python(PROGRAMS
  scripts/random_forest_learner.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/activity_learner
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

#add_action_files(
#   FILES
#)

# Generate added messages and services with any dependencies listed here
#generate_messages(
# DEPENDENCIES
#   actionlib_msgs   std_msgs
#)
