cmake_minimum_required(VERSION 2.8.3)
project(drrobot_h20_arm_player)

find_package(catkin REQUIRED COMPONENTS DrRobotMotionSensorDriver roscpp roslib rospy tf message_generation pluginlib moveit_core moveit_ros_planning_interface moveit_ros_perception interactive_markers actionlib trajectory_msgs control_msgs controller_manager controller_interface hardware_interface moveit_ros_planning_interface geometry_msgs actionlib_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
    ArmCmd.msg
		arm_cmd.msg
		joint_cmd.msg
)

add_action_files(
DIRECTORY action
FILES
arm.action)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
   geometry_msgs  # Or other packages containing msgs
   sensor_msgs
   trajectory_msgs
   control_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES drrobot_h20_arm_player
   CATKIN_DEPENDS DrRobotMotionSensorDriver roscpp roslib rospy tf message_runtime actionlib_msgs sensor_msgs trajectory_msgs control_msgs hardware_interface controller_manager controller_interface moveit_ros_move_group geometry_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

find_package(LibXML2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
set(LIBS ${LIBS} ${LIBXML2_LIBRARIES})

## Declare a cpp library
add_library(drrobot_h20_arm_player src/test.cpp)
target_link_libraries(drrobot_h20_arm_player ${catkin_LIBRARIES})
add_dependencies(drrobot_h20_arm_player drrobot_h20_arm_player_gencpp)

 install(TARGETS drrobot_h20_arm_player
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY $(PROJECT_NAME)/include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".svn" EXCLUDE
 )

#add_executable(spawn_obj src/spawn_obj.cpp)
#target_link_libraries(spawn_obj ${catkin_LIBRARIES})
#add_dependencies(spawn_obj drrobot_h20_arm_player_gencpp)


add_executable(arm_player_eff src/arm_player_eff.cpp)
target_link_libraries(arm_player_eff ${catkin_LIBRARIES})
add_dependencies(arm_player_eff drrobot_h20_arm_player_gencpp)

add_executable(ArmMovement src/ArmMovement.cpp
src/ArmOptimizer.cpp
src/scoreEnergy.cpp
src/dynaMatM.cpp
src/dynaMatC.cpp
src/dynaMatG.cpp)

target_link_libraries(ArmMovement ${catkin_LIBRARIES})
add_dependencies(ArmMovement drrobot_h20_arm_player_gencpp)

add_executable(ArmMovementClient src/ArmMovementClient.cpp)
target_link_libraries(ArmMovementClient ${catkin_LIBRARIES})
add_dependencies(ArmMovementClient drrobot_h20_arm_player_gencpp)

add_executable(classtest src/test.cpp)
target_link_libraries(classtest ${catkin_LIBRARIES})
add_dependencies(classtest drrobot_h20_arm_player_gencpp)

add_executable(DrRobotArmPlayer src/DrRobot_Arm_Player.cpp)
target_link_libraries(DrRobotArmPlayer ${catkin_LIBRARIES})
add_dependencies(DrRobotArmPlayer drrobot_h20_arm_player_gencpp)
