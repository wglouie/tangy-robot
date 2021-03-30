cmake_minimum_required(VERSION 2.8.3)
project(drrobot_h20_player)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS DrRobotMotionSensorDriver geometry_msgs joy nav_msgs roscpp roslib rospy sensor_msgs std_msgs tf message_generation)

## Generate messages in the 'msg' folder
  add_message_files(
    FILES
    CustomSensor.msg
    MotorInfo.msg
    MotorInfoArray.msg
    PowerInfo.msg
    Range.msg
    RangeArray.msg
    StandardSensor.msg
    HeadCmd.msg
#   Message1.msg
#   Message2.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   nav_msgs   sensor_msgs   std_msgs
 )

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES drrobot_h20_player
   CATKIN_DEPENDS DrRobotMotionSensorDriver geometry_msgs joy nav_msgs roscpp roslib rospy sensor_msgs std_msgs tf message_runtime
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(drrobot_player src/drrobot_player.cpp
				src/odometry.cpp)
target_link_libraries(drrobot_player ${catkin_LIBRARIES})
add_dependencies(drrobot_player drrobot_h20_player_gencpp)

add_executable(drrobotMsgTester src/drrobotMsgTester.cpp)
target_link_libraries(drrobotMsgTester ${catkin_LIBRARIES})
add_dependencies(drrobotMsgTester drrobot_h20_player_gencpp)

add_executable(drrobot_keyboard_teleop src/drrobot_keyboard_teleop.cpp)
target_link_libraries(drrobot_keyboard_teleop ${catkin_LIBRARIES})
add_dependencies(drrobot_keyboard_teleop drrobot_h20_player_gencpp)

add_executable(drrobot_joystick_teleop src/drrobot_joystick_teleop.cpp)
target_link_libraries(drrobot_joystick_teleop ${catkin_LIBRARIES})
add_dependencies(drrobot_joystick_teleop drrobot_h20_player_gencpp)

add_executable(drrobot_head_keyboard_teleop src/drrobot_head_keyboard_teleop.cpp)
target_link_libraries(drrobot_head_keyboard_teleop ${catkin_LIBRARIES})
add_dependencies(drrobot_head_keyboard_teleop drrobot_h20_player_gencpp)

#add_executable(odometry_source src/odometry_source.cpp)
#target_link_libraries(odometry_source ${catkin_LIBRARIES})


