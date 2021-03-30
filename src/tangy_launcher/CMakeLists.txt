# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(tangy_launcher) #battery_reader is the name of the file/package

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS drrobot_h20_player roscpp rospy std_msgs genmsg) 
## Declare ROS messages and services
##add_message_files(FILES	batterylevel.msg powerstatus.msg) 

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs) 

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS 
#  LIBRARIES clock_msgs
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(talker src/batteryread.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker drrobot_h20_player_generate_messages_cpp)

##add_executable(batread src/batteryread.cpp)
##target_link_libraries(batread ${catkin_LIBRARIES})
##add_dependencies(batread drrobot_h20_player_generate_messages_cpp)

# %EndTag(FULLTEXT)%
