cmake_minimum_required(VERSION 2.8.3)
project(debug)

find_package(catkin REQUIRED COMPONENTS 
   actionlib 
   roscpp 
   move_base_msgs    
   nav_msgs
   std_msgs     
   message_generation
   tf
   message_filters
#packages for costmap plugin
   costmap_2d
   dynamic_reconfigure    
) 

##add_service_files(
##   FILES
##   CheckNavigationGoal.srv
##)

##generate_messages(
##  DEPENDENCIES
##  geometry_msgs std_msgs  actionlib_msgs move_base_msgs visualization_msgs
##)

catkin_package(
   INCLUDE_DIRS include/tangy_move
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(debug src/debug.cpp)
target_link_libraries(debug ${catkin_LIBRARIES})
