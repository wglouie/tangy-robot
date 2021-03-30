cmake_minimum_required(VERSION 2.8.3)
project(user_tracking)

find_package(catkin REQUIRED COMPONENTS face_detection actionlib cv_bridge image_transport message_generation roscpp rospy std_msgs)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

#######################################
## Declare ROS messages and services ##
#######################################

add_action_files(
  DIRECTORY action
  FILES
  UserTracking.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS actionlib_msgs std_msgs
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

include_directories(/usr/local/include/OMRON)

add_executable(UserTracking src/UserTracking.cpp)
add_dependencies(UserTracking HandDetection_gencpp)
add_dependencies(UserTracking user_tracking_gencpp)
target_link_libraries(UserTracking ${catkin_LIBRARIES})

add_executable(UserTrackingServer src/UserTrackingServer.cpp)
add_dependencies(UserTrackingServer HandDetection_gencpp)
add_dependencies(UserTrackingServer user_tracking_gencpp)
target_link_libraries(UserTrackingServer ${catkin_LIBRARIES})

add_executable(UserTrackingClient src/UserTrackingClient.cpp)
add_dependencies(UserTrackingClient user_tracking_gencpp)
#add_dependencies(UserTracking HandDetection_gencpp)
target_link_libraries(UserTrackingClient ${catkin_LIBRARIES}) 

