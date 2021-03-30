cmake_minimum_required(VERSION 2.8.3)
project(tangy_move)

find_package(
	catkin REQUIRED COMPONENTS 
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

add_service_files(
  	DIRECTORY srv
	FILES
	navServerCmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	INCLUDE_DIRS 
	include/tangy_move
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(
	moveMainHelper 
	src/client/navigationClient.cpp 
	src/client/rotate.cpp 
	src/client/pause.cpp 
	src/client/move_straight.cpp 
	src/client/move.cpp 
)
target_link_libraries(
	moveMainHelper 
	${catkin_LIBRARIES}
)
add_dependencies(moveMainHelper tangy_move_gencpp)

add_executable(
	navigationServer 
	src/server/navigationServer.cpp	
	src/server/tangy_move.cpp 
	src/server/rotate.cpp 
	src/server/pause.cpp 
	src/server/move_straight.cpp 
	src/server/move.cpp 
	src/server/helpers.cpp
)
target_link_libraries(
	navigationServer 
	${catkin_LIBRARIES}
)
add_dependencies(navigationServer tangy_move_gencpp)

add_executable(
	moveMain 
	src/client/tangy_move_main.cpp
)
target_link_libraries(
	moveMain 
	moveMainHelper 
	${catkin_LIBRARIES}
)

add_executable(
	controller 
	src/controller/manual_control.cpp
)
target_link_libraries(
	controller 
	moveMainHelper 
	${catkin_LIBRARIES}
)

#add_library(costmap_layer plugins/new_layer.cpp)
#target_link_libraries(costmap_layer ${catkin_LIBRARIES})
#add_dependencies(costmap_layer ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
