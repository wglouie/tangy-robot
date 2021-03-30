cmake_minimum_required(VERSION 2.8.3)
project(help_indicators)

find_package (
	catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	tf
	cv_bridge
	image_transport
	actionlib
	pcl_ros 
	tangy_move
        drrobot_h20_player
)

add_message_files (
	DIRECTORY msg
	FILES
	triangle.msg
	transformRequest.msg
)

add_service_files (
	DIRECTORY srv
	FILES
	get_help_indicators.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	actionlib_msgs
)

catkin_package(
	INCLUDE_DIRS include include/help_indicators
	CATKIN_DEPENDS actionlib_msgs
)

include_directories( 
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

#   Help Indicators Server
add_executable (
	help_indicators 
	src/main.cpp 
	src/help_indicators.cpp 
	src/update_new_help_indicators.cpp 
	src/update_help_indicators.cpp 
	src/helpers.cpp 
	src/triangle_detection.cpp 
	src/transform_help_indicator.cpp
	src/squareFinder.cpp
	src/acknowledge_help.cpp
)

add_dependencies(
	help_indicators 
	help_indicators_gencpp
        drrobot_h20_player_gencpp
)

target_link_libraries (
	help_indicators 
	${catkin_LIBRARIES}
	${Boost_LIBS}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

#   Help Indicators Client
add_executable (
        test_help_indicators
        src/test_help_indicators.cpp
)

add_dependencies(
        test_help_indicators
	help_indicators_gencpp
)

target_link_libraries (
        test_help_indicators
	${catkin_LIBRARIES}
	${Boost_LIBS}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

#   Kinect Calibration
add_executable (
	kinect_calibration
	src/kinect_calibration.cpp 
)

target_link_libraries (
	kinect_calibration
	moveMainHelper
)


add_dependencies(
	kinect_calibration
	help_indicators_gencpp
)

target_link_libraries (
	kinect_calibration
	${catkin_LIBRARIES}
	${Boost_LIBS}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

#   Help Transformers
add_executable (
	help_transformers 
	src/help_transformers.cpp 
	src/help_indicators.cpp 
	src/update_new_help_indicators.cpp 
	src/update_help_indicators.cpp 
	src/helpers.cpp 
	src/triangle_detection.cpp 
	src/transform_help_indicator.cpp
	src/squareFinder.cpp
	src/acknowledge_help.cpp
)
target_link_libraries (
	help_transformers
	moveMainHelper
)


add_dependencies(
	help_transformers
	help_indicators_gencpp
)

target_link_libraries (
	help_transformers
	${catkin_LIBRARIES}
	${Boost_LIBS}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

add_library(HelpIndicatorsLibrary src/help_indicators_client.cpp)
add_dependencies(HelpIndicatorsLibrary help_indicators_gencpp)
target_link_libraries(HelpIndicatorsLibrary ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable (
	ir_rgb_streamer 
	src/ir_rgb_stream.cpp 
)

add_dependencies(
	ir_rgb_streamer 
	help_indicators_gencpp
)

target_link_libraries (
        ir_rgb_streamer
	${catkin_LIBRARIES}
	${Boost_LIBS}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

