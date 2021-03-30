cmake_minimum_required(VERSION 2.8.3)
project(fault_diagnosis)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  message_generation
  keyboard
  image_transport
#  cv_bridge
  text_to_speech
)
###ADD BOOST IF NEEDED
find_package(Boost REQUIRED COMPONENTS thread)
#find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)

add_message_files(DIRECTORY msg
  FILES Fault.msg AudioData.msg
)

add_service_files(DIRECTORY srv
  FILES fault_return.srv print_faults.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   INCLUDE_DIRS include 
   CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs roscpp diagnostic_msgs keyboard text_to_speech
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS} )#${OpenCV_INCLUDE_DIRS})

add_definitions(-std=c++11) ##http://answers.ros.org/question/216842/ros-using-c-11-how-to-use-with-catkin/

add_executable(fault_diagnosis_core src/fault_diagnosis_core.cpp)

#add_dependencies(fault_diagnosis_core fault_diagnosis_generate_messages_cpp)
add_dependencies(fault_diagnosis_core fault_diagnosis_gencpp)
target_link_libraries(fault_diagnosis_core ${catkin_LIBRARIES})


add_executable(fault_monitor_rangefinder src/fault_monitor_rangefinder.cpp)

add_dependencies(fault_monitor_rangefinder fault_diagnosis_gencpp)
target_link_libraries(fault_monitor_rangefinder ${catkin_LIBRARIES})

add_executable(fault_monitor_axiscamera src/fault_monitor_axiscamera.cpp)

add_dependencies(fault_monitor_axiscamera fault_diagnosis_gencpp)
target_link_libraries(fault_monitor_axiscamera ${catkin_LIBRARIES})

add_executable(test_monitor_keyboard src/test_monitor_keyboard.cpp)

add_dependencies(test_monitor_keyboard fault_diagnosis_gencpp)
target_link_libraries(test_monitor_keyboard ${catkin_LIBRARIES})

add_executable(fault_monitor_bingo_environment src/fault_monitor__bingo_environment.cpp)

add_dependencies(fault_monitor_bingo_environment fault_diagnosis_gencpp)
target_link_libraries(fault_monitor_bingo_environment ${catkin_LIBRARIES})

add_executable(fault_diagnosis_print_requester src/fault_diagnosis_print_requester.cpp)

add_dependencies(fault_diagnosis_print_requester fault_diagnosis_gencpp)
target_link_libraries(fault_diagnosis_print_requester ${catkin_LIBRARIES})

add_executable(fault_monitor_navigation src/fault_monitor_navigation.cpp)

add_dependencies(fault_monitor_navigation fault_diagnosis_gencpp)
target_link_libraries(fault_monitor_navigation ${catkin_LIBRARIES})

add_executable(fault_monitor_disconnect src/fault_monitor_disconnect.cpp)

add_dependencies(fault_monitor_disconnect fault_diagnosis_gencpp)
target_link_libraries(fault_monitor_disconnect ${catkin_LIBRARIES})

add_executable(fault_monitor_kinect src/fault_monitor_kinect.cpp)

add_dependencies(fault_monitor_kinect fault_diagnosis_gencpp)
target_link_libraries(fault_monitor_kinect ${catkin_LIBRARIES})

add_executable(fault_monitor_webcam src/fault_monitor_webcam.cpp)

add_dependencies(fault_monitor_webcam fault_diagnosis_gencpp)
target_link_libraries(fault_monitor_webcam ${catkin_LIBRARIES})

#add_executable(image_saver src/image_saver.cpp)

#add_dependencies(image_saver fault_diagnosis_gencpp)
#target_link_libraries(image_saver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(test_monitor src/test_monitor.cpp)

#add_dependencies(test_monitor fault_diagnosis_gencpp)
#target_link_libraries(test_monitor ${catkin_LIBRARIES})

add_executable(audio_saver src/audio_saver.cpp)

add_dependencies(audio_saver fault_diagnosis_gencpp)
target_link_libraries(audio_saver ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})
