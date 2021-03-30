##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(robot_gui)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp actionlib  actionlib_msgs text_to_speech  drrobot_h20_player)
include_directories(include ${catkin_INCLUDE_DIRS})

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
#find_package(catkin REQUIRED COMPONENTS qt_build roscpp actionlib actionlib_msgs message_generation)
#include_directories(${catkin_INCLUDE_DIRS})



find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_action_files(
	DIRECTORY action
	FILES 	Telepresence.action
		Robot_gui.action
)

generate_messages(
	DEPENDENCIES actionlib_msgs std_msgs
)

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.
catkin_package(
  INCLUDE_DIRS include include/${PROJECT_NAME}
)
#catkin_package(CATKIN_DEPENDS actionlib_msgs)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically
# included via the dependency call in manifest.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/robot_gui/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

#add_executable(gui_server src/robot_gui_server.cpp)
#add_dependencies(gui_server robot_gui_gencpp)
#target_link_libraries(gui_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(robot_gui src/qnode.cpp
#			 src/main.cpp
#			 src/main_window.cpp
#			 src/telepresence_server.cpp
#			 src/navigation_client.cpp
#			 ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP}) ##${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

add_executable(robot_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(robot_gui robot_gui_gencpp drrobot_h20_player_gencpp)
target_link_libraries(robot_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS robot_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})





