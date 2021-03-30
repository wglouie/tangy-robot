cmake_minimum_required(VERSION 2.8.3)
project(head_arm_control_gui)
find_package(catkin REQUIRED
  COMPONENTS 
    actionlib 
    roscpp 
    move_base_msgs
    message_generation
    drrobot_h20_player
    drrobot_h20_arm_player
) 

#### QT
set(QT_QMAKE_EXECUTABLE "/usr/bin/qmake-qt4")
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtXML)
include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

# include this for ui_h
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#### /QT

add_message_files(
  FILES
  Slider.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(qttest src/main.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(qttest drrobot_h20_player_gencpp drrobot_h20_arm_player_gencpp)
target_link_libraries(qttest ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})
