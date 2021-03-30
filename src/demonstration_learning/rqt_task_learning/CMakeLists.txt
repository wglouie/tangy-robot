cmake_minimum_required(VERSION 2.8.3)
project(rqt_task_learning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  std_msgs
  image_transport
  sensor_msgs
  cv_bridge
  action_executor
  world_state_identifier
  activity_learner
)

catkin_package(
        INCLUDE_DIRS ${rqt_task_learning_INCLUDE_DIRECTORIES}
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS rqt_gui rqt_gui_cpp std_msgs image_transport sensor_msgs cv_bridge action_executor world_state_identifier activity_learner roslib
)

catkin_python_setup()

add_definitions(-std=c++11)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rqt_task_learning_SRCS
  src/rqt_task_learning/task_learning.cpp
  src/rqt_task_learning/wait_dialog.cpp
  src/rqt_task_learning/ratio_layouted_frame.cpp
  src/rqt_task_learning/gesture_widget.cpp
  src/rqt_task_learning/gesture_speech_executor.cpp
  src/rqt_task_learning/skeleton_watcher.cpp
  src/rqt_task_learning/wireless_device_receiver.cpp
  src/rqt_task_learning/watch_action_executor_thread.cpp
  src/rqt_task_learning/watch_world_state_thread.cpp
  src/rqt_task_learning/learner.cpp
  src/rqt_task_learning/main_menu_widget.cpp
)

set(rqt_task_learning_HDRS
  include/rqt_task_learning/task_learning.h
  include/rqt_task_learning/ratio_layouted_frame.h
)

set(rqt_task_learning_UIS
  src/rqt_task_learning/world_state_identifier.ui
  src/rqt_task_learning/policy_gui.ui
  src/rqt_task_learning/instructions.ui
  src/rqt_task_learning/start_menu.ui
  src/rqt_task_learning/new_gui.ui
  src/rqt_task_learning/gesture.ui
)

set(rqt_task_learning_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)



file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
qt4_wrap_cpp(rqt_task_learning_MOCS ${rqt_task_learning_HDRS})
qt4_wrap_ui(rqt_task_learning_UIS_H ${rqt_task_learning_UIS})
QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})


include_directories(${rqt_task_learning_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_task_learning_SRCS} ${rqt_task_learning_MOCS} ${rqt_task_learning_UIS_H} ${QT_RESOURCES_CPP})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY}  ActionExecutorClientLibrary WorldStateIdentifierClientLibrary)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(PROGRAMS scripts/rqt_task_learning
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
