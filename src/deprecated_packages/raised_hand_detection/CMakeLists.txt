cmake_minimum_required(VERSION 2.8.3)
project(raised_hand_detection)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)

catkin_package(
   INCLUDE_DIRS 
#  LIBRARIES test_detections
   CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(test_detections
#   src/${PROJECT_NAME}/test_detections.cpp
# )

add_executable(raised_hand src/raised_hand_detection.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(raised_hand raised_hand_detection_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(raised_hand
   ${catkin_LIBRARIES}
)
