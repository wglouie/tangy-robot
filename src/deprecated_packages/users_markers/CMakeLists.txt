cmake_minimum_required(VERSION 2.8.3)
project(users_markers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS 
#  LIBRARIES test_detections
   CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(test_detections
#   src/${PROJECT_NAME}/test_detections.cpp
# )

## Declare a cpp executable
add_executable(users_markers src/users_markers.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(users_markers users_markers_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(users_markers
   ${catkin_LIBRARIES}
)

