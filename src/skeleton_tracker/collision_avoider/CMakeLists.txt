cmake_minimum_required(VERSION 2.8.3)
project(collision_avoider)

find_package(catkin REQUIRED COMPONENTS  
         roscpp  std_msgs message_generation) 

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)


#add_message_files(
#   FILES
#)


add_service_files(
   FILES
   check_collision.srv
)

#add_action_files(
#   FILES
#   action_name.action
#)

generate_messages(
 DEPENDENCIES std_msgs
)


catkin_package(
   INCLUDE_DIRS include/collision_avoider
)

# ADD ALL CPP's YOU USE
#add_library(library_name src/cpp_name.cpp)
#target_link_libraries(library_name ${catkin_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
#add_dependencies(library_name example_package_gencpp library_you_depend_on)

# ADD ALL CPP's YOU USE
add_executable(collision_avoider src/main.cpp)
target_link_libraries(collision_avoider ${catkin_LIBRARIES} ${Boost_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
add_dependencies(collision_avoider collision_avoider_gencpp)

