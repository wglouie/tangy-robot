cmake_minimum_required(VERSION 2.8.3)
project(coblya_test_package)

find_package(catkin REQUIRED COMPONENTS  
         roscpp  std_msgs message_generation nlopt)


include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)


#add_message_files(
#   FILES
#)


add_service_files(
   FILES
   optimize_angles.srv
)

#add_action_files(
#   FILES
#   action_name.action
#)

generate_messages(
 DEPENDENCIES std_msgs
)


catkin_package(
   INCLUDE_DIRS include/coblya_test_package
)

# ADD ALL CPP's YOU USE
#add_library(library_name src/cpp_name.cpp)
#target_link_libraries(library_name ${catkin_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
#add_dependencies(library_name example_package_gencpp library_you_depend_on)

# ADD ALL CPP's YOU USE
add_executable(coblya_test src/main.cpp)
target_link_libraries(coblya_test nlopt ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
add_dependencies(coblya_test coblya_test_package_gencpp)

