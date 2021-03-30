cmake_minimum_required(VERSION 2.8.3)
project(example_package)

find_package(catkin REQUIRED COMPONENTS  
   roscpp  
) 

catkin_package(
   INCLUDE_DIRS include/example_package
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)


#add_message_files(
#   FILES
#   message_name.msg
#)


#add_service_files(
#   FILES
#   service_name.srv
#)

#add_action_files(
#   FILES
#   action_name.action
#)

# ADD ALL CPP's YOU USE
#add_library(library_name src/cpp_name.cpp)
#target_link_libraries(library_name ${catkin_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
#add_dependencies(library_name example_package_gencpp library_you_depend_on)

# ADD ALL CPP's YOU USE
#add_executable(executable_name src/example.cpp)
#target_link_libraries(executable_name ${catkin_LIBRARIES} ${Boost_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
#add_dependencies(executable_name example_package_gencpp library_you_depend_on)

