cmake_minimum_required(VERSION 2.8.3)
project(test_package)

find_package(catkin REQUIRED COMPONENTS  
   roscpp  
) 

catkin_package(
   INCLUDE_DIRS include/test_package
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

add_executable(node1 src/node1.cpp src/nodeClass.cpp)
target_link_libraries(node1 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(node2 src/node2.cpp src/nodeClass.cpp)
target_link_libraries(node2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(node3 src/node3.cpp src/nodeClass.cpp)
target_link_libraries(node3 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
