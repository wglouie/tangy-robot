cmake_minimum_required(VERSION 2.8.3)
project(trajectory_executor)

find_package(catkin REQUIRED COMPONENTS  
  	 roscpp  
	 drrobot_h20_arm_player
         std_msgs
	 actionlib
	 actionlib_msgs
) 

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)


add_message_files(
   FILES
   arm_trajectory.msg
   both_arm_trajectory.msg
)


add_service_files(
   FILES
   execute_both_arms.srv
)

add_action_files(
   FILES
   execute_file.action
)

generate_messages(
 DEPENDENCIES
   actionlib_msgs   std_msgs   drrobot_h20_arm_player
)


catkin_package(
   INCLUDE_DIRS include/trajectory_executor
)

# ADD ALL CPP's YOU USE
#add_library(library_name src/cpp_name.cpp)
#target_link_libraries(library_name ${catkin_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
#add_dependencies(library_name example_package_gencpp library_you_depend_on)

# ADD ALL CPP's YOU USE
add_executable(trajectory_executor src/example.cpp)
target_link_libraries(trajectory_executor ${catkin_LIBRARIES} ${Boost_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
add_dependencies(trajectory_executor drrobot_h20_arm_player_gencpp trajectory_executor_gencpp kinect_ik_gencpp
)

# Create ActionExecutorClientLibrary
add_library(TrajectoryExecutorClientLibrary src/trajectory_executor_client.cpp)
add_dependencies(TrajectoryExecutorClientLibrary
trajectory_executor_gencpp
drrobot_h20_arm_player_gencpp
kinect_ik_gencpp)
target_link_libraries(TrajectoryExecutorClientLibrary ${catkin_LIBRARIES} ${OpenCV_LIBS})


# ADD ALL CPP's YOU USE
add_executable(trajectory_executor_client_test src/trajectory_executor_client_test.cpp)
target_link_libraries(trajectory_executor_client_test TrajectoryExecutorClientLibrary ${catkin_LIBRARIES} ${Boost_LIBRARIES})
### IF YOU USE A MESSAGE/SERVICE/ACTION OR DEPEND ON A LIBRARY
add_dependencies(trajectory_executor_client_test
trajectory_executor_gencpp
drrobot_h20_arm_player_gencpp
kinect_ik_gencpp)

