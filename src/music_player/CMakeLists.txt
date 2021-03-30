cmake_minimum_required(VERSION 2.8.3)
project(music_player)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS actionlib_msgs roscpp actionlib  message_generation rospy std_msgs roslib)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(SDL REQUIRED)
find_package(SDL_mixer REQUIRED)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES music_player.action
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs std_msgs
)

catkin_package(
   INCLUDE_DIRS include

   CATKIN_DEPENDS actionlib actionlib_msgs roscpp rospy std_msgs
   DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${sdl_INCLUDE_DIR}
  ${SDL_mixer_INCLUDE_DIRS}
  #/bingo_game/bingo_game_client.hpp
  # ${PROJECT_SOURCE_DIR}/drrobot_h20_arm_player
)

## Declare a cpp executable
add_executable(music_player src/music_player.cpp)
target_link_libraries(music_player ${catkin_LIBRARIES} ${SDL_LIBRARY} ${SDLMIXER_LIBRARY})
add_dependencies(music_player music_player_gencpp)

add_executable(music_test src/music_player_client.cpp)
target_link_libraries(music_test ${catkin_LIBRARIES} ${SDL_LIBRARY} ${SDLMIXER_LIBRARY})
add_dependencies(music_test music_player_gencpp)

add_library(music_client_lib src/music_player_client.cpp)
target_link_libraries(music_client_lib ${catkin_LIBRARIES} ${SDL_LIBRARY} ${SDLMIXER_LIBRARY})
add_dependencies(music_client_lib music_player_gencpp)

add_executable(audio_player src/audio_player.cpp)
target_link_libraries(audio_player ${catkin_LIBRARIES} ${SDL_LIBRARY} ${SDLMIXER_LIBRARY})
add_dependencies(audio_player music_player_gencpp)

add_library(audio_player_lib src/audio_player.cpp)
target_link_libraries(audio_player_lib ${catkin_LIBRARIES} ${SDL_LIBRARY} ${SDLMIXER_LIBRARY})
add_dependencies(audio_player_lib music_player_gencpp)

add_executable(audio_player_test src/audio_player_client_test.cpp )
target_link_libraries(audio_player_test audio_player_lib ${catkin_LIBRARIES} ${SDL_LIBRARY} ${SDLMIXER_LIBRARY})
add_dependencies(audio_player_test music_player_gencpp)

