cmake_minimum_required(VERSION 2.8.3)
project(bingo_game)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS drrobot_h20_arm_player help_indicators actionlib_msgs roscpp tf actionlib  message_generation robot_gui rospy face_detection std_msgs roslib tangy_move music_player bingo_detection)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  BingoGame.action
)


generate_messages(
   DEPENDENCIES
   actionlib_msgs std_msgs
)

catkin_package(
   INCLUDE_DIRS include include/BingoGameFullServer
   #LIBRARIES drrobot_h20_arm_player
   CATKIN_DEPENDS actionlib actionlib_msgs roscpp rospy std_msgs message_runtime drrobot_h20_arm_player
   DEPENDS system_lib
)

###########
## Build ##
###########

#find_package(drrobot_h20_arm_player REQUIRED)
#include_directories(${drrobot_h20_arm_player_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

find_package(LibXML2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
set(LIBS ${LIBS} ${LIBXML2_LIBRARIES})

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
add_executable(bingo_game src/BingoGameFullServer/BingoGameFullServer.cpp
	src/BingoGameFullServer/text_to_speech.cpp
	src/BingoGameFullServer/verbal_interactions.cpp
	src/BingoGameFullServer/hand_gestures.cpp
	src/BingoGameFullServer/move.cpp
	src/BingoGameFullServer/head_movement.cpp
	src/BingoGameFullServer/help.cpp
	src/BingoGameFullServer/pause.cpp
	src/BingoGameFullServer/bingo_game.cpp
	src/BingoGameFullServer/face_detection.cpp
	src/BingoGameFullServer/check_card.cpp
	src/BingoGameFullServer/gui.cpp
	src/BingoGameFullServer/debug.cpp
  src/BingoGameFullServer/music.cpp
	##src/bingo_game_server.cpp
	src/joke_handler.cpp
	src/intro_speech_handler.cpp
	src/outro_speech_handler.cpp
	src/small_talk_handler.cpp
	src/bingo_number_handler.cpp
	src/demo_number_handler.cpp
	src/neck_handler.cpp
	src/help_text_handler.cpp)
target_link_libraries(bingo_game FaceDetectionLibrary moveMainHelper music_client_lib
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LIBS}
)
add_dependencies(bingo_game bingo_game_gencpp
          joke_handler_gencpp
          intro_speech_handler_gencpp
          outro_speech_handler_gencpp
          small_talk_handler_gencpp
          bingo_number_handler_gencpp
	  #bingo_search_gencpp
	  bingo_detection_gencpp
          neck_handler_gencpp
          help_text_handler_gencpp
          #drrobot_h20_arm_player_gencpp
					##Add HelpFinder
					)

#add_executable(bingo_game_client src/bingo_game_client.cpp)
#target_link_libraries(bingo_game_client
#   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
#)
#add_dependencies(bingo_game_client bingo_game_gencpp) #drrobot_h20_arm_player_gencpp

add_executable(client src/bingo_game_client.cpp)
target_link_libraries(client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(client bingo_game_gencpp)

add_executable(face_tracker src/face_tracker.cpp src/neck_handler.cpp)
target_link_libraries(face_tracker FaceDetectionLibrary ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(face_tracker bingo_game_gencpp)


