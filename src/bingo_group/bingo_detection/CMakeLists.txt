cmake_minimum_required(VERSION 2.8.3)
project(bingo_detection)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs image_transport cv_bridge actionlib text_to_speech)

find_package(gscam)

find_package(OpenCV 2.4.9 EXACT REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
 

#Check that the correct OpenCV version is being compiled
#if(OpenCV_FOUND)
#message(----OPENCV ${OpenCV_VERSION}----)
#message(----${OpenCV_INCLUDE_DIRS}----)
#message(----${OpenCV_LIBS}----)
#endif(OpenCV_FOUND)


#Link opencv libraries
#link_directories(${OpenCV_LIBRARY_DIRS})

## Generate actions in the action folder
add_action_files(
  DIRECTORY action
  FILES
  BingoDetection.action
)

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS actionlib_msgs
)

add_definitions(-std=c++11)
#-std=c++11
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(bingo_detection_server src/mainBingo.cpp
			    	      src/surfdetection.cpp
				      src/squarefinder.cpp
				      src/redmarkers.cpp
				      src/processedimage.cpp
                                      src/bingocard.cpp
				      src/bingodetectionserver.cpp
				      src/debugopencv.cpp)
add_dependencies(bingo_detection_server bingo_detection_gencpp)
target_link_libraries(bingo_detection_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_calib3d.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_contrib.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_core.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_features2d.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_flann.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_gpu.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_highgui.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_imgproc.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_legacy.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_ml.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_nonfree.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_objdetect.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_photo.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_stitching.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_video.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_videostab.so 
${Boost_LIBS})

add_executable(BingoDetectionClient src/BingoDetectionClient.cpp)
add_dependencies(BingoDetectionClient bingo_detection_gencpp)
target_link_libraries(BingoDetectionClient ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_calib3d.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_contrib.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_core.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_features2d.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_flann.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_gpu.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_highgui.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_imgproc.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_legacy.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_ml.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_nonfree.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_objdetect.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_photo.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_stitching.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_video.so
#${CMAKE_CURRENT_SOURCE_DIR}/lib/libopencv_videostab.so 
${Boost_LIBS})

add_executable(bingo_detection_client src/client_test.cpp)
add_dependencies(bingo_detection_client bingo_detection_gencpp)
target_link_libraries(bingo_detection_client ${catkin_LIBRARIES})

add_executable(bingo_subscriber src/mainSubscribeToBingoNode.cpp
				src/subscribernode.cpp)
add_dependencies(bingo_subscriber bingo_detection_gencpp)
target_link_libraries(bingo_subscriber ${catkin_LIBRARIES})
