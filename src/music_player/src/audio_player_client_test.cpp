//============================================================================
// Name        : music_player_client.cpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description :Use the functions play_music(), pause_music(), stop_music(), change_vol(int vol)
//============================================================================
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <ros/ros.h>
#include <string>
#include "audio_player.h"
#include <iostream>

int main(int argc, char **argv){
	ros::init(argc,argv,"audio_client_test");
	ros::NodeHandle nh_;
  // ros::Publisher volume_pub= nh_.advertise<std_msgs::Int32>("audio_volume", 1000);
  // ros::Publisher audio_control_pub = nh_.advertise<std_msgs::String>("audio_controller", 1000);
	ros::AsyncSpinner spinner(0);
	spinner.start();
  audioPlayer player;
  player.play_file("/home/tangy/Desktop/mp3/13236468414691287632.mp3");
  // player.play_file("/home/tangy/tangy-robot/src/bingo_group/music_player/music/file1.mp3");
//   player.play_file("/home/tangy/tangy-robot/src/bingo_group/music_player/music/file2.mp3");
// 	player.play_file("/home/tangy/tangy-robot/src/bingo_group/music_player/music/file3.mp3");
// 	player.play_file("/home/tangy/tangy-robot/src/bingo_group/music_player/music/file4.mp3");
// 	player.play_file("/home/tangy/tangy-robot/src/bingo_group/music_player/music/file5.mp3");
// 	player.play_file("/home/tangy/tangy-robot/src/bingo_group/music_player/music/file6.mp3");
	
	ros::waitForShutdown();
 	return 0;

}