//============================================================================
// Name        : audio_player.h
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Helper class for text to speech; play any mp3 audio file by using the function play_file(string filePath)
//============================================================================

#ifndef audio_player_H_
#define audio_player_H_
#include <ros/ros.h>
#include <string>
#include <SDL/SDL.h>
#include <SDL/SDL_mixer.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <music_player/music_playerAction.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

class audioPlayer {
  
  ros::NodeHandle nh_;

  	/*Action library variables*/

  public:
    
    bool stop_playing;
    audioPlayer();
    ~audioPlayer();
    
    bool init();
    void play_file(std::string filePath);
    void volumeCB(const std_msgs::Int32::ConstPtr& vo);
    void playControlCB(const std_msgs::String::ConstPtr& str);
};
#endif