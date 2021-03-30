//============================================================================
// Name        : music_player.h
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description :
//============================================================================

#ifndef music_player_H_
#define music_player_H_
#include <ros/ros.h>
#include <string>
#include <SDL/SDL.h>
#include <SDL/SDL_mixer.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <music_player/music_playerAction.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

class musicServer {
  
  ros::NodeHandle nh_;

  	/*Action library variables*/
	actionlib::SimpleActionServer<music_player::music_playerAction> as_;
	std::string action_name_;
	music_player::music_playerFeedback feedback_;
  music_player::music_playerResult result_;
  public:
    std::string filepath1;
    std::string filepath2;
    std::string filepath3;
    std::string filepath4;
    std::string filepath5;
    std::string filepath6;
    std::string beeppath;
    std::string celebratepath;
    
    bool stop_playing;
    musicServer();
    ~musicServer();
    
    bool init();
    void play_file(std::string curr_num);
    void goalCB();
    void preemptCB();
    void volumeCB(const std_msgs::Int32::ConstPtr& vo);
    void playControlCB(const std_msgs::String::ConstPtr& str);
};
#endif
