//============================================================================
// Name        : music_player_client.h
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
#include <SDL/SDL.h>
#include <SDL/SDL_mixer.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <music_player/music_playerAction.h>
class musicClient{

  public:
    actionlib::SimpleActionClient<music_player::music_playerAction> ac;
    musicClient();
  	ros::NodeHandle nh_;
  	ros::Publisher volume_pub;
  	ros::Publisher music_control_pub;
    
  	void play_music(std::string type);
  	void pause_music();
  	void resume_music();
  	void stop_music();
  	void beep();
  	
  	void change_vol(int vol);

  	void send(std::string str);
  
  private:
  	
  
  	void doneCb(const actionlib::SimpleClientGoalState& state,
  			const music_player::music_playerResultConstPtr& result);
  	void activeCb();
  	void feedbackCb(const music_player::music_playerFeedbackConstPtr& feedback);
	
};

