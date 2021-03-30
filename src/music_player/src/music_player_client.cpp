//============================================================================
// Name        : music_player_client.cpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description :Use the functions play_music(), pause_music(), stop_music(), change_vol(int vol)
//============================================================================

#include "music_player_client.h"

musicClient::musicClient():ac("music_player",true){
  volume_pub= nh_.advertise<std_msgs::Int32>("music_volume", 1000);
  music_control_pub = nh_.advertise<std_msgs::String>("music_controller", 1000);
}

void musicClient::play_music(std::string type){
  // if(type.compare("normal")==0){
  //     send("normal");
  // }else if(type.compare("celebrate")==0){
  //     send("celebrate");
  // }
  send(type);

}
void musicClient::beep(){
  send("beep");

}
void musicClient::pause_music(){
  std_msgs::String msg;
  msg.data="pause";
  music_control_pub.publish(msg);
}
void musicClient::resume_music()
{
  std_msgs::String msg;
  msg.data="resume";
  music_control_pub.publish(msg);
}
void musicClient::stop_music(){
  std_msgs::String msg;
  msg.data="stop";
  music_control_pub.publish(msg);
  ac.cancelAllGoals();
}

void musicClient::change_vol(int vol){
  std_msgs::Int32 msg;
  msg.data=vol;
  volume_pub.publish(msg);
}

void musicClient::send(std::string str){
  music_player::music_playerGoal newGoal;
  newGoal.volume=-1;
  newGoal.start=str;
  ac.sendGoal(newGoal, boost::bind(&musicClient::doneCb, this, _1, _2),
			boost::bind(&musicClient::activeCb, this),
			boost::bind(&musicClient::feedbackCb, this, _1));
	ROS_INFO("Sending music goal.");
}

void musicClient::doneCb(const actionlib::SimpleClientGoalState& state,
		const music_player::music_playerResultConstPtr& result){
		  
		}
void musicClient::activeCb(){
  
}
void musicClient::feedbackCb(const music_player::music_playerFeedbackConstPtr& feedback){
  
}


int main(int argc, char **argv){
	ros::init(argc,argv,"music_test");
	ros::AsyncSpinner spinner(0);
	spinner.start();
	musicClient client;
	
	ros::waitForShutdown();
 	return 0;

}
