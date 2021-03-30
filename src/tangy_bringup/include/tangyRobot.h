//============================================================================
// Name        : tangyRobot.h
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description :A class to represent Tangy as a set of action clients
//              for use with its output modules
//============================================================================


#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>

#include <ArmMovementClient.h>
#include "MoveBaseClient.h"
#include <robot_gui_client.h>
#include <FaceDetectionClient.h>
#include <music_player_client.h>
#include "neck_handler.h"
#include <ros/ros.h>
#include <tangy_move.h>

#define single_text_tab = 0;
#define multi_2_text_tab = 1;
#define multi_4_text_tab = 2;

class tangyRobot {

	
private:

	ros::NodeHandle nh_;
  
  bool pause;
	int wave_plan_num;
	int point_at_screen_plan_num;
	int celebrate_plan_num;
	int convo_gesture_num;
	int laugh_gesture_num;
	int clap_gesture_num;

  //All clients, publishers and subscribers
	ros::Subscriber pause_sub;
	ros::Publisher start_track_face_pub;
	ros::Publisher stop_track_face_pub;
	ros::Publisher nod_pub;
	ros::Publisher stop_neck_movement;
	ros::Publisher pick_a_person_pub;

	//Declare all helper handlers and clients
	RobotGuiClient robotGuiClient;
	MoveBaseClient moveBaseClient;
	FaceDetectionClient faceDetectionClient;
	ArmMovementClient armMovementClient;
	neck_handler neckHandler;
	navigationClient navClient;
	musicClient musicPlayer;
	
public:
	const static int LOUD=23;
	const static int QUIET=18;
	
    tangyRobot(std::string name = "");
    tangyRobot(float wait_time, std::string name = "");
	~tangyRobot();

	bool init();
	//Head/Neck
	void reset_head_pos();
	void rand_head_pos();
	void look_down();
    void look_at_card_1();
    void look_at_card_2();
    void look_at_card_3();

	void look_back_to_person();
	void ready_to_nod();
	void not_ready_to_nod();
	void nod();
	void shake();
	void moveNeck(int horizAng, int vertAng);

	//Face detection
	std::string detect_face();
	void look_at_face();
	void stop_look_at_face();
	void changePOI();
	void stopNeckMovement(bool stopneckmove);

	//Navigation
	void move(std::string frame, float x, float y);
	void rotate_no_wait(std::string frame, float ang);
	void rotate(std::string frame, float ang);
	void move_straight(float distance);
	void move_straight_no_rotate(float distance);
	void face(float x, float y);
	void move_to_help(float x, float y);
	void move_to_orig_pos();
	void rotate_manual(double ang);
	void rotate_to_face();
	void pauseCallback(const std_msgs::String::ConstPtr& str);
	
	//Arms
	void plan_wave();
	void plan_point_at_screen();
	void plan_celebrate();
	void plan_point_at();
	void plan_convo_gesture();
	void plan_laugh_gesture();
	void plan_clap();
	void wave();
	void point_at_screen();
	void celebrate();
	void point_at();
	void convo_gesture();
	void laugh_gesture();
	void clap();
	void armServerHandler(std::string goal);
	
	//Talking
	void say(std::string activity, std::string str, int subtab=0);
	void display_text(std::string activity, std::string str, int subtab=0);
	void say_and_display(std::string activity, std::string str, int subtab=0);
	int get_speech_delay(std::string str);
	void say_and_display(std::string activity, std::string str0, std::string str1, int subtab);
	void say_and_display(std::string activity, std::string str0, std::string str1="", std::string str2="", std::string str3="", std::string str4="", int subtab=0);
	void gui_handler(robot_gui::Robot_guiGoal interface_goal, int delay);

	//Music
	void stop_music();
	void pause_music();
	void play_music(std::string type);
	void resume_music();
	void set_music_volume(int vol);
	void beep();
	
};
