//Modes of the Bingo Game
#define MOVE
#define MOVE_ARMS
#define TTS
#define MOVE_HEAD
#define NDEMO
//Standard Headers
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <sstream>
#include <vector>
//Input and Output Files
#include <iostream>
#include <fstream>
//ActionLib Headers
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <bingo_game/BingoGameAction.h>
#include <algorithm>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
//All helper clients and handlers
#include <ArmMovementClient.h>
#include "MoveBaseClient.h"
#include <robot_gui_client.h>
#include <FaceDetectionClient.h>
#include "bingo_detection_client.h"
#include "joke_handler.h"
#include "intro_speech_handler.h"
#include "outro_speech_handler.h"
#include "small_talk_handler.h"
#include "bingo_number_handler.h"
#include "demo_number_handler.h"
#include "neck_handler.h"
#include "help_text_handler.h"
#include  <tangy_move.h>
#include <std_msgs/Bool.h>
#include <music_player_client.h>

#include <help_indicators/get_help_indicators.h>
#include <help_indicators/triangle.h>

///////////////////////////////////////////////////////////////////////////////////////////
#include <string.h>
/////////////////////////////////////////////////////////////////////////////////////////////

class BingoGameFullServer {

private:

	ros::NodeHandle nh_;

	// debug pubkisher
	ros::Publisher debug_pub;
	std_msgs::String debug_string;

	//Game State variables
	bool game_began;
	bool end_game;
	bool pause_game;
	int wave_plan_num;
	int point_at_screen_plan_num;
	int celebrate_plan_num;
	int convo_gesture_num;
	int laugh_gesture_num;
	int num_times_helped;
	int num_times_failed;
	const static int LOUD=23;
	const static int QUIET=18;
	
	//For help light stuff
	std_msgs::String stop_msg;
	std_msgs::String go_msg;
	
	//Game state variable
	std_msgs::String gs_msg;

  //All clients, publishers and subscribers
	ros::ServiceClient client;
	ros::Publisher go_pub;
	ros::Publisher clear_pub;
	ros::Publisher game_state_pub;
	ros::Subscriber pause_sub;
	ros::Publisher nod_pub;
	ros::Publisher start_track_face_pub;
	ros::Publisher stop_track_face_pub;

//////////////////////////////////////////////////////////
	ros::Publisher stop_neck_movement;
	ros::Publisher pick_a_person_pub;
	//ros::Publisher choosePOI;
//////////////////////////////////////////////////////////

  //File paths
	std::string resultFileLocation;
	std::string progressFileLocation;
	std::string bingoGameNumbersFileLocation;
	std::string jokesFileLocation;
	std::string smallTalkFileLocation;
	std::string introFileLocation;
	std::string outroFileLocation;
	std::string helpTextFileLocation;
	std::string musicFolderLocation;
	std::string demoprogressFileLocation;
	std::string demoIntroLocation;
	std::string deomHelloLocation;

	// create the bingo action server
	actionlib::SimpleActionServer<bingo_game::BingoGameAction> as_;
	std::string action_name_;

	// declaring result variables to get the results from each server
	bingo_game::BingoGameFeedback feedback_;
	bingo_game::BingoGameResult result_;

	//Declare all helper handlers and clients
	RobotGuiClient robotGuiClient;
	BingoDetectionClient bingoDetectionClient;
	MoveBaseClient moveBaseClient;
	FaceDetectionClient faceDetectionClient;
	ArmMovementClient armMovementClient;
	joke_handler jokeHandler;
	intro_speech_handler introHandler;
	intro_speech_handler introDemoHandler;
	intro_speech_handler demoHelloHandler;
	outro_speech_handler outroHandler;
	small_talk_handler smallTalkHandler;
	bingo_number_handler numberHandler;
	demo_number_handler demoNumberHandler;
	neck_handler neckHandler;
	help_text_handler helpHandler;
	navigationClient navClient;
  musicClient musicPlayer;
	
public:

	BingoGameFullServer(std::string name);
	~BingoGameFullServer(void);
	void debug_print(std::string string);
	void state_pub(std::string state);
	bool delete_progress();
  bool init();
  //Head/Neck
	void reset_head_pos();
	void rand_head_pos();
	void look_down();
	void ready_to_nod();
	void not_ready_to_nod();
  void look_at_card_1();            //Three different positions for looking at the card
  void look_at_card_2();
  void look_at_card_3();
  void look_back_to_person();       //after looking at a card

	//Pause
	void pauseCallback(const std_msgs::String::ConstPtr& str);
	void pause_everything();
	
	//Face detection
	std::string detect_face();
	void look_at_face();
	void stop_look_at_face();
	void changePOI();
///////////////////////////////////////////////////////////////////////////////////////////
	void stopNeckMovement(bool stopneckmove);
///////////////////////////////////////////////////////////////////////////////////////////
	
	//Bingo card detection
	void check_bingo_card();

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
	
	//Arms
	void plan_wave();
	void plan_point_at_screen();
	void plan_celebrate();
	void plan_point_at();
	void plan_convo_gesture();
	void plan_laugh_gesture();
	void wave();
	void point_at_screen();
	void celebrate();
	void point_at();
	void convo_gesture();
	void laugh_gesture();
	void armServerHandler(std::string goal);
	
	//Talking
	void say(std::string str);
	void display_text(std::string str);
	void say_and_display(std::string str);
	int get_speech_delay(std::string str);
	void say_and_display(std::string str1, std::string str2);
	void gui_handler(robot_gui::Robot_guiGoal interface_goal, int delay);
	void give_intro_speech();
    void give_demo_intro_speech();
	void give_outro_speech();
	void tell_joke();
	void make_small_talk();
	
	//Music
	void stop_music();
	void pause_music();
	void play_music(std::string type);
	void resume_music();
	void set_music_volume(int vol);
	
	//Help
	void begin_looking_for_help();
	void stop_looking_for_help();
	std::vector<help_indicators::triangle> get_help_requests();
	void clear_request(std::string name);
	bool determine_help();
	void do_help();
    void demo_help(std::string name, float p_x, float p_y);
	void help(std::string name, float p_x, float p_y);
	void reset_num_times_helped();
	void reset_num_times_failed();
	
	//General Bingo stuff
	void play_Bingo();
    void play_Bingo_Demo();
	void end_bingo_game();
	void executeCB(const bingo_game::BingoGameGoal::ConstPtr &goal);
};
