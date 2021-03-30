#include <sys/file.h>
#include <string.h>
#include <linux/input.h>
#include <signal.h>
#include <unistd.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <vector>
#include <string.h>
#include <stdio.h>      /* int to string */
#include <stdlib.h>     /* srand, rand */
#include <time.h>
#include "intro_speech_handler.h"
#include "outro_speech_handler.h"
#include "joke_handler.h"
#include "help_text_handler.h"
#include <trivia/TriviaGameAction.h>
#include <trivia/trivia_question_handler.h>
#include <boost/algorithm/string.hpp>
#include "std_msgs/String.h"
#include <math.h>
#include <robot_gui_client.h>
//#include <curses.h>

class triviaGameServer_limited{

private:

  const static int KEY_CODE_RESPONSE=76;
  const static int KEY_CODE_A=30;
  const static int KEY_CODE_B=33;
  const static int KEY_CODE_C=36;
  const static int KEY_CODE_HELP=57;

  ros::NodeHandle nh_;
	actionlib::SimpleActionServer<trivia::TriviaGameAction> as_;
	std::string action_name_;
	std::string activity_code;
	
	std::string jokeFileLocation;
	std::string introFileLocation;
	std::string outroFileLocation;
	std::string helpTextFileLocation;
	std::string indexFilePath;
	std::string keyLog0;
	std::string keyLog1;
	std::string evID0;
	std::string evID1;
	std::vector<std::string> filePaths;

	std::vector<int> teams;
	std::vector<int> scores;
	std::vector< std::vector<float> > locations;
	std::vector<std::string> categories;
	std::vector<std::string> categories_for_paths;
	std::vector< trivia_question_handler > quesHandlers;
	help_text_handler helpHandler;
	intro_speech_handler introHandler;
	outro_speech_handler outroHandler;
	joke_handler jokeHandler;
	RobotGuiClient robotGuiClient;

public:

  bool game_began;
	bool end_game;
	bool pause_game;
	int ques_count;
	int num_teams;
	bool said_question_alrdy;
	
  struct input_event ev0, ev1;
  int dev0, dev1, rd0, rd1;

	void calibrate_keyboards();
	
  void say(std::string speech);
  void say_and_display(std::string s0, std::string s1="", std::string s2="", std::string s3="", std::string s4="");
  void say_question(std::string q, std::string a1, std::string a2, std::string a3, bool repeat=false);
  void say_and_display(std::string activity, std::string str0, std::string str1, int subtab);
	
	//Wait for input from keyboard, and then store input
	int wait_for_response();
	std::string wait_for_answer(int team);
	std::vector<std::string> wait_for_category_votes();
	
	//A trivia question handler chooses and returns questions+answers for one category.
	//A question handler is generated for each different category.
	int query_category(std::string cat1, std::string cat2, std::string cat3);
	
	//Say and display a trivia question from a category
	void give_question(int cat_num);
	void give_hint(vector<string> question);
	
	//Returns a location for a specific team.
	std::vector<float> loc_teams(int team);
	
	//Game flow stuff
	void play_game();
	void give_intro_speech();
	void give_outro_speech();
	void tell_joke();
	
	//General stuff
	triviaGameServer_limited(std::string name);
	~triviaGameServer_limited();
	bool init();
	bool delete_progress();
	void pauseCallback(const std_msgs::String::ConstPtr& str);
	void endGameCallback(const std_msgs::String::ConstPtr& str);
  void executeCB(const trivia::TriviaGameGoal::ConstPtr &goal);

};
