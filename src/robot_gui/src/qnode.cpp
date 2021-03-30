/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/robot_gui/main_window.hpp"
#include "../include/robot_gui/qnode.hpp"
#include <drrobot_h20_player/PowerInfo.h>
//#include <Python.h>
#include <text_to_speech/tts.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


#define default_tab 	 0
#define robotStatus_tab  1
#define trivia_tab 2
#define telepresence_tab 3
#define bingo_tab	 4
#define log_tab 	 5

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{
		//aS = new RobotGuiServer("robot_gui_server");
		onCall = false;
		callRejected = false;
		sessionCompleted = false;
		tabIndex = 0;
		waiting = true;
		showingDefaultTab = false;
		showingTelepresence = false;
		showingBingo = false;
	}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}
//initialization
bool QNode::init() {
	ros::init(init_argc,init_argv,"robot_gui");
	if ( ! ros::master::check() )
	{
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	//chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	aS = new RobotGuiServer("robot_gui_server");

	chatter_publisher = n.advertise<std_msgs::String>("telepresence_chatter", 1000);
	//chatter_subscriber = n.subscribe("telepresence_chatter", 1000, chatterCallback);
	powerInfo_pub = n.advertise<drrobot_h20_player::PowerInfo>("drrobot_powerinfo", 1);
	powerInfo_sub = n.subscribe("drrobot_powerinfo",1, &QNode::powerInfoReceived, this);
    tts_client = n.serviceClient<text_to_speech::tts>("tts");

    bat1_vol_ = 0.0;
    bat2_vol_ = 0.0;
	bat1_temp_ = 0.0;
	bat2_temp_ = 0.0;
	dcin_vol_ = 0.0;
	ref_vol_ = 0.0;
	//unsigned short int power_status_;
	//unsigned short int power_path_;
	//unsigned short int charge_path_;

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"robot_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	//chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

	aS = new RobotGuiServer("robot_gui_server");

	chatter_publisher = n.advertise<std_msgs::String>("telepresence_chatter", 1000);
	//chatter_subscriber = n.subscribe("telepresence_chatter", 1000, chatterCallback);
	powerInfo_pub = n.advertise<drrobot_h20_player::PowerInfo>("drrobot_powerinfo", 1);
    powerInfo_sub = n.subscribe("drrobot_powerinfo",1, &QNode::powerInfoReceived, this);

	bat1_vol_ = 0.0;
	bat2_vol_ = 0.0;
	bat1_temp_ = 0.0;
	bat2_temp_ = 0.0;
	dcin_vol_ = 0.0;
	ref_vol_ = 0.0;
	//unsigned short int power_status_;
	//unsigned short int power_path_;
	//unsigned short int charge_path_;

	start();
	return true;
}

void QNode::powerInfoReceived(const drrobot_h20_player::PowerInfo::ConstPtr& powerInfo){
	bat1_vol_ = powerInfo->bat1_vol;
	bat2_vol_ = powerInfo->bat2_vol;
	bat1_temp_ = powerInfo->bat1_temp;
	bat2_temp_ = powerInfo->bat2_temp;
	dcin_vol_ = powerInfo->dcin_vol;
	ref_vol_ = powerInfo->ref_vol;
	//unsigned short int power_status_;
	//unsigned short int power_path_;
	//unsigned short int charge_path_;

	/*
	ROS_INFO("bat1_vol: [%.2f]", powerInfo->bat1_vol);
	ROS_INFO("bat2_vol: [%.2f]", powerInfo->bat2_vol);
	ROS_INFO("bat1_temp: [%.2f]", powerInfo->bat1_temp);
	ROS_INFO("bat2_temp: [%.2f]", powerInfo->bat2_temp);
	ROS_INFO("dcin_vol: [%.2f]", powerInfo->dcin_vol);
	ROS_INFO("ref_vol: [%.2f]", powerInfo->ref_vol);
	*/
	emit updateBatteryInfo();
}

void QNode::askToPlayBingo(){
    ttsSpeak("Hi There! Would you like to play Bingo With me?", "en", false, true);
}
void QNode::setGUIText(){
	command_speech_s = "";
	command_speech_ss.str("");
	ROS_INFO("Aciton server code: [%d]", aS->code_);
	switch(aS->code_){
		case(0):
			ROS_INFO("Show the string in the tab selected");
			auxstr = QString::fromStdString(aS->text_);
			formatedstring = auxstr;//"<font color=\"white\" size=\"100\">"+ auxstr+ "</font>";
			subtab = aS->subtab_;
			speak = false;
			showText = true;
			break;
		case(1):
			ROS_INFO("Speak the text in the variable speech");
			command_speech_ss << "echo " << aS->speech_ << " | festival --tts";
			subtab = aS->subtab_;
			command_speech_s = command_speech_ss.str();
			speak = true;
			showText = false;
			//system(command_speech_s.c_str());
			break;
		case(2):
			ROS_INFO("Show the string in the variable text and speak the text in the variable speech");
			//command_speech_ss << "echo " << aS->speech_ << " | festival --tts";
			//command_speech_s = command_speech_ss.str();
			auxstr = QString::fromStdString(aS->text_);
			formatedstring = auxstr;//"<font color=\"white\" size=\"100\">"+ auxstr+ "</font>";
			subtab = aS->subtab_;
			speak = true;
			showText = true;
			break;
        default:
			ROS_INFO("Nothing happens");
			speak = false;
			showText = false;
            emit
			break;
	}

}

void QNode::run() {
  ros::Rate loop_rate(30);
	int count = 0;
	bool goal_finished = true;
	//std_msgs::String msg;
	//std::stringstream ss;
	//RobotGuiServer aS("robot_gui_server");

	while ( ros::ok() ) {
		//std_msgs::String msg;
		////std::stringstream ss;
		//ss << "hello world " << count;//
		//msg.data = ss.str();//
		//chatter_publisher.publish(msg);
		//log(Info,std::string("I sent: ")+msg.data);

		///*
		if(aS->newGoal){
			goal_finished = false;
			aS->newGoal = false;

			ROS_INFO("Received a new goal: %s", aS->activity_.c_str());

			setGUIText();

			//show the default tab
			if(aS->activity_ == "default_tab"){
				
				log(Info,std::string("[")+aS->action_name_.c_str()+std::string("]: Show default tab"));
				emit changeTab(default_tab);
				showingDefaultTab = true;
				//actionServer.result_.end = 2;
				//actionServer.telepresenceAS_.setSucceeded(actionServer.result_);
				//continue;
			}
			//show the robot status tab
			else if(aS->activity_ == "robot_status"){
				log(Info,std::string("[")+aS->action_name_.c_str()+std::string("]: Show robot status tab"));
				emit changeTab(robotStatus_tab);
			}
			//show the telepresence tab
			else if(aS->activity_ == "telepresence"){
				log(Info,std::string("[")+aS->action_name_.c_str()+std::string("]: Show telepresence tab"));
				showingTelepresence = true;
				skypeUser.str("");
				skypeUser << aS->text_;
				nameOfCaller.str("");
				nameOfCaller << aS->speech_;
				emit changeTab(telepresence_tab);
			}
			//show the bingo tab
			else if(aS->activity_ == "bingo"){
				log(Info,std::string("[")+aS->action_name_.c_str()+std::string("]: Show bingo tab"));
				showingBingo = true;
                aS->feedback_.feedback = 3;
                aS->robot_guiAS_.publishFeedback(aS->feedback_);
				
                //emit changeTab(bingo_tab);
			}
			//show the bingo tab, but for trivia game
			else if(aS->activity_ == "trivia"){
				log(Info,std::string("[")+aS->action_name_.c_str()+std::string("]: Show trivia tab"));
				showingTrivia = true;
                aS->feedback_.feedback = 3;
                aS->robot_guiAS_.publishFeedback(aS->feedback_);
				
                emit changeTab(trivia_tab);
			}
			//show the log tab
			else if(aS->activity_ == "log_tab"){
				log(Info,std::string("[")+aS->action_name_.c_str()+std::string("]: Show log tab"));
				emit changeTab(log_tab);
			}



		}
		else{
			if(showingTelepresence){
				log(Info,std::string("[")+aS->action_name_.c_str()+std::string("]: Waiting for user"));
				aS->feedback_.feedback = 1;
				aS->robot_guiAS_.publishFeedback(aS->feedback_);
				//waits for the user interaction
				while(waiting){/*ros::spinOnce();loop_rate.sleep();*/}

				//checks if the session was accepted
				if(!callRejected){
					log(Info,std::string("[")+aS->action_name_.c_str()+std::string("]: On Call"));
					aS->feedback_.feedback = 2;
					aS->robot_guiAS_.publishFeedback(aS->feedback_);
					while(!sessionCompleted){
						//aS->robot_guiAS_.publishFeedback(aS->feedback_);
						/*ros::spinOnce();loop_rate.sleep();*/
					}
					log(Info,std::string("[")+aS->action_name_.c_str()+std::string("]: Session Completed"));
					aS->result_.end = 1;
					aS->robot_guiAS_.setSucceeded(aS->result_);
				}
				else{
					log(Info,std::string("[")+aS->action_name_.c_str()+std::string("]: Session rejected"));
					aS->result_.end = 2;
					aS->robot_guiAS_.setSucceeded(aS->result_);
				}
				showingTelepresence = false;
				waiting = true;
				sessionCompleted = false;
			}
			else if(showingBingo){
				log(Info,std::string("[")+aS->action_name_.c_str()+std::string("]: Showing bingo tab"));
				//execute the actions to bingo activity
				if((!speak) && (!showText)){
				// 	printf("Nothing to show or speak\n\n");
				}
				else{
					ROS_INFO("Bingo activity signal emitted");
					emit bingo_activity();
				}
               // sleep(1);           //DO NOT REMOVE THIS SLEEP! It allows the system to download the speech properly

				//showText = false;
				//speak = false;
				/*
				aS->result_.end = 3;
				aS->result_.statusCode = 1;
        aS->robot_guiAS_.setSucceeded(aS->result_);
				*/
				showingBingo = false;
			}
			else if(showingTrivia){
				log(Info,std::string("[")+aS->action_name_.c_str()+std::string("]: Showing trivia tab"));
				//execute the actions to bingo activity
				if((!speak) && (!showText)){
				// 	printf("Nothing to show or speak\n\n");
				}
				else
					emit trivia_activity();
				
                //sleep(1);

				showText = false;
				speak = false;

				aS->result_.end = 3;
				aS->result_.statusCode = 1;
                if(aS->code_==0)
                    aS->robot_guiAS_.setSucceeded(aS->result_);

				showingTrivia = false;
			}
			else if(showingDefaultTab){
				printf("I'll show the text in the tab\n\n");
				emit default_activity();
				sleep(5);
				
				emit reset_defaultTab();
				showText = false;
				speak = false;
				
				aS->result_.end = 0;
				aS->result_.statusCode = 1;
				aS->robot_guiAS_.setSucceeded(aS->result_);
				showingDefaultTab = false;
			}
		
		}

		if(!goal_finished){
			if((!speak) && (!showText)){
				goal_finished = true;
				//aS->result_.end = 3;
				//aS->result_.statusCode = 1;
        aS->robot_guiAS_.setSucceeded(aS->result_);
			}
		}

		//*/

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				// ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::closeInterface(){
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	//sleep(10);
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::getTTSDefaults(QString *lang, bool *male, bool *webbased){

	std::string langstd;
	if(!ros::param::get("/tts/language", langstd)){
		langstd = "en_us";
	}
	std::string genderstd;
	if(!ros::param::get("/tts/gender", genderstd)){
		genderstd = "male";
	}
	bool webstd;
	if(!ros::param::get("/tts/webbased", webstd)){
		webstd = true;
	}
	*male = (strcmp(genderstd.c_str(), "male") == 0);
	*webbased = webstd;
	*lang = QString::fromStdString(langstd);
}

void QNode::ttsSpeak(QString text, QString lang, bool male, bool webbased){

	// create and initialize request
	text_to_speech::tts srv;
	srv.request.speak = text.toUtf8().constData();
	srv.request.language = lang.toUtf8().constData();
	srv.request.webbased = webbased;
	if(male){
		srv.request.gender = text_to_speech::tts::Request::MALE;
	} else {
		srv.request.gender = text_to_speech::tts::Request::FEMALE;
	}
	// call request and dump whether it was successfull or not.
	if (tts_client.call(srv))
	{
		if(srv.response.success){
			ROS_INFO("Success!");
            if(aS->robot_guiAS_.isActive()){
            aS->robot_guiAS_.setSucceeded(aS->result_);
            }

		} else {
			ROS_INFO("Error!");
			ROS_ERROR(srv.response.error.c_str());
			aS->robot_guiAS_.setAborted(aS->result_); //Chris
		}
	}
	else
	{
		ROS_ERROR("Failed to call service. Error:");
		return;
	}

}
}  // namespace robot_gui
