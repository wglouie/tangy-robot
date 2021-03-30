//include the libraryies necessary
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//#include <robot_gui/robot_gui_server.hpp>
#include "../include/robot_gui/robot_gui_server.hpp"
//#include "../include/robot_gui/qnode.hpp"

RobotGuiServer::RobotGuiServer(std::string name):
	robot_guiAS_(nh_, name, false),	//Initialize action server
//	//robot_guiAS_(nh_ , name , boost::bind(&RobotGuiServer::executeCB , this , _1) , false),
        action_name_(name)
	{
	
	/* Action library initialize */
	robot_guiAS_.registerGoalCallback(boost::bind(&RobotGuiServer::RobotGuiServerGoalCB, this));
	robot_guiAS_.registerPreemptCallback(boost::bind(&RobotGuiServer::RobotGuiServerPreemptCB, this));
	robot_guiAS_.start();
	newGoal = false;
	}

//RobotGuiServer::~RobotGuiServer()
//{
//}

/* Execute this code when a goal is provided from action client */
/* This function will start the telepresence GUI */

void RobotGuiServer::RobotGuiServerGoalCB()
{
	//client = new NavigationClient("myClient");

	goal_= robot_guiAS_.acceptNewGoal();

	activity_ = goal_->activity;//.c_str();
	code_ = goal_->code;
	speech_ = goal_->speech;
	text_ = goal_->text;
	subtab_ = goal_->subtab;

	//std::stringstream ss;
	//speech_ = "";

	//ss << goal_->speech;
	//speech_ = ss.str();

/*	ROS_INFO("[%s] Received: %d", action_name_.c_str(), code_);
	ROS_INFO("[%s] Received activity: %s", action_name_.c_str(), activity_.c_str());
	ROS_INFO("[%s] Received speech: %s", action_name_.c_str(), speech_.c_str());
	ROS_INFO("[%s] Received text: %s", action_name_.c_str(), text_.c_str());
*/
	newGoal = true;
}

void RobotGuiServer::RobotGuiServerPreemptCB()
{
	ROS_INFO("[%s]: Preempted", action_name_.c_str());
	// set the action state to preempted
	robot_guiAS_.setPreempted();
	//tab_ = 0;
	newGoal = false;
}

/*
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_gui_server");

  RobotGuiServer gui_server(ros::this_node::getName());
  ROS_INFO("GUI server is running");
  ros::spin();

  return 0;
}*/
