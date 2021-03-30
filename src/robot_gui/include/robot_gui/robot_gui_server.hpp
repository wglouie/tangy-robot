/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ROBOT_GUI_MESSAGE_ROBOT_GUIACTIONS_HPP_
#define ROBOT_GUI_MESSAGE_ROBOT_GUIACTIONS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_gui/Robot_guiAction.h>

//#include "navigation_client.hpp"

//#include "qnode.hpp"
#include <vector>

#include <string>
#include <std_msgs/String.h>
#include <sstream>

/*****************************************************************************
** Namespaces
*****************************************************************************/

/*****************************************************************************
** Class
*****************************************************************************/

class RobotGuiServer{

	ros::NodeHandle nh_;
//	NavigationClient *client;

public:
	RobotGuiServer(std::string name);
	//virtual ~RobotGuiServer();

    	void RobotGuiServerGoalCB();
    	void RobotGuiServerPreemptCB();
	
	bool newGoal;
	int code_;
	std::string activity_;
	std::string speech_;
	std::string text_;
	int subtab_;
	//std::stringstream ss;

    	/*Action library variables*/
	actionlib::SimpleActionServer<robot_gui::Robot_guiAction> robot_guiAS_;

	std::string action_name_;
	robot_gui::Robot_guiFeedback feedback_;//Provides feedback on the session status
	robot_gui::Robot_guiResult result_;	 //Provides feedback on the result of the session
	robot_gui::Robot_guiGoalConstPtr goal_;

//private:

};
#endif // ROBOT_GUI_MESSAGE_ROBOT_GUIACTION_H
