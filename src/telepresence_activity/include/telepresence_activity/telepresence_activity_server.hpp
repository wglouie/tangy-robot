/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <robot_gui/robot_gui_client.h>
#include <user_tracking/user_tracking_client.h>

#include <actionlib/server/simple_action_server.h>
#include <robot_gui/Robot_guiAction.h>
#include <telepresence_activity/Telepresence_activity_serverAction.h>

#include <string>
#include <std_msgs/String.h>
#include <sstream>

#include <geometry_msgs/Twist.h>
#include <drrobot_h20_player/HeadCmd.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

/*****************************************************************************
** Class
*****************************************************************************/

class TelepresenceActivityServer{

	ros::NodeHandle nh_, n;
//	NavigationClient *client;

public:
	TelepresenceActivityServer(std::string name);

    	void TelepresenceActivityServerGoalCB();
    	void TelepresenceActivityServerPreemptCB();
	void resetHead();
	//void executeCB(const telepresence_activity::Telepresence_activity_ServerGoal::ConstPtr &goal);
	
	//bool newGoal;	
	std::string user_;
	std::string skypeUser_;
	std::string nameOfCaller_;
	
	//navigation client and goal
	//NavigationClient *navigationClient;
	//move_base_msgs::MoveBaseGoal navigation_goal, navigation_goal1;

	//interface client and goal
	RobotGuiClient *robotGuiClient;
	robot_gui::Robot_guiGoal interface_goal;
	
	//userTracking client
	UserTrackingClient *userTrackingClient;
	

	//std::stringstream ss;

    	/*Action library variables*/
	actionlib::SimpleActionServer<telepresence_activity::Telepresence_activity_serverAction> aS_;

	std::string action_name_;
	telepresence_activity::Telepresence_activity_serverFeedback feedback_;//Provides feedback on the session status
	telepresence_activity::Telepresence_activity_serverResult result_;//Provides feedback on the result of the session
	telepresence_activity::Telepresence_activity_serverGoalConstPtr goal_;

//private:

	//geometry_msgs::Twist cmdvel_;
        drrobot_h20_player::HeadCmd cmdhead_;
        //doesn't need //ros::NodeHandle n_;
        ros::Publisher pub_base, pub_head;

};
