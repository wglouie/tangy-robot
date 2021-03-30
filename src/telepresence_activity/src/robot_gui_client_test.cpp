//============================================================================
// Name        : robot_gui_client.cpp
// Author      : Rafael Farias
// Version     :
// Copyright   : Your copyright notice
// Description : 
//============================================================================

#include "../include/robot_gui/robot_gui_client.h"


int main (int argc, char **argv)
{
	ros::init(argc, argv, "robot_gui_client_node");
	ros::NodeHandle nh_;
	ros::Rate loop_rate(1);

	robot_gui::Robot_guiGoal goal_;


	//Usage check to make sure the client is being used properly

	//Initialize the client
	RobotGuiClient client("robot_gui_client");

	goal_.activity = "telepresence";
	goal_.code = -1;
	goal_.speech = "Tiago";
	goal_.text = "echo123";

	client.sendGoal(goal_);

	client.waitForResult();
//*/
/*	std::stringstream aux,aux1;
	//aux  << "testing";
	//aux1 << "nothing to show for while";	
*/
/*	for(int a = 0; a < 3; a++){
	//sleep(5);

	goal_.activity = "bingo";
	goal_.code = a;//2;
	//aux << aux.str();
	goal_.speech = "Rafael";
	//aux1 << aux1.str();
	goal_.text = "nothing to show for while";

	client.send(goal_);
	client.waitForResult();
	}
*/
	//ROS_INFO("sendig goal to change to bingo_tab");
	//client.send(3,"testing");
	//ROS_INFO("sent goal to change to bingo_tab");
	//client.waitForResult();

	//while(ros::ok()){
	//	ros::spinOnce();
	//        loop_rate.sleep();
	//}

	return 0;
}
