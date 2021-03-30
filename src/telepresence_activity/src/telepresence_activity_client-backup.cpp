//============================================================================
// Name        : robot_gui_client.cpp
// Author      : Rafael Farias
// Version     :
// Copyright   : Your copyright notice
// Description : 
//============================================================================

//#include "../include/telepresence_activity_client.hpp"
#include <telepresence_activity/telepresence_activity_client.h>
//#include <telepresence_activity/navigation_client.hpp>
#include <tangerine_2dnav/navigation_client.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "telepresence_activity_client_node");
	ros::NodeHandle nh_;
	ros::Rate loop_rate(1);

	telepresence_activity::Telepresence_activity_serverGoal goal_;
	//Initialize the client
	TelepresenceActivityClient client("telepresence_activity_client");

	move_base_msgs::MoveBaseGoal navigation_goal, navigation_goal1;
	NavigationClient navigation_client("navigation_client");

	double position3[1][7] = {{0.550,0.000,0.000,0.000,0.000,-0.707,0.707}};
	double position1[1][7] = { {3.786,1.590,0.000,0.000,0.000,-0.028,1.000}};
	double position2[1][7] = { {4.456,3.811,0.000,0.000,0.000,-0.006,1.000}};
//	double initialPosition[1][7] = { {31.261,9.950,0.000,0.000,0.000,0.000,1.000}};
	double initialPosition[1][7] = { {0.700,0.850,0.000,0.000,0.000,0.000,1.000}};	



	ROS_INFO("[%s] Starting new navigation", client.action_name_.c_str());

	navigation_goal.target_pose.header.frame_id = "/map";
	navigation_goal.target_pose.header.stamp = ros::Time::now();
	navigation_goal.target_pose.pose.position.x = position1[0][0];
	navigation_goal.target_pose.pose.position.y = position1[0][1];
	navigation_goal.target_pose.pose.position.z = position1[0][2];
	navigation_goal.target_pose.pose.orientation.x = position1[0][3];
	navigation_goal.target_pose.pose.orientation.y = position1[0][4];
	navigation_goal.target_pose.pose.orientation.z = position1[0][5];
	navigation_goal.target_pose.pose.orientation.w = position1[0][6];

	ROS_INFO("[%s] Sending goal to the navigation server", client.action_name_.c_str());
//	navigation_client.sendGoal(navigation_goal);

	ROS_INFO("[%s] Waiting for the first position", client.action_name_.c_str());
//	navigation_client.ac.waitForResult();
	
	std::string command_speech_s1;
	command_speech_s1 = "echo \" I got the first position.\" | festival --tts";
	system(command_speech_s1.c_str());

	sleep(5);



	// first telepresence session	
	goal_.user = "Matthew";
	goal_.skypeUser = "echo123";
	goal_.nameOfCaller = "Tiago";
	client.sendGoal(goal_);
	client.waitForResult();

	sleep(2);


	//second part of navigation
	ROS_INFO("[%s] Starting new navigation", client.action_name_.c_str());

	navigation_goal.target_pose.header.frame_id = "/map";
	navigation_goal.target_pose.header.stamp = ros::Time::now();
	navigation_goal.target_pose.pose.position.x = position2[0][0];
	navigation_goal.target_pose.pose.position.y = position2[0][1];
	navigation_goal.target_pose.pose.position.z = position2[0][2];
	navigation_goal.target_pose.pose.orientation.x = position2[0][3];
	navigation_goal.target_pose.pose.orientation.y = position2[0][4];
	navigation_goal.target_pose.pose.orientation.z = position2[0][5];
	navigation_goal.target_pose.pose.orientation.w = position2[0][6];

	ROS_INFO("[%s] Sending goal to the navigation server", client.action_name_.c_str());
//	navigation_client.sendGoal(navigation_goal);

	ROS_INFO("[%s] Waiting for the second position", client.action_name_.c_str());
//	navigation_client.ac.waitForResult();
	
	std::string command_speech_s2;
	command_speech_s2 = "echo \" I got the second position.\" | festival --tts";
	system(command_speech_s2.c_str());

	sleep(5);


	//second telepresence session	
	goal_.user = "Rafael";
	goal_.skypeUser = "echo123";
	goal_.nameOfCaller = "Tiago";
	client.sendGoal(goal_);
	client.waitForResult();

	sleep(2);


	//second part of navigation
	ROS_INFO("[%s] Starting new navigation", client.action_name_.c_str());

	navigation_goal.target_pose.header.frame_id = "/map";
	navigation_goal.target_pose.header.stamp = ros::Time::now();
	navigation_goal.target_pose.pose.position.x = position3[0][0];
	navigation_goal.target_pose.pose.position.y = position3[0][1];
	navigation_goal.target_pose.pose.position.z = position3[0][2];
	navigation_goal.target_pose.pose.orientation.x = position3[0][3];
	navigation_goal.target_pose.pose.orientation.y = position3[0][4];
	navigation_goal.target_pose.pose.orientation.z = position3[0][5];
	navigation_goal.target_pose.pose.orientation.w = position3[0][6];

	ROS_INFO("[%s] Sending goal to the navigation server", client.action_name_.c_str());
//	navigation_client.sendGoal(navigation_goal);

	ROS_INFO("[%s] Waiting for the third position", client.action_name_.c_str());
//	navigation_client.ac.waitForResult();
	
	std::string command_speech_s3;
	command_speech_s3 = "echo \" I got the third position.\" | festival --tts";
	system(command_speech_s3.c_str());

	sleep(5);
	
	ROS_INFO("[%s] It's done. ;)", client.action_name_.c_str());

	return 0;
}
