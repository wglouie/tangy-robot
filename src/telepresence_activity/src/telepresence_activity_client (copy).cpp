//============================================================================
// Name        : robot_gui_client.cpp
// Author      : Rafael Farias
// Version     :
// Copyright   : Your copyright notice
// Description : 
//============================================================================

#include "../include/telepresence_activity_client.hpp"


TelepresenceActivityClient::TelepresenceActivityClient(std::string name):
	//Set up the client. It's publishing to topic "test_action", and is set to auto-spin
	ac("telepresence_activity_server", true),
	//Stores the name
	action_name_(name)
{
	//Get connection to a server
	ROS_INFO("[%s] Waiting For Server...", action_name_.c_str());
	//Wait for the connection to be valid
	ac.waitForServer();
	ROS_INFO("[%s] Got a Server...", action_name_.c_str());
}

void TelepresenceActivityClient::send(const telepresence_activity::Telepresence_activity_serverGoal goal){
	telepresence_activity::Telepresence_activity_serverGoal newGoal;
	newGoal.user = goal.user;
	newGoal.skypeUser = goal.skypeUser;
	
		
	//Once again, have to used boost::bind because you are inside a class
	ac.sendGoal(newGoal, boost::bind(&TelepresenceActivityClient::doneCb, this, _1, _2),
					boost::bind(&TelepresenceActivityClient::activeCb, this),
					boost::bind(&TelepresenceActivityClient::feedbackCb, this, _1));
}

void TelepresenceActivityClient::feedbackCb(const telepresence_activity::Telepresence_activity_serverFeedbackConstPtr& feedback){
	ROS_INFO("[%s} Receiving feedback...", action_name_.c_str());
	/*
	if(feedback->feedback == 0){
		ROS_INFO("[%s] Server idle", action_name.c_str());
	}
	else if(feedback->feedback == 1){
		ROS_INFO("[%s] Waiting for user", action_name.c_str());
	}
	else if(feedback->feedback == 2){
		ROS_INFO("[%s] On call", action_name.c_str());
	}
	else if(feedback->feedback == 3){
		ROS_INFO("[%s] Executing bingo activity", action_name.c_str());
	}
	*/
}

void TelepresenceActivityClient::activeCb(){

	ROS_INFO("[%s] Goal just went active...", action_name_.c_str());

}

void TelepresenceActivityClient::doneCb(const actionlib::SimpleClientGoalState& state,
		const telepresence_activity::Telepresence_activity_serverResultConstPtr& result){
	/*
	if(result->end == 1){
		ROS_INFO("[%s] Session completed", action_name.c_str());
	}
	else if(result->end == 2){
		ROS_INFO("[%s] Session rejected", action_name.c_str());
	}
	else if(result->end == 3){
		ROS_INFO("[%s] Bingo session executed", action_name.c_str());
	}
	
	ROS_INFO("[%s] It's done...", action_name.c_str());
	//ros::shutdown();
	*/
}

void TelepresenceActivityClient::waitForResult(){
	ROS_INFO("[%s] Waiting for result...", action_name_.c_str());
	ac.waitForResult();
}

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
	navigation_client.send(navigation_goal);

	ROS_INFO("[%s] Waiting for the first position", client.action_name_.c_str());
	navigation_client.ac.waitForResult();
	
	std::string command_speech_s1;
	command_speech_s1 = "echo \" I got the first position.\" | festival --tts";
	system(command_speech_s1.c_str());

	sleep(5);

	// first telepresence session	
	goal_.user = "Matthew";
	goal_.skypeUser = "echo123";
	goal_.nameOfCaller = "Tiago";

	client.sendGoal(goal_);
	

	client.ac.waitForResult();

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
	navigation_client.send(navigation_goal);

	ROS_INFO("[%s] Waiting for the second position", client.action_name_.c_str());
	navigation_client.ac.waitForResult();
	
	std::string command_speech_s2;
	command_speech_s2 = "echo \" I got the second position.\" | festival --tts";
	system(command_speech_s2.c_str());

	sleep(5);

	//second telepresence session	
	goal_.user = "Rafael";
	goal_.skypeUser = "echo123";
	goal_.nameOfCaller = "Tiago";

	client.send(goal_);

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
	navigation_client.send(navigation_goal);

	ROS_INFO("[%s] Waiting for the third position", client.action_name_.c_str());
	navigation_client.ac.waitForResult();
	
	std::string command_speech_s3;
	command_speech_s3 = "echo \" I got the third position.\" | festival --tts";
	system(command_speech_s3.c_str());

	sleep(5);
	
	ROS_INFO("[%s] It's done. ;)", client.action_name_.c_str());

	return 0;
}
