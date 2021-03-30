//============================================================================
// Name        : arm_player.cpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Using code from DrRobotMotionArmDriver to drive arm movements
//============================================================================
#include "ros/ros.h"
#include <vector>
#include <iostream>
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"
#include "control_msgs/FollowJointTrajectoryActionFeedback.h"
#include "arm_player_eff.hpp"
#include "controller_manager/controller_manager.h"
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include <ros/console.h>
#include <controller_manager/controller_loader.h>
#include <controller_manager_msgs/ControllerState.h>
#include <unistd.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>


	arm_player::arm_player()
	{

	  
		_robotConfig = new DrRobotArmConfig;
		_robotConfig->robotID = "DrRobot";
		_robotConfig->robotIP = "127.0.0.1";
		_robotConfig->portNum = DEFAULT_PORT;

		client.sin_family = AF_INET;
		client.sin_port = htons(_robotConfig->portNum);
		client.sin_addr.s_addr = inet_addr(_robotConfig->robotIP.c_str());

		sockCheck = socket(AF_INET, SOCK_STREAM, 0);
		connectCheck = 0;
		msgCheck = 0;
		
  	// connect and register the joint state interfaces
    hardware_interface::JointStateHandle state_handle_0("lower_to_wrist_right_arm", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_0);
    hardware_interface::JointStateHandle state_handle_1("wrist_right_arm_z_revolute_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_1);
    hardware_interface::JointStateHandle state_handle_2("upper_to_lower_right_joint", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_2);
    hardware_interface::JointStateHandle state_handle_3("upper_right_arm_z_revolute_joint", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_3);
    hardware_interface::JointStateHandle state_handle_4("upper_right_arm_y_revolute_joint", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_4);
    hardware_interface::JointStateHandle state_handle_5("right_shoulder_x_revolute", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_5);
    hardware_interface::JointStateHandle state_handle_6("lower_to_wrist_left_arm", &pos[6], &vel[6], &eff[6]);
    jnt_state_interface.registerHandle(state_handle_6);
    hardware_interface::JointStateHandle state_handle_7("wrist_left_arm_z_revolute_joint", &pos[7], &vel[7], &eff[7]);
    jnt_state_interface.registerHandle(state_handle_7);
    hardware_interface::JointStateHandle state_handle_8("upper_to_lower_left_joint", &pos[8], &vel[8], &eff[8]);
    jnt_state_interface.registerHandle(state_handle_8);
    hardware_interface::JointStateHandle state_handle_9("upper_left_arm_z_revolute_joint", &pos[9], &vel[9], &eff[9]);
    jnt_state_interface.registerHandle(state_handle_9);
    hardware_interface::JointStateHandle state_handle_10("upper_left_arm_y_revolute_joint", &pos[10], &vel[10], &eff[10]);
    jnt_state_interface.registerHandle(state_handle_10);
    hardware_interface::JointStateHandle state_handle_11("left_shoulder_x_revolute", &pos[11], &vel[11], &eff[11]);
    jnt_state_interface.registerHandle(state_handle_11);
    
    this->registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle eff_handle_0(jnt_state_interface.getHandle("lower_to_wrist_right_arm"), &cmd[0]);
    jnt_eff_interface.registerHandle(eff_handle_0);
    hardware_interface::JointHandle eff_handle_1(jnt_state_interface.getHandle("wrist_right_arm_z_revolute_joint"), &cmd[1]);
    jnt_eff_interface.registerHandle(eff_handle_1);
    hardware_interface::JointHandle eff_handle_2(jnt_state_interface.getHandle("upper_to_lower_right_joint"), &cmd[2]);
    jnt_eff_interface.registerHandle(eff_handle_2);
    hardware_interface::JointHandle eff_handle_3(jnt_state_interface.getHandle("upper_right_arm_z_revolute_joint"), &cmd[3]);
    jnt_eff_interface.registerHandle(eff_handle_3);
    hardware_interface::JointHandle eff_handle_4(jnt_state_interface.getHandle("upper_right_arm_y_revolute_joint"), &cmd[4]);
    jnt_eff_interface.registerHandle(eff_handle_4);
    hardware_interface::JointHandle eff_handle_5(jnt_state_interface.getHandle("right_shoulder_x_revolute"), &cmd[5]);
    jnt_eff_interface.registerHandle(eff_handle_5);
    hardware_interface::JointHandle eff_handle_6(jnt_state_interface.getHandle("lower_to_wrist_left_arm"), &cmd[6]);
    jnt_eff_interface.registerHandle(eff_handle_6);
    hardware_interface::JointHandle eff_handle_7(jnt_state_interface.getHandle("wrist_left_arm_z_revolute_joint"), &cmd[7]);
    jnt_eff_interface.registerHandle(eff_handle_7);
    hardware_interface::JointHandle eff_handle_8(jnt_state_interface.getHandle("upper_to_lower_left_joint"), &cmd[8]);
    jnt_eff_interface.registerHandle(eff_handle_8);
    hardware_interface::JointHandle eff_handle_9(jnt_state_interface.getHandle("upper_left_arm_z_revolute_joint"), &cmd[9]);
    jnt_eff_interface.registerHandle(eff_handle_9);
    hardware_interface::JointHandle eff_handle_10(jnt_state_interface.getHandle("upper_left_arm_y_revolute_joint"), &cmd[10]);
    jnt_eff_interface.registerHandle(eff_handle_10);
    hardware_interface::JointHandle eff_handle_11(jnt_state_interface.getHandle("left_shoulder_x_revolute"), &cmd[11]);
    jnt_eff_interface.registerHandle(eff_handle_11);
    
  	this->registerInterface(&jnt_eff_interface);
		
	}

	arm_player::~arm_player()
	{

	}

	void arm_player::close()
	{
		if (sockCheck > 0)
		    {
			  shutdown(sockCheck,SHUT_RDWR);
		      sockCheck = -1;
		    }
	}

	int arm_player::vali_ip(const char*  ip_str)
	{
	  unsigned int n1,n2,n3,n4;
	  if ( sscanf(ip_str, "%u.%u.%u.%u", &n1,&n2,&n3,&n4) != 4 ) return 1;
	  if ((n1 != 0) && (n1 <= 255) && (n2 <= 255) && (n3 <= 255) && (n4 <= 255) )
	  {
	    char buf[64];
	    sprintf(buf,"%u.%u.%u.%u", n1,n2,n3,n4);
	    if (strcmp(buf,ip_str)) return 1;
	    return 0;
	  }

	  return 1;
	}
	
	int arm_player::openConnection(std::string robotIP, int portNum)
	{

		//check the parameter first
		sockCheck = socket(AF_INET, SOCK_STREAM, 0);

		if(sockCheck == -1)
		{
			printf("Could not create socket\n");
			return 0;
		}

		printf("Socket created!\n");

		if (portNum <= 0)
		{
		  return -1;
		}

		if (vali_ip(robotIP.c_str()) == 1)
		{
		  printf("DrRobot Motion/Sensor Driver Error Message: invalid IP address: %s\n", robotIP.c_str());
		  return -2;
		}

		_robotConfig->portNum = portNum;
		_robotConfig->robotIP = robotIP;
		client.sin_family = AF_INET;
		client.sin_port = htons(_robotConfig->portNum);

		if (inet_addr(_robotConfig->robotIP.c_str()) < 0)
		{
			printf("DrRobot Motion/Sensor Driver Error Message: invalid IP address: %s\n", _robotConfig->robotIP.c_str());
			return -3;
		}
		else
		{
			client.sin_addr.s_addr = inet_addr(_robotConfig->robotIP.c_str());
		}
		
		printf("Connecting to %s(%d)...\n",_robotConfig->robotIP.c_str(),_robotConfig->portNum);
		connectCheck = connect(sockCheck , (struct sockaddr *)&client , sizeof(client));

		if(connectCheck < 0)
		{
			printf("Connection failed.\n");
			return -4;
		}

		printf("Connected!\n");

		return 1;

	}

	void arm_player::getDrRobotArmDriverConfig(DrRobotArmConfig *driverConfig)
	{
		driverConfig->portNum = _robotConfig->portNum;
		driverConfig->robotID = _robotConfig->robotID;
		driverConfig->robotIP = _robotConfig->robotIP;
	}

	void arm_player::setDrRobotArmDriverConfig(DrRobotArmConfig *driverConfig)
	{
		_robotConfig->portNum = driverConfig->portNum;
		_robotConfig->robotID = driverConfig->robotID;
		_robotConfig->robotIP = driverConfig->robotIP;
	}
	
	int arm_player::sendServoCtrlAllCmd(int cmd1, int cmd2, int cmd3, int cmd4, int cmd5, int cmd6, int cmd7, int cmd8, int cmd9, int cmd10, int cmd11, int cmd12, int cmd13, int cmd14, int cmd15, int cmd16)
	{
		int poses[16];
		int i;

		if (cmd1 != NOCONTROL)
		{
			if (cmd1 < min_pos[0]) cmd1 = min_pos[0];
			if (cmd1 > max_pos[0]) cmd1 = max_pos[0];
		}
		if (cmd2 != NOCONTROL)
		{
			if (cmd2 < min_pos[1]) cmd2 = min_pos[1];
			if (cmd2 > max_pos[1]) cmd2 = max_pos[1];
		}
		if (cmd3 != NOCONTROL)
		{
			if (cmd3 < min_pos[2]) cmd3 = min_pos[2];
			if (cmd3 > max_pos[2]) cmd3 = max_pos[2];
		}
		if (cmd4 != NOCONTROL)
		{
			if (cmd4 < min_pos[3]) cmd4 = min_pos[3];
			if (cmd4 > max_pos[3]) cmd4 = max_pos[3];
		}
		if (cmd5 != NOCONTROL)
		{
			if (cmd5 < min_pos[4]) cmd5 = min_pos[4];
			if (cmd5 > max_pos[4]) cmd5 = max_pos[4];
		}
		if (cmd6 != NOCONTROL)
		{
			if (cmd6 < min_pos[5]) cmd6 = min_pos[5];
			if (cmd6 > max_pos[5]) cmd6 = max_pos[5];
		}
		if (cmd7 != NOCONTROL)
		{
			if (cmd7 < min_pos[6]) cmd7 = min_pos[6];
			if (cmd7 > max_pos[6]) cmd7 = max_pos[6];

		}
		if (cmd8 != NOCONTROL)
		{
			if (cmd8 < min_pos[7]) cmd8 = min_pos[7];
			if (cmd8 > max_pos[7]) cmd8 = max_pos[7];
		}
		if (cmd9 != NOCONTROL)
		{
			if (cmd9 < min_pos[8]) cmd9 = min_pos[8];
			if (cmd9 > max_pos[8]) cmd9 = max_pos[8];
		}
		if (cmd10 != NOCONTROL)
		{
			if (cmd10 < min_pos[9]) cmd10 = min_pos[9];
			if (cmd10 > max_pos[9]) cmd10 = max_pos[9];

		}
		if (cmd11 != NOCONTROL)
		{
			if (cmd11 < min_pos[10]) cmd11 = min_pos[10];
			if (cmd11 > max_pos[10]) cmd11 = max_pos[10];
		}
		if (cmd12 != NOCONTROL)
		{
			if (cmd12 < min_pos[11]) cmd12 = min_pos[11];
			if (cmd12 > max_pos[11]) cmd12 = max_pos[11];
		}
		if (cmd13 != NOCONTROL)
		{
			if (cmd13 < min_pos[12]) cmd13 = min_pos[12];
			if (cmd13 > max_pos[12]) cmd13 = max_pos[12];
		}
		if (cmd14 != NOCONTROL)
		{
			if (cmd14 < min_pos[13]) cmd14 = min_pos[13];
			if (cmd14 > max_pos[13]) cmd14 = max_pos[13];
		}
		if (cmd15 != NOCONTROL)
		{
			if (cmd15 < min_pos[14]) cmd15 = min_pos[14];
			if (cmd15 > max_pos[14]) cmd15 = max_pos[14];
		}
		if (cmd16 != NOCONTROL)
		{
			if (cmd16 < min_pos[15]) cmd1 = min_pos[15];
			if (cmd16 > max_pos[15]) cmd1 = max_pos[15];
		}

		for (i = 0; i < 16; i++)
		{
			if (i == 0)
			{
				poses[i] = cmd1;
			}
			if (i == 1)
			{
				poses[i] = cmd2;
			}
			if (i == 2)
			{
				poses[i] = cmd3;
			}
			if (i == 3)
			{
				poses[i] = cmd4;
			}
			if (i == 4)
			{
				poses[i] = cmd5;
			}
			if (i == 5)
			{
				poses[i] = cmd6;
			}
			if (i == 6)
			{
				poses[i] = cmd7;
			}
			if (i == 7)
			{
				poses[i] = cmd8;
			}
			if (i == 8)
			{
			 // poses[i]=NOCONTROL;
				poses[i] = cmd9;
			}
			if (i == 9)
			{
			 // poses[i]=NOCONTROL;
				poses[i] = cmd10;
			}
			if (i == 10)
			{
			 // poses[i]=NOCONTROL;
				poses[i] = cmd11;
			}
			if (i == 11)
			{
			 // poses[i]=NOCONTROL;
				poses[i] = cmd12;
			}
			if (i == 12)
			{
			 // poses[i]=NOCONTROL;
				poses[i] = cmd13;
			}
			if (i == 13)
			{
			 // poses[i]=NOCONTROL;
				poses[i] = cmd14;
			}
			if (i == 14)
			{
			 // poses[i]=NOCONTROL;
				poses[i] = cmd15;
			}
			if (i == 15)
			{
			 // poses[i]=NOCONTROL;
				poses[i] = cmd16;
			}
		}

		return sendCommand(poses, i, 2);
	}
	
	int arm_player::sendServoCtrlAllCmd(int cmd1, int cmd2, int cmd3, int cmd4, int cmd5, int cmd6, int cmd7, int cmd8, int cmd9, int cmd10, int cmd11, int cmd12, int cmd13, int cmd14, int cmd15, int cmd16, int vel, bool speed)
	{
		int poses[18];
		int i;

		if (cmd1 != NOCONTROL)
		{
			if (cmd1 < min_pos[0]) cmd1 = min_pos[0];
			if (cmd1 > max_pos[0]) cmd1 = max_pos[0];
		}
		if (cmd2 != NOCONTROL)
		{
			if (cmd2 < min_pos[1]) cmd2 = min_pos[1];
			if (cmd2 > max_pos[1]) cmd2 = max_pos[1];
		}
		if (cmd3 != NOCONTROL)
		{
			if (cmd3 < min_pos[2]) cmd3 = min_pos[2];
			if (cmd3 > max_pos[2]) cmd3 = max_pos[2];
		}
		if (cmd4 != NOCONTROL)
		{
			if (cmd4 < min_pos[3]) cmd4 = min_pos[3];
			if (cmd4 > max_pos[3]) cmd4 = max_pos[3];
		}
		if (cmd5 != NOCONTROL)
		{
			if (cmd5 < min_pos[4]) cmd5 = min_pos[4];
			if (cmd5 > max_pos[4]) cmd5 = max_pos[4];
		}
		if (cmd6 != NOCONTROL)
		{
			if (cmd6 < min_pos[5]) cmd6 = min_pos[5];
			if (cmd6 > max_pos[5]) cmd6 = max_pos[5];
		}
		if (cmd7 != NOCONTROL)
		{
			if (cmd7 < min_pos[6]) cmd7 = min_pos[6];
			if (cmd7 > max_pos[6]) cmd7 = max_pos[6];

		}
		if (cmd8 != NOCONTROL)
		{
			if (cmd8 < min_pos[7]) cmd8 = min_pos[7];
			if (cmd8 > max_pos[7]) cmd8 = max_pos[7];
		}
		if (cmd9 != NOCONTROL)
		{
			if (cmd9 < min_pos[8]) cmd9 = min_pos[8];
			if (cmd9 > max_pos[8]) cmd9 = max_pos[8];
		}
		if (cmd10 != NOCONTROL)
		{
			if (cmd10 < min_pos[9]) cmd10 = min_pos[9];
			if (cmd10 > max_pos[9]) cmd10 = max_pos[9];

		}
		if (cmd11 != NOCONTROL)
		{
			if (cmd11 < min_pos[10]) cmd11 = min_pos[10];
			if (cmd11 > max_pos[10]) cmd11 = max_pos[10];
		}
		if (cmd12 != NOCONTROL)
		{
			if (cmd12 < min_pos[11]) cmd12 = min_pos[11];
			if (cmd12 > max_pos[11]) cmd12 = max_pos[11];
		}
		if (cmd13 != NOCONTROL)
		{
			if (cmd13 < min_pos[12]) cmd13 = min_pos[12];
			if (cmd13 > max_pos[12]) cmd13 = max_pos[12];
		}
		if (cmd14 != NOCONTROL)
		{
			if (cmd14 < min_pos[13]) cmd14 = min_pos[13];
			if (cmd14 > max_pos[13]) cmd14 = max_pos[13];
		}
		if (cmd15 != NOCONTROL)
		{
			if (cmd15 < min_pos[14]) cmd15 = min_pos[14];
			if (cmd15 > max_pos[14]) cmd15 = max_pos[14];
		}
		if (cmd16 != NOCONTROL)
		{
			if (cmd16 < min_pos[15]) cmd1 = min_pos[15];
			if (cmd16 > max_pos[15]) cmd1 = max_pos[15];
		}


		if (vel <= 0) vel = 1;
		

		for (i = 0; i < 18; i++)
		{
			if (i == 0)
			{
				poses[i] = cmd1;
			}
			if (i == 1)
			{
				poses[i] = cmd2;
			}
			if (i == 2)
			{
				poses[i] = cmd3;
			}
			if (i == 3)
			{
				poses[i] = cmd4;
			}
			if (i == 4)
			{
				poses[i] = cmd5;
			}
			if (i == 5)
			{
				poses[i] = cmd6;
			}
			if (i == 6)
			{
				poses[i] = cmd7;
			}
			if (i == 7)
			{
				poses[i] = cmd8;
			}
			if (i == 8)
			{
				poses[i] = cmd9;
			}
			if (i == 9)
			{
				poses[i] = cmd10;
			}
			if (i == 10)
			{
				poses[i] = cmd11;
			}
			if (i == 11)
			{
				poses[i] = cmd12;
			}
			if (i == 12)
			{
				poses[i] = cmd13;
			}
			if (i == 13)
			{
				poses[i] = cmd14;
			}
			if (i == 14)
			{
				poses[i] = cmd15;
			}
			if (i == 15)
			{
				poses[i] = cmd16;
			}
			if (i == 16)
			{
				poses[i] = vel;
			}
			if (i == 17)
			{
				if (speed == true) poses[i] = 1;
				else poses[i] = 0;
			}
		}

		return sendCommand(poses, i, 1);
	}


	//This function send a char with the commands to the controller, it return -1 if fails.
	//It receive a vector of positions, the vector length and the mode, which is the type of control the user wants.
	//Could be:
	//mode1: Send commands with time or speed setup
	//mode2: Send commands with time default
	int arm_player::sendCommand(int poses[], int vecLen, int mode)
	{

		std::string message;
		int servo_id, temp;

		if (mode == 1)
		{

			for (servo_id = 0; servo_id < (vecLen - 2); servo_id++)
			{
				if (poses[servo_id] != NOCONTROL)
				{
					temp = servo_id;
					
					if(servo_id > 7)
					{
						temp = temp + SERVO_CHANNEL_OFFSET;
						message += "#" + to_string(temp) + " P" + to_string(poses[servo_id]) + " ";
					}
					else
					{
						message += "#" + to_string(servo_id) + " P" + to_string(poses[servo_id]) + " ";
					}
					
				}
			}

			if(poses[vecLen-1] == 0) message += "T" + to_string(poses[vecLen-2]) + '\r';
			
			if(poses[vecLen-1] == 1) message += "S" + to_string(poses[vecLen-2]) + '\r';

		}

		if (mode == 2)
		{
			for (servo_id = 0; servo_id < (vecLen); servo_id++)
			{
				if (poses[servo_id] != NOCONTROL)
				{
					temp = servo_id;
					
					if(servo_id > 7)
					{
						temp = temp + SERVO_CHANNEL_OFFSET;
						message += "#" + to_string(temp) + " P" + to_string(poses[servo_id]) + " ";
					}
					else
					{
						message += "#" + to_string(servo_id) + " P" + to_string(poses[servo_id]) + " ";
					}
				}
			}
			message += "T500\r";

		}
		
		//ROS_INFO("right_arm [%d %d %d %d %d %d %d %d]", poses[0], poses[1], poses[2], poses[3], poses[4], poses[5], poses[6], poses[7]);
		//ROS_INFO("left_arm [%d %d %d %d %d %d %d %d]",poses[8], poses[9], poses[10], poses[11], poses[12], poses[13], poses[14], poses[15]);

		msgCheck = send(sockCheck, message.c_str(), strlen(message.c_str()), 0);

		if (msgCheck < 0)
		{
			puts("Send failed");
			return -1;
		}
		
		//checkStatus();
		
		return 0;
	}
	
	int arm_player::checkStatus()
	{
		char feedBack[10];
		int queue;

		client_len = sizeof(struct sockaddr_in);
		
		if (sockCheck < 0)
		{
			puts("Create socket first.\n");
			return -1;
		}
		if (connectCheck < 0)
		{
			puts("You are not connect to the server...");
			return -2;
		}
		
		puts("Waiting for movements...");
		
		while(1)
		{
			queue = send(sockCheck, "q", 1, 0);
			if (queue < 0)
			{
				puts("Can not send messages.");
				return -3;
			}
			puts("here");
					
			if( recv(sockCheck , feedBack , 10 , MSG_OOB) < 0)
			{
				puts("Can not receive messages!");
				return -4;
			}
			puts("here too");
					
			if(feedBack == "+") puts("Action in progress, still moving...");
			else
			{
				puts("Action finished!");
				break;
			}
		}
		
		return 0;
	}
	
	//The following functions populate the vector of servo commands for each joint
	//Each trajCallback function adds elements onto a different joint
	//The functions use the trajectory information broadcasted on the controllers
	//servo_goals_0 corresponds to servo 3 on the right arm, as defined by the pictures in file servo_calibration_values.xlsx
	void arm_player::trajCallback0(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_0.assign(num_points,0);
	  for(int i=0; i<num_points; i++){
	    servo_goals_0.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-496.56)+rest_pos_s2;
	    ROS_INFO("Callback was called!");
    }
	}
	//servo_goals_1 corresponds to servo 4 on the right arm, as defined by the pictures in file servo_calibration_values.xlsx
	void arm_player::trajCallback1(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_1.assign(num_points,0);
	  for(int i=0; i<num_points; i++){
	    servo_goals_1.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-582.51)+rest_pos_s3;
    }
	}
	//defined in same scheme as above
	void arm_player::trajCallback2(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_2.assign(num_points,0);
	  for(int i=0; i<num_points; i++){
	    servo_goals_2.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-181.44)+rest_pos_s4;
    }
	}
	
	void arm_player::trajCallback3(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_3.assign(num_points,0);
	  for(int i=0; i<num_points; i++){
	    servo_goals_3.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(175.07)+rest_pos_s5;
    }
	}
	
	void arm_player::trajCallback4(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_4.assign(num_points,0);
	  for(int i=0; i<num_points; i++){
	    servo_goals_4.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(522.03)+rest_pos_s6;
    }
	}
	
	void arm_player::trajCallback5(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_5.assign(num_points,0);
	  for(int i=0; i<num_points; i++){
	    servo_goals_5.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(534.76)+rest_pos_s7;
    }
	}
	//servo_goals_6 corresponds to servo 3 on left arm, and etc
	void arm_player::trajCallback6(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_6.assign(num_points,0);
	  for(int i=0; i<num_points; i++){
	    servo_goals_6.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-612.98)+rest_pos_s10;
    }
	}
	
	void arm_player::trajCallback7(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_7.assign(num_points,0);
	  for(int i=0; i<num_points; i++){
	    servo_goals_7.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(588.873)+rest_pos_s11;
    }
	}
	
	void arm_player::trajCallback8(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_8.assign(num_points,0);
	  for(int i=0; i<num_points; i++){
	    servo_goals_8.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(194.91)+rest_pos_s12;
    }
	}
	
	void arm_player::trajCallback9(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_9.assign(num_points,0);
	  for(int i=0; i<num_points; i++){
	    servo_goals_9.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-185.53)+rest_pos_s13;
    }
	}
	
	void arm_player::trajCallback10(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_10.assign(num_points,0);
	  for(int i=0; i<num_points; i++){
	    servo_goals_10.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-413.11)+rest_pos_s14;
    }
	}
	
	void arm_player::trajCallback11(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_11.assign(num_points,0);
	  for(int i=0; i<num_points; i++){
	    servo_goals_11.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-502.2)+rest_pos_s15;
    }
	}

///////////////*****************************************GEOFF ADDED THIS NEW STUFF **************************/////////////////////

	void arm_player::jointCb_right(drrobot_h20_arm_player::arm_cmd goal){
		int rest_positions[6] = {rest_pos_s2, rest_pos_s3, rest_pos_s4, rest_pos_s5, rest_pos_s6, rest_pos_s7};
		float servo_to_angs[6] = {-496.56,-582.51,-181.44,175.07,-509.29,534.76};
		std::vector<int> servo_values;
		for(int i = 0; i < 6; i++) {
			servo_values.push_back(goal.joint_commands[i].joint_angle * servo_to_angs[i] + rest_positions[i]);
		}   
		this->sendServoCtrlAllCmd(NOCONTROL, NOCONTROL, servo_values[0], servo_values[1], servo_values[2], servo_values[3], servo_values[4],
	                            servo_values[5], NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL,
															goal.velocity,false);		
	}

	void arm_player::jointCb_left(drrobot_h20_arm_player::arm_cmd goal){
		int rest_positions[6] = {rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, rest_pos_s14, rest_pos_s15};
		float servo_to_angs[6] = {-612.98,588.873,194.91,-185.53, 464.73,-502.2};
		std::vector<int> servo_values;
		for(int i = 0; i < 6; i++) {
			servo_values.push_back(goal.joint_commands[i].joint_angle * servo_to_angs[i] + rest_positions[i]);
		}   
		this->sendServoCtrlAllCmd(NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL, 
		                          NOCONTROL, NOCONTROL, servo_values[0], servo_values[1], servo_values[2], servo_values[3], servo_values[4],
	                            servo_values[5], goal.velocity,false);	

  }

///////////////*************************************************************************************************/////////////////////
	//Checks whether the right or the left arm is being moved, and assigns default values to the other arm's servos
	void arm_player::move_group_checker(std::vector<int> curr_pos)
	{
	  ROS_INFO("move_group_checker called");
	  int num_points;
	  if(servo_goals_0.empty())
	  {
	    num_points=servo_goals_6.size();
	    servo_goals_0.assign(num_points, curr_pos.at(2));
	    servo_goals_1.assign(num_points, curr_pos.at(3));
	    servo_goals_2.assign(num_points, curr_pos.at(4));
	    servo_goals_3.assign(num_points, curr_pos.at(5));
	    servo_goals_4.assign(num_points, curr_pos.at(6));
	    servo_goals_5.assign(num_points, curr_pos.at(7));
	  }
	  if(servo_goals_6.empty())
	  {
	    num_points=servo_goals_0.size();
	    servo_goals_6.assign(num_points, curr_pos.at(10));
	    servo_goals_7.assign(num_points, curr_pos.at(11));
	    servo_goals_8.assign(num_points, curr_pos.at(12));
	    servo_goals_9.assign(num_points, curr_pos.at(13));
	    servo_goals_10.assign(num_points, curr_pos.at(14));
	    servo_goals_11.assign(num_points, curr_pos.at(15));
	  }
	 }


    void arm_player::start_arm_cb(const std_msgs::String::ConstPtr& msg){
        std::string command = msg->data;
        if(command.compare("start") == 0){
            std::string ip = "192.168.0.94";
            openConnection(ip,10001);
 			sendServoCtrlAllCmd(rest_pos_s0 ,rest_pos_s1, rest_pos_s2, rest_pos_s3, rest_pos_s4, rest_pos_s5, rest_pos_s6, rest_pos_s7, rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, rest_pos_s14, rest_pos_s15);
            ROS_INFO("Starting up arms to receive commands");
        } else if(command.compare("stop") == 0){
			sendServoCtrlAllCmd(rest_pos_s0 ,rest_pos_s1, rest_pos_s2, rest_pos_s3, rest_pos_s4, rest_pos_s5, rest_pos_s6, rest_pos_s7, rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, rest_pos_s14, rest_pos_s15);
            close();
            ROS_INFO("Shutting down arms to refuse all commands sent");
        } else {
            ROS_WARN("Command does not exist for start arm subscriber");
        }

    }


	void arm_player::fnExit()
	{
	  this->sendServoCtrlAllCmd(NOCONTROL,NOCONTROL,NOCONTROL,NOCONTROL,NOCONTROL,NOCONTROL,NOCONTROL,NOCONTROL,NOCONTROL,NOCONTROL,NOCONTROL,NOCONTROL,NOCONTROL,NOCONTROL, NOCONTROL,NOCONTROL);
	  
	  //exit(1);
	  
	}

void arm_player::cmdArmReceived(const drrobot_h20_arm_player::ArmCmd::ConstPtr& cmd_arm)
{
		int right_arm_finger1 = cmd_arm->right_arm[0];
		int right_arm_finger2 = cmd_arm->right_arm[1];
		int right_arm_wrist_x_revolute = cmd_arm->right_arm[2];
		int right_arm_wrist_z_revolute = cmd_arm->right_arm[3];
		int right_lower_arm = cmd_arm->right_arm[4];
		int right_upper_arm_z_revolute = cmd_arm->right_arm[5];
		int right_upper_arm_x_revolute = cmd_arm->right_arm[6];
		int right_shoulder = cmd_arm->right_arm[7];
		int left_arm_finger1 = cmd_arm->left_arm[0];
		int left_arm_finger2 = cmd_arm->left_arm[1];
		int left_arm_wrist_x_revolute = cmd_arm->left_arm[2];
		int left_arm_wrist_z_revolute = cmd_arm->left_arm[3];
		int left_lower_arm = cmd_arm->left_arm[4];
		int left_upper_arm_z_revolute = cmd_arm->left_arm[5];
		int left_upper_arm_x_revolute = cmd_arm->left_arm[6];
		int left_shoulder = cmd_arm->left_arm[7];

		int _vel = cmd_arm->vel;
		bool _speed = cmd_arm->speed;

	this->sendServoCtrlAllCmd(right_arm_finger1, right_arm_finger2, right_arm_wrist_x_revolute, right_arm_wrist_z_revolute, 
	    right_lower_arm, right_upper_arm_z_revolute, right_upper_arm_x_revolute, right_shoulder,
			left_arm_finger1, left_arm_finger2, left_arm_wrist_x_revolute, left_arm_wrist_z_revolute, left_lower_arm, 
	    left_upper_arm_z_revolute, left_upper_arm_x_revolute, left_shoulder,_vel,_speed);

}

int main(int argc , char *argv[])
{

  ros::init(argc, argv, "Trajectory_Subscriber");
  ros::NodeHandle n;
  ros::Rate loop_rate(3);
  
  //Connect to Tangy's arm hardware
	arm_player client;
	std::string ip = "192.168.0.94";
	
	client.openConnection(ip,10001);
	int celeb_pos_s7;
	int celeb_pos_s6;
	int celeb_pos_s14;
	int celeb_pos_s15;
	int rest_pos_s0=client.rest_pos_s0;
	int rest_pos_s1=client.rest_pos_s1;
	int rest_pos_s2=client.rest_pos_s2;
	int rest_pos_s3=client.rest_pos_s3;
	int rest_pos_s4=client.rest_pos_s4;
	int rest_pos_s5=client.rest_pos_s5;
	int rest_pos_s6=client.rest_pos_s6;
	int rest_pos_s7=client.rest_pos_s7;
	int rest_pos_s8=client.rest_pos_s8;
	int rest_pos_s9=client.rest_pos_s9;
	int rest_pos_s10=client.rest_pos_s10;
	int rest_pos_s11=client.rest_pos_s11;
	int rest_pos_s12=client.rest_pos_s12;
	int rest_pos_s13=client.rest_pos_s13;
	int rest_pos_s14=client.rest_pos_s14;
	int rest_pos_s15=client.rest_pos_s15;
	
	ros::Publisher feedback_pub = n.advertise<std_msgs::String>("feedback", 10);
	std_msgs::String msg;
	
	//Start controller manager
	controller_manager::ControllerManager tangy_moveit_controller_manager(&client,n);
	tangy_moveit_controller_manager.loadController("r_arm_controller_1");
	tangy_moveit_controller_manager.loadController("r_arm_controller_2");
	tangy_moveit_controller_manager.loadController("r_arm_controller_3");
	tangy_moveit_controller_manager.loadController("r_arm_controller_4");
	tangy_moveit_controller_manager.loadController("r_arm_controller_5");
	tangy_moveit_controller_manager.loadController("r_arm_controller_6");
	tangy_moveit_controller_manager.loadController("l_arm_controller_1");
	tangy_moveit_controller_manager.loadController("l_arm_controller_2");
	tangy_moveit_controller_manager.loadController("l_arm_controller_3");
	tangy_moveit_controller_manager.loadController("l_arm_controller_4");
	tangy_moveit_controller_manager.loadController("l_arm_controller_5");
	tangy_moveit_controller_manager.loadController("l_arm_controller_6");

    //Subscribe to each individual joint's trajectory goal topic and call the function trajCallback
  ros::Subscriber traj_sub_0=n.subscribe("r_arm_controller_1/follow_joint_trajectory/goal",10, &arm_player::trajCallback0, &client);
  ros::Subscriber traj_sub_1=n.subscribe("r_arm_controller_2/follow_joint_trajectory/goal",10, &arm_player::trajCallback1, &client);
  ros::Subscriber traj_sub_2=n.subscribe("r_arm_controller_3/follow_joint_trajectory/goal",10, &arm_player::trajCallback2, &client);
  ros::Subscriber traj_sub_3=n.subscribe("r_arm_controller_4/follow_joint_trajectory/goal",10, &arm_player::trajCallback3, &client);
  ros::Subscriber traj_sub_4=n.subscribe("r_arm_controller_5/follow_joint_trajectory/goal",10, &arm_player::trajCallback4, &client);
  ros::Subscriber traj_sub_5=n.subscribe("r_arm_controller_6/follow_joint_trajectory/goal",10, &arm_player::trajCallback5, &client);
  ros::Subscriber traj_sub_6=n.subscribe("l_arm_controller_1/follow_joint_trajectory/goal",10, &arm_player::trajCallback6, &client);
  ros::Subscriber traj_sub_7=n.subscribe("l_arm_controller_2/follow_joint_trajectory/goal",10, &arm_player::trajCallback7, &client);
  ros::Subscriber traj_sub_8=n.subscribe("l_arm_controller_3/follow_joint_trajectory/goal",10, &arm_player::trajCallback8, &client);
  ros::Subscriber traj_sub_9=n.subscribe("l_arm_controller_4/follow_joint_trajectory/goal",10, &arm_player::trajCallback9, &client);
  ros::Subscriber traj_sub_10=n.subscribe("l_arm_controller_5/follow_joint_trajectory/goal",10, &arm_player::trajCallback10, &client);
  ros::Subscriber traj_sub_11=n.subscribe("l_arm_controller_6/follow_joint_trajectory/goal",10, &arm_player::trajCallback11, &client);
 
  ros::Subscriber right_joint_sub = n.subscribe("r_arm_joint_cmds",10, &arm_player::jointCb_right, &client);
  ros::Subscriber left_joint_sub = n.subscribe("l_arm_joint_cmds",10, &arm_player::jointCb_left, &client);
  ros::Subscriber start_arm_sub = n.subscribe("start_arms", 10, &arm_player::start_arm_cb, &client);

  ros::Subscriber cmd_arm_sub = n.subscribe<drrobot_h20_arm_player::ArmCmd>("cmd_arm", 1, &arm_player::cmdArmReceived, &client);



  ROS_INFO("Subscribed!");
  client.sendServoCtrlAllCmd(rest_pos_s0 ,rest_pos_s1, rest_pos_s2, rest_pos_s3, rest_pos_s4, rest_pos_s5, rest_pos_s6, rest_pos_s7, rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, rest_pos_s14, rest_pos_s15);
  ROS_INFO("Sending to home position now!");
    
  //curr_pos stores the current position of the arm and is initialized at the home position
  int myints[]={rest_pos_s0 ,rest_pos_s1, rest_pos_s2, rest_pos_s3, rest_pos_s4, rest_pos_s5, rest_pos_s6, rest_pos_s7, rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, rest_pos_s14, rest_pos_s15};
  std::vector<int> curr_pos(myints, myints+sizeof(myints)/sizeof(int));

  //Supporting variables (defined more in depth later)
  int num_points;
  int a;
  bool the_same=1;

  while(ros::ok()){

    tangy_moveit_controller_manager.update(ros::Time::now(), ros::Duration(1), 0);
    
    //Execute servo commands
    num_points=std::max(client.servo_goals_0.size(), client.servo_goals_6.size());
    a=num_points;                   //Dummy variable to de-clutter the initialization of final_pos
    if(num_points!=0){
      
      //Check latest motion plan and store first point in start_pos and last point in final_pos
      //Firstly, check to make sure all servo_goals_# fields are populated using move_group_checker
      client.move_group_checker(curr_pos);
      int myints3[]={rest_pos_s0,rest_pos_s1, client.servo_goals_0.at(0), client.servo_goals_1.at(0), client.servo_goals_2.at(0), client.servo_goals_3.at(0), client.servo_goals_4.at(0), client.servo_goals_5.at(0), rest_pos_s8, rest_pos_s9, client.servo_goals_6.at(0), client.servo_goals_7.at(0), client.servo_goals_8.at(0), client.servo_goals_9.at(0), client.servo_goals_10.at(0), client.servo_goals_11.at(0)};
      std::vector<int> start_pos(myints3, myints3+sizeof(myints3)/sizeof(int));
      int myints2[]={rest_pos_s0,rest_pos_s1, client.servo_goals_0.at(a-1), client.servo_goals_1.at(a-1), client.servo_goals_2.at(a-1), client.servo_goals_3.at(a-1), client.servo_goals_4.at(a-1), client.servo_goals_5.at(a-1), rest_pos_s8, rest_pos_s9, client.servo_goals_6.at(a-1), client.servo_goals_7.at(a-1), client.servo_goals_8.at(a-1), client.servo_goals_9.at(a-1), client.servo_goals_10.at(a-1), client.servo_goals_11.at(a-1)};
      std::vector<int> final_pos(myints2, myints2+sizeof(myints2)/sizeof(int));
      
      msg.data="Executing plan";
      feedback_pub.publish(msg);
      
      //Execute trajectory according to each point i in the motion plan
      for(int i=0; i< num_points; i++)
      {
        ROS_INFO("Executing movement point %u",i);
        //Check to see if current arm position is at the final position of the trajectory goal
        the_same=1;
        for(int k=0; k<16; k++)
        {
          if(curr_pos[k]!=final_pos[k]){
            the_same=0;
          }
        }
        //If the current position is not at the end goal state yet, move the arm to the next point in the motion plan
        if(!the_same)
        {
          //Execute next point in motion plan
          client.sendServoCtrlAllCmd(rest_pos_s0, rest_pos_s1, client.servo_goals_0.at(i), client.servo_goals_1.at(i), client.servo_goals_2.at(i), client.servo_goals_3.at(i), client.servo_goals_4.at(i), client.servo_goals_5.at(i), rest_pos_s8, rest_pos_s9, client.servo_goals_6.at(i), client.servo_goals_7.at(i), client.servo_goals_8.at(i), client.servo_goals_9.at(i), client.servo_goals_10.at(i), client.servo_goals_11.at(i));
          
          //Store current position of arm
          int c_pos[]={rest_pos_s0,rest_pos_s1, client.servo_goals_0.at(i), client.servo_goals_1.at(i), client.servo_goals_2.at(i), client.servo_goals_3.at(i), client.servo_goals_4.at(i), client.servo_goals_5.at(i), rest_pos_s8, rest_pos_s9, client.servo_goals_6.at(i), client.servo_goals_7.at(i), client.servo_goals_8.at(i), client.servo_goals_9.at(i), client.servo_goals_10.at(i), client.servo_goals_11.at(i)};
          std::vector<int> c_p(c_pos, c_pos+sizeof(c_pos)/sizeof(int));
          curr_pos=c_p;
          
          //Wait for a little bit to ensure the servos have moved to their designated positions
          usleep(110000);
        }
      }
      
      
      //Wave/celebrate goodbye hack
      if(abs(final_pos[2]-rest_pos_s2)<10 && abs(final_pos[3]-rest_pos_s3+230)<10 && abs(final_pos[4]-rest_pos_s4-290)<10 && abs(final_pos[5]-rest_pos_s5+280)<10 && abs(final_pos[6]-rest_pos_s6+680)<10 && abs(final_pos[7]-rest_pos_s7-1283)<10)
      {
        //1850 1240 1639 1302 2005 1699 1581 2063
        //1230 1100 2165 1525 1379 1950 1260 2400
        usleep(1800000);
        ROS_INFO("Executing wave goodbye behavior");
        client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,1302,2005,1720,1601,2063,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, rest_pos_s14, rest_pos_s15);
        usleep(100000);
        client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,1302,1800,1720,1601,2063,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, rest_pos_s14, rest_pos_s15);
        usleep(1350000);
        client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,1302,2050,1720,1601,2063,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, rest_pos_s14, rest_pos_s15);
        usleep(1350000);
        client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,1302,1800,1720,1601,2063,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, rest_pos_s14, rest_pos_s15);
        usleep(1350000);
        client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,1302,2050,1720,1601,2063,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, rest_pos_s14, rest_pos_s15);
        usleep(1350000);
        client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,1302,2005,1720,1601,2063,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, rest_pos_s14, rest_pos_s15);
        usleep(270000);
      }else if(abs(final_pos[10]-rest_pos_s10)<10 && abs(final_pos[11]-rest_pos_s11)<10 && abs(final_pos[12]-rest_pos_s12)<10 && abs(final_pos[13]-rest_pos_s13)<10 && abs(final_pos[14]-rest_pos_s14-100)<10 && abs(final_pos[15]-rest_pos_s15+1577)<10)
      {
        // 1850 1240 1639 1535 1715 1980 2207 2384
        //1230 1100 2165 1525 1379 1950 1363 823
        usleep(1800000);
        celeb_pos_s6=final_pos[6];
        celeb_pos_s7=final_pos[7];
        celeb_pos_s14=final_pos[14];
        celeb_pos_s15=final_pos[15];
        ROS_INFO("Executing celebrate behavior");
        client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,rest_pos_s3,rest_pos_s4,rest_pos_s5,celeb_pos_s6,celeb_pos_s7,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, celeb_pos_s14,  celeb_pos_s15);
        usleep(500000);
	client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,rest_pos_s3,rest_pos_s4,rest_pos_s5,celeb_pos_s6+200,celeb_pos_s7,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, celeb_pos_s14+200,  celeb_pos_s15);
        usleep(1350000);
        client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,rest_pos_s3,rest_pos_s4,rest_pos_s5,celeb_pos_s6-200,celeb_pos_s7,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, celeb_pos_s14-200, celeb_pos_s15);
        usleep(1350000);
        client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,rest_pos_s3,rest_pos_s4,rest_pos_s5,celeb_pos_s6+200,celeb_pos_s7,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, celeb_pos_s14+200,  celeb_pos_s15);
        usleep(1350000);
        client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,rest_pos_s3,rest_pos_s4,rest_pos_s5,celeb_pos_s6-200,celeb_pos_s7,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, celeb_pos_s14-200, celeb_pos_s15);
        usleep(1350000);
        client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,rest_pos_s3,rest_pos_s4,rest_pos_s5,celeb_pos_s6+200,celeb_pos_s7,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, celeb_pos_s14+200,  celeb_pos_s15);
        usleep(1350000);
        client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,rest_pos_s3,rest_pos_s4,rest_pos_s5,celeb_pos_s6,celeb_pos_s7,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, celeb_pos_s14,  celeb_pos_s15);
        // usleep(1000000);
        // client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,rest_pos_s3,rest_pos_s4,rest_pos_s5,2227,2584,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, 753, 1023);
        // usleep(1000000);
        // client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,rest_pos_s3,rest_pos_s4,rest_pos_s5,2277,2184,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, 753,  623);
        // usleep(1000000);
        // client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,rest_pos_s3,rest_pos_s4,rest_pos_s5,2227,2584,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, 753, 1073);
        // usleep(1000000);
        // client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,rest_pos_s3,rest_pos_s4,rest_pos_s5,2277,2184,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, 753,  623);
        // usleep(1000000);
        // client.sendServoCtrlAllCmd(rest_pos_s0,rest_pos_s1,rest_pos_s2,rest_pos_s3,rest_pos_s4,rest_pos_s5,2227,2384,rest_pos_s8, rest_pos_s9, rest_pos_s10, rest_pos_s11, rest_pos_s12, rest_pos_s13, 753,  823);
        usleep(1350000);
      } else {
        if(num_points<20){
          usleep(180000);
        }else{
          usleep(2700000);
        }
        
      }

      //Reverse execute trajectory plan
      for(int i=num_points-1; i>=0; i--)
      {
        ROS_INFO("Executing reverse movement point %u",i);
        //Check to see if current arm position is at the final position of the trajectory goal
        the_same=1;
        for(int k=0; k<16; k++)
        {
          if(curr_pos[k]!=start_pos[k]){
            the_same=0;
          }
        }
        //If the current position is not at the end goal state yet, move the arm to the next point in the motion plan
        if(!the_same)
        {
          //Execute next point in motion plan
          client.sendServoCtrlAllCmd(rest_pos_s0, rest_pos_s1, client.servo_goals_0.at(i), client.servo_goals_1.at(i), client.servo_goals_2.at(i), client.servo_goals_3.at(i), client.servo_goals_4.at(i), client.servo_goals_5.at(i), rest_pos_s8, rest_pos_s9, client.servo_goals_6.at(i), client.servo_goals_7.at(i), client.servo_goals_8.at(i), client.servo_goals_9.at(i), client.servo_goals_10.at(i), client.servo_goals_11.at(i));
          
          //Store current position of arm
          int c_pos[]={rest_pos_s0,rest_pos_s1, client.servo_goals_0.at(i), client.servo_goals_1.at(i), client.servo_goals_2.at(i), client.servo_goals_3.at(i), client.servo_goals_4.at(i), client.servo_goals_5.at(i), rest_pos_s8, rest_pos_s9, client.servo_goals_6.at(i), client.servo_goals_7.at(i), client.servo_goals_8.at(i), client.servo_goals_9.at(i), client.servo_goals_10.at(i), client.servo_goals_11.at(i)};
          std::vector<int> c_p(c_pos, c_pos+sizeof(c_pos)/sizeof(int));
          curr_pos=c_p;
          
          //Wait for a little bit to ensure the servos have moved to their designated positions
          usleep(110000);
        }

      }
      msg.data="Finished executing plan";
      sleep(3);
      feedback_pub.publish(msg);
    }
    
    //Clear the motion plans (which have just been completed) from the execution queue
    client.servo_goals_0.clear();
    client.servo_goals_1.clear();
    client.servo_goals_2.clear();
    client.servo_goals_3.clear();
    client.servo_goals_4.clear();
    client.servo_goals_5.clear();
    client.servo_goals_6.clear();
    client.servo_goals_7.clear();
    client.servo_goals_8.clear();
    client.servo_goals_9.clear();
    client.servo_goals_10.clear();
    client.servo_goals_11.clear();
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
