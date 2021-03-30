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
#include "arm_player_pos.hpp"
#include "controller_manager/controller_manager.h"
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include <ros/console.h>
#include <controller_manager/controller_loader.h>
#include <controller_manager_msgs/ControllerState.h>

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

    hardware_interface::JointHandle pos_handle_0(jnt_state_interface.getHandle("lower_to_wrist_right_arm"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_0);
    hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("wrist_right_arm_z_revolute_joint"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_1);
    hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("upper_to_lower_right_joint"), &cmd[2]);
    jnt_pos_interface.registerHandle(pos_handle_2);
    hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("upper_right_arm_z_revolute_joint"), &cmd[3]);
    jnt_pos_interface.registerHandle(pos_handle_3);
    hardware_interface::JointHandle pos_handle_4(jnt_state_interface.getHandle("upper_right_arm_y_revolute_joint"), &cmd[4]);
    jnt_pos_interface.registerHandle(pos_handle_4);
    hardware_interface::JointHandle pos_handle_5(jnt_state_interface.getHandle("right_shoulder_x_revolute"), &cmd[5]);
    jnt_pos_interface.registerHandle(pos_handle_5);
    hardware_interface::JointHandle pos_handle_6(jnt_state_interface.getHandle("lower_to_wrist_left_arm"), &cmd[6]);
    jnt_pos_interface.registerHandle(pos_handle_6);
    hardware_interface::JointHandle pos_handle_7(jnt_state_interface.getHandle("wrist_left_arm_z_revolute_joint"), &cmd[7]);
    jnt_pos_interface.registerHandle(pos_handle_7);
    hardware_interface::JointHandle pos_handle_8(jnt_state_interface.getHandle("upper_to_lower_left_joint"), &cmd[8]);
    jnt_pos_interface.registerHandle(pos_handle_8);
    hardware_interface::JointHandle pos_handle_9(jnt_state_interface.getHandle("upper_left_arm_z_revolute_joint"), &cmd[9]);
    jnt_pos_interface.registerHandle(pos_handle_9);
    hardware_interface::JointHandle pos_handle_10(jnt_state_interface.getHandle("upper_left_arm_y_revolute_joint"), &cmd[10]);
    jnt_pos_interface.registerHandle(pos_handle_10);
    hardware_interface::JointHandle pos_handle_11(jnt_state_interface.getHandle("left_shoulder_x_revolute"), &cmd[11]);
    jnt_pos_interface.registerHandle(pos_handle_11);
    
  	this->registerInterface(&jnt_pos_interface);
		
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
	
	//Servos channel starts with 0 and goes to 15, representing all 16 servos.
	//Each servo has its own angle limitation.

	/*int arm_player::sendServoCtrlAllCmd(int cmd1, int cmd2, int cmd3, int cmd4, int cmd5, int cmd6, int cmd7, int cmd8, int cmd9, int cmd10, int cmd11, int cmd12, int cmd13, int cmd14, int cmd15, int cmd16, int vel, bool speed)
	{
		int poses[18];
		int i;
		
		if (cmd1 != NOCONTROL)
		{
			if (cmd1 > 2110) cmd1 = 2110;
			if (cmd1 < 590) cmd1 = 590;
		}
		if (cmd2 != NOCONTROL)
		{
			if (cmd2 > 2400) cmd2 = 2400;
			if (cmd2 < 950) cmd2 = 950;
		}
		if (cmd3 != NOCONTROL)
		{
			if (cmd3 > 2250) cmd3 = 2250;
			if (cmd3 < 680) cmd3 = 680;
		}
		if (cmd4 != NOCONTROL)
		{
			if (cmd4 > 2350) cmd4 = 2350;
			if (cmd4 < 580) cmd4 = 580;
		}
		if (cmd5 != NOCONTROL)
		{
			if (cmd5 > 2110) cmd5 = 2110;
			if (cmd5 < 1400) cmd5 = 1400;
		}
		if (cmd6 != NOCONTROL)
		{
			if (cmd6 > 2350) cmd6 = 2350;
			if (cmd6 < 1670) cmd6 = 1670;
		}
		if (cmd7 != NOCONTROL)
		{
			if (cmd7 > 2400) cmd7 = 2400;
			if (cmd7 < 1460) cmd7 = 1460;
		}
		if (cmd8 != NOCONTROL)
		{
			if (cmd8 > 2450) cmd8 = 2450;
			if (cmd8 < 525) cmd8 = 525;
		}
		if (cmd9 != NOCONTROL)
		{
			if (cmd9 > 2380) cmd9 = 2380;
			if (cmd9 < 960) cmd9 = 960;
		}
		if (cmd10 != NOCONTROL)
		{
			if (cmd10 > 2015) cmd10 = 2015;
			if (cmd10 < 565) cmd10 = 565;
		}
		if (cmd11 != NOCONTROL)
		{	
			if (cmd11 > 2200) cmd11 = 2200;
			if (cmd11 < 650) cmd11 = 650;
		}
		if (cmd12 != NOCONTROL)
		{
			if (cmd12 > 2430) cmd12 = 2430;
			if (cmd12 < 580) cmd12 = 580;
		}
		if (cmd13 != NOCONTROL)
		{
			if (cmd13 > 2255) cmd13 = 2255;
			if (cmd13 < 1510) cmd13 = 1510;
		}
		if (cmd14 != NOCONTROL)
		{
			if (cmd14 > 2350) cmd14 = 2350;
			if (cmd14 < 1670) cmd14 = 1670;
		}
		if (cmd15 != NOCONTROL)
		{
			if (cmd15 > 2015) cmd15 = 2015;
			if (cmd15 < 1170) cmd15 = 1170;
		}
		if (cmd16 != NOCONTROL)
		{
			if (cmd16 > 2450) cmd16 = 2450;
			if (cmd16 < 560) cmd16 = 560;
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
	}*/

	int arm_player::sendServoCtrlAllCmd(int cmd1, int cmd2, int cmd3, int cmd4, int cmd5, int cmd6, int cmd7, int cmd8, int cmd9, int cmd10, int cmd11, int cmd12, int cmd13, int cmd14, int cmd15, int cmd16)
	{
		int poses[16];
		int i;

		if (cmd1 != NOCONTROL)
		{
			if (cmd1 > 2110) cmd1 = 2110;
			if (cmd1 < 590) cmd1 = 590;
		}
		if (cmd2 != NOCONTROL)
		{
			if (cmd2 > 2400) cmd2 = 2400;
			if (cmd2 < 950) cmd2 = 950;
		}
		if (cmd3 != NOCONTROL)
		{
			if (cmd3 > 2250) cmd3 = 2250;
			if (cmd3 < 680) cmd3 = 680;
		}
		if (cmd4 != NOCONTROL)
		{
			if (cmd4 > 2350) cmd4 = 2350;
			if (cmd4 < 580) cmd4 = 580;
		}
		if (cmd5 != NOCONTROL)
		{
			if (cmd5 > 2110) cmd5 = 2110;
			if (cmd5 < 1400) cmd5 = 1400;
		}
		if (cmd6 != NOCONTROL)
		{
			if (cmd6 > 2350) cmd6 = 2350;
			if (cmd6 < 1670) cmd6 = 1670;
		}
		if (cmd7 != NOCONTROL)
		{
			if (cmd7 > 2400) cmd7 = 2400;
			if (cmd7 < 1460) cmd7 = 1460;
		}
		if (cmd8 != NOCONTROL)
		{
			if (cmd8 > 2450) cmd8 = 2450;
			if (cmd8 < 525) cmd8 = 525;
		}
		if (cmd9 != NOCONTROL)
		{
			if (cmd9 > 2380) cmd9 = 2380;
			if (cmd9 < 960) cmd9 = 960;
		}
		if (cmd10 != NOCONTROL)
		{
			if (cmd10 > 2015) cmd10 = 2015;
			if (cmd10 < 565) cmd10 = 565;
		}
		if (cmd11 != NOCONTROL)
		{	
			if (cmd11 > 2200) cmd11 = 2200;
			if (cmd11 < 650) cmd11 = 650;
		}
		if (cmd12 != NOCONTROL)
		{
			if (cmd12 > 2430) cmd12 = 2430;
			if (cmd12 < 580) cmd12 = 580;
		}
		if (cmd13 != NOCONTROL)
		{
			if (cmd13 > 2255) cmd13 = 2255;
			if (cmd13 < 1510) cmd13 = 1510;
		}
		if (cmd14 != NOCONTROL)
		{
			if (cmd14 > 2350) cmd14 = 2350;
			if (cmd14 < 1670) cmd14 = 1670;
		}
		if (cmd15 != NOCONTROL)
		{
			if (cmd15 > 2015) cmd15 = 2015;
			if (cmd15 < 1170) cmd15 = 1170;
		}
		if (cmd16 != NOCONTROL)
		{
			if (cmd16 > 2450) cmd16 = 2450;
			if (cmd16 < 560) cmd16 = 560;
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
		}

		return sendCommand(poses, i, 2);
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
			message += "T1000\r";
		}
		
		ROS_INFO("Sending commands...");
		ROS_INFO("right_arm [%d %d %d %d %d %d %d %d]", poses[0], poses[1], poses[2], poses[3], poses[4], poses[5], poses[6], poses[7]);
		ROS_INFO("left_arm [%d %d %d %d %d %d %d %d]",poses[8], poses[9], poses[10], poses[11], poses[12], poses[13], poses[14], poses[15]);
		if (mode == 1) ROS_INFO("Time/Speed [%d]\n", poses[16]);
		else ROS_INFO("Time/Speed [1000]\n");
		
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
	void arm_player::trajCallback0(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_0.assign(num_points,0);
	  for(int i=0; i<num_points; i++){
	    servo_goals_0.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-621.48)+1608.7;
	    ROS_INFO("Callback was called!");
    }
	}
	
	void arm_player::trajCallback1(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_1.assign(num_points,0);	  
	  for(int i=0; i<num_points; i++){
	    servo_goals_1.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-563.409)+1491.7;
    }
	}
	
	void arm_player::trajCallback2(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_2.assign(num_points,0);	  
	  for(int i=0; i<num_points; i++){
	    servo_goals_2.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-185.19)+1698;
    }
	}
	
	void arm_player::trajCallback3(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_3.assign(num_points,0);	  
	  for(int i=0; i<num_points; i++){
	    servo_goals_3.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(185.529)+2006.7;
    }
	}
	
	void arm_player::trajCallback4(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_4.assign(num_points,0);	  
	  for(int i=0; i<num_points; i++){
	    servo_goals_4.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(454.3)+2330.8;
    }
	}
	
	void arm_player::trajCallback5(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_5.assign(num_points,0);	  
	  for(int i=0; i<num_points; i++){
	    servo_goals_5.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(513.86)+730.81;
    }
	}
	
	void arm_player::trajCallback6(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_6.assign(num_points,0);	  
	  for(int i=0; i<num_points; i++){
	    servo_goals_6.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-612.98)+1562.5;
    }
	}
	
	void arm_player::trajCallback7(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_7.assign(num_points,0);	  
	  for(int i=0; i<num_points; i++){
	    servo_goals_7.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(588.873)+1510;
    }
	}
	
	void arm_player::trajCallback8(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_8.assign(num_points,0);	  
	  for(int i=0; i<num_points; i++){
	    servo_goals_8.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(194.91)+1945.8;
    }
	}
	
	void arm_player::trajCallback9(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_9.assign(num_points,0);	  
	  for(int i=0; i<num_points; i++){
	    servo_goals_9.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-185.53)+2010;
    }
	}
	
	void arm_player::trajCallback10(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_10.assign(num_points,0);	  
	  for(int i=0; i<num_points; i++){
	    servo_goals_10.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-413.11)+1222.6;
    }
	}
	
	void arm_player::trajCallback11(control_msgs::FollowJointTrajectoryActionGoal traj_goal)
	{
	  int num_points=traj_goal.goal.trajectory.points.size();
	  servo_goals_11.assign(num_points,0);	  
	  for(int i=0; i<num_points; i++){
	    servo_goals_11.at(i)=traj_goal.goal.trajectory.points[i].positions[0]*(-502.2)+2239.2;
    }
	}
	
	//Checks whether the right or the left arm is being moved, and assigns default values to the other arm's servos
	void arm_player::move_group_checker()
	{
	  ROS_INFO("move_group_checker called");
	  int num_points;
	  if(servo_goals_0.empty())
	  {
	    num_points=servo_goals_6.size();
	    servo_goals_0.assign(num_points, 1625);
	    servo_goals_1.assign(num_points, 1545);
	    servo_goals_2.assign(num_points, 1710);
	    servo_goals_3.assign(num_points, 2000);
	    servo_goals_4.assign(num_points, 2200);
	    servo_goals_5.assign(num_points, 760);
	  }
	  if(servo_goals_6.empty())
	  {
	    num_points=servo_goals_0.size();
	    servo_goals_6.assign(num_points, 1570);
	    servo_goals_7.assign(num_points, 1520);
	    servo_goals_8.assign(num_points, 1940);
	    servo_goals_9.assign(num_points, 2010);
	    servo_goals_10.assign(num_points, 1240);
	    servo_goals_11.assign(num_points, 2200); 
	  }
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
  
  ROS_INFO("Subscribed!");
  client.sendServoCtrlAllCmd(1700 ,1269, 1625, 2350, 1710, 2000, 2200, 760, 1290, 1720, 1570, 1520, 1940, 2010, 1240, 2200);
  ROS_INFO("Sending to home position now!");
    
  //curr_pos stores the current position of the arm and is initialized at the home position
  int myints[]={1700 ,1269, 1625, 1545, 1710, 2000, 2200, 760, 1290, 1720, 1570, 1520, 1940, 2010, 1240, 2200};
  std::vector<int> curr_pos(myints, myints+sizeof(myints)/sizeof(int));

  //Supporting variables (defined more in depth later)
  int num_points;
  int a;
  bool the_same=1;

  while(ros::ok()){

    tangy_moveit_controller_manager.update(ros::Time::now(), ros::Duration(1), 0);
    
    //Execute servo commands
    num_points=client.servo_goals_0.size();
    a=num_points;                   //Dummy variable to de-clutter the initialization of final_pos
    ROS_INFO("Number of points in the motion plan= %u", a);
    if(num_points!=0){
      
      //Check latest motion plan and store last point in final_pos
      //Firstly, check to make sure all servo_goals_# fields are populated using move_group_checker
      client.move_group_checker();
    
      int myints2[]={1700,1269, client.servo_goals_0.at(a-1), client.servo_goals_1.at(a-1), client.servo_goals_2.at(a-1), client.servo_goals_3.at(a-1), client.servo_goals_4.at(a-1), client.servo_goals_5.at(a-1), 1290, 1720, client.servo_goals_6.at(a-1), client.servo_goals_7.at(a-1), client.servo_goals_8.at(a-1), client.servo_goals_9.at(a-1), client.servo_goals_10.at(a-1), client.servo_goals_11.at(a-1)};
      std::vector<int> final_pos(myints2, myints2+sizeof(myints2)/sizeof(int));
    
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
          client.sendServoCtrlAllCmd(1700, 1269, client.servo_goals_0.at(i), client.servo_goals_1.at(i), client.servo_goals_2.at(i), client.servo_goals_3.at(i), client.servo_goals_4.at(i), client.servo_goals_5.at(i), 1290, 1720, client.servo_goals_6.at(i), client.servo_goals_7.at(i), client.servo_goals_8.at(i), client.servo_goals_9.at(i), client.servo_goals_10.at(i), client.servo_goals_11.at(i));
          
          //Store current position of arm
          int c_pos[]={1700,1269, client.servo_goals_0.at(i), client.servo_goals_1.at(i), client.servo_goals_2.at(i), client.servo_goals_3.at(i), client.servo_goals_4.at(i), client.servo_goals_5.at(i), 1290, 1720, client.servo_goals_6.at(i), client.servo_goals_7.at(i), client.servo_goals_8.at(i), client.servo_goals_9.at(i), client.servo_goals_10.at(i), client.servo_goals_11.at(i)};
          std::vector<int> c_p(c_pos, c_pos+sizeof(c_pos)/sizeof(int));
          curr_pos=c_p;
          
          //Wait for a little bit to ensure the servos have moved to their designated positions
          sleep(0.4);
        }
      }
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
