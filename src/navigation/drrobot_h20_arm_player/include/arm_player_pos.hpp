//============================================================================
// Name        : arm_player.hpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : 
//============================================================================

#ifndef arm_player_HPP_
#define arm_player_HPP_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>    //strlen
#include <sys/socket.h>    //socket
#include <arpa/inet.h> //inet_addr
#include <netinet/in.h>
#include <unistd.h>    //write
#include <sys/time.h>
#include <sys/types.h>
#include <string>
#include <iostream>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#define DEFAULT_PORT 8888
#define NOCONTROL -32768
#define SERVO_CHANNEL_OFFSET 8

using namespace std;

class arm_player: public hardware_interface::RobotHW
{
public:
	struct DrRobotArmConfig
		{
			std::string robotID;
			std::string robotIP;
			int portNum;
			
		};
	
	arm_player();
	~arm_player();

	std::vector<int> servo_goals_0;
	std::vector<int> servo_goals_1;
	std::vector<int> servo_goals_2;
	std::vector<int> servo_goals_3;
	std::vector<int> servo_goals_4;
	std::vector<int> servo_goals_5;
	std::vector<int> servo_goals_6;
	std::vector<int> servo_goals_7;
	std::vector<int> servo_goals_8;
	std::vector<int> servo_goals_9;
	std::vector<int> servo_goals_10;
	std::vector<int> servo_goals_11;
	
	void close();
	
	int vali_ip(const char*  ip_str);
	
	int openConnection(std::string robotIP, int portNum);
	
	void getDrRobotArmDriverConfig(DrRobotArmConfig *driverConfig);
	
	void setDrRobotArmDriverConfig(DrRobotArmConfig *driverConfig);
	
	int sendServoCtrlAllCmd(int cmd1, int cmd2, int cmd3, int cmd4, int cmd5, int cmd6, int cmd7, int cmd8, int cmd9, int cmd10, int cmd11, int cmd12, int cmd13, int cmd14, int cmd15, int cmd16, int vel, bool speed);
	int sendServoCtrlAllCmd(int cmd1, int cmd2, int cmd3, int cmd4, int cmd5, int cmd6, int cmd7, int cmd8, int cmd9, int cmd10, int cmd11, int cmd12, int cmd13, int cmd14, int cmd15, int cmd16);

	int sendCommand(int poses[], int vecLen, int mode);
	
	int checkStatus();

  void trajCallback0(control_msgs::FollowJointTrajectoryActionGoal traj_goal);
  void trajCallback1(control_msgs::FollowJointTrajectoryActionGoal traj_goal);
  void trajCallback2(control_msgs::FollowJointTrajectoryActionGoal traj_goal);
  void trajCallback3(control_msgs::FollowJointTrajectoryActionGoal traj_goal);
  void trajCallback4(control_msgs::FollowJointTrajectoryActionGoal traj_goal);
  void trajCallback5(control_msgs::FollowJointTrajectoryActionGoal traj_goal);
  void trajCallback6(control_msgs::FollowJointTrajectoryActionGoal traj_goal);
  void trajCallback7(control_msgs::FollowJointTrajectoryActionGoal traj_goal);
  void trajCallback8(control_msgs::FollowJointTrajectoryActionGoal traj_goal);
  void trajCallback9(control_msgs::FollowJointTrajectoryActionGoal traj_goal);
  void trajCallback10(control_msgs::FollowJointTrajectoryActionGoal traj_goal);
  void trajCallback11(control_msgs::FollowJointTrajectoryActionGoal traj_goal);
  
  void move_group_checker();
	
private:
	DrRobotArmConfig *_robotConfig;
	struct sockaddr_in client;
	int sockCheck;
	int connectCheck;
	int msgCheck;
	socklen_t client_len;
	
	hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[12];
  double pos[12];
  double vel[12];
  double eff[12];
	
	std::string to_string(int value)
	{
		stringstream ss;
		ss << value;
		return ss.str();
	}
};

#endif
