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
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <drrobot_h20_arm_player/joint_cmd.h>
#include <drrobot_h20_arm_player/arm_cmd.h>
#include <drrobot_h20_arm_player/ArmCmd.h>

#define DEFAULT_PORT 8888
#define NOCONTROL -32768
#define SERVO_CHANNEL_OFFSET 8

using namespace std;


// This is bad and should be fixed
const static int min_pos[16] = {590, 950,	680, 580, 1400, 1670, 1710, 680, 1050, 400, 1780, 1090, 600, 1030, 1170, 700};
const static int init_pos[16] = {1850, 1240, 1640, 1535, 1715, 1980, 2500, 780, 1270, 1100, 2165, 1525, 1380, 1950, 1265, 2400};
const static int max_pos[16] = {2110, 2400, 2250, 2350, 2110, 2350, 2800, 2490, 2380, 1550, 2460, 1835, 2450, 2580, 2015, 2650};

class arm_player: public hardware_interface::RobotHW
{
public:
	struct DrRobotArmConfig
		{
			std::string robotID;
			std::string robotIP;
			int portNum;
			
		};
	
		const static int rest_pos_s0=1850;      //finger
	  const static int rest_pos_s1=1240;      //finger
	  const static int rest_pos_s2=1640;
	  const static int rest_pos_s3=1535;
	  const static int rest_pos_s4=1715;
	  const static int rest_pos_s5=1980;
	  const static int rest_pos_s6=2500;
	  const static int rest_pos_s7=780;
	  const static int rest_pos_s8=1270;      //finger
	  const static int rest_pos_s9=1100;      //finger
	  const static int rest_pos_s10=2165;
	  const static int rest_pos_s11=1525;
	  const static int rest_pos_s12=1380;
	  const static int rest_pos_s13=1950;
		const static int rest_pos_s14=1265;
	  const static int rest_pos_s15=2400;
		

    int celeb_pos_s6;
    int celeb_pos_s7;
    int celeb_pos_s14;
	  int celeb_pos_s15;
	
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
  

	void jointCb_right(drrobot_h20_arm_player::arm_cmd goal);
	void jointCb_left(drrobot_h20_arm_player::arm_cmd goal);
	void cmdArmReceived(const drrobot_h20_arm_player::ArmCmd::ConstPtr& cmd_arm);  
    void start_arm_cb(const std_msgs::String::ConstPtr& msg);

  void move_group_checker(std::vector<int> curr_pos);
	void fnExit();


private:
	DrRobotArmConfig *_robotConfig;
	struct sockaddr_in client;
	int sockCheck;
	int connectCheck;
	int msgCheck;
	socklen_t client_len;
	
	hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
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
