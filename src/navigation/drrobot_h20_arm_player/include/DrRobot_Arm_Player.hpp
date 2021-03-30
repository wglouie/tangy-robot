//============================================================================
// Name        : DrRobot_Arm_Player.cpp
// Author      : Ananias Paiva
// Version     :
// Copyright   : Your copyright notice
// Description : Arms player
//============================================================================

#ifndef DRROBOT_ARM_PLAYER_HPP_
#define DRROBOT_ARM_PLAYER_HPP_

#include <iostream>
#include <math.h>
#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include "DrRobotMotionArmDriver.hpp"

#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <drrobot_h20_arm_player/ArmCmd.h>


using namespace std;

class DrRobot_Arm_Player
{
public:

	ros::Subscriber cmd_arm_sub;
	ros::NodeHandle node;
	
	DrRobot_Arm_Player();
	~DrRobot_Arm_Player();
	
	int start();
	int stop();
	
	void cmdArmReceived(const drrobot_h20_arm_player::ArmCmd::ConstPtr& cmd_arm);
	
	void doUpdate();
	
private:
	
	DrRobotMotionArmDriver *_drrobotmotionarmdriver;
	struct DrRobotMotionArmDriver::DrRobotArmConfig _robotConfig ;
	
	ros::Publisher armPose_pub;
	drrobot_h20_arm_player::ArmCmd pose_arm;

	int right_arm_finger1;
	int right_arm_finger2;
	int right_arm_wrist_x_revolute;
	int right_arm_wrist_z_revolute;
	int right_lower_arm;
	int right_upper_arm_z_revolute;
	int right_upper_arm_x_revolute;
	int right_shoulder;
	int left_arm_finger1;
	int left_arm_finger2;
	int left_arm_wrist_x_revolute;
	int left_arm_wrist_z_revolute;
	int left_lower_arm;
	int left_upper_arm_z_revolute;
	int left_upper_arm_x_revolute;
	int left_shoulder;

	int _vel;
	bool _speed;

	int open;

};

#endif
