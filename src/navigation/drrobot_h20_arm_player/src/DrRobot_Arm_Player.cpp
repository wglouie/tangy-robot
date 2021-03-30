//============================================================================
// Name        : DrRobot_Arm_Player.cpp
// Author      : Ananias Paiva
// Copyright   : Your copyright notice
// Description : Arms player
//============================================================================

#include "DrRobot_Arm_Player.hpp"

DrRobot_Arm_Player::DrRobot_Arm_Player()
{
	_robotConfig.robotIP = "192.168.0.94";
	_robotConfig.portNum = 10001;
	
	right_arm_finger1 = NOCONTROL;
	right_arm_finger2 = NOCONTROL;
	right_arm_wrist_x_revolute = NOCONTROL;
	right_arm_wrist_z_revolute = NOCONTROL;
	right_lower_arm = NOCONTROL;
	right_upper_arm_z_revolute = NOCONTROL;
	right_upper_arm_x_revolute = NOCONTROL;
	right_shoulder = NOCONTROL;
	left_arm_finger1 = NOCONTROL;
	left_arm_finger2 = NOCONTROL;
	left_arm_wrist_x_revolute = NOCONTROL;
	left_arm_wrist_z_revolute = NOCONTROL;
	left_lower_arm = NOCONTROL;
	left_upper_arm_z_revolute = NOCONTROL;
	left_upper_arm_x_revolute = NOCONTROL;
	left_shoulder = NOCONTROL;

	_vel = 1000;
    _speed = false;
    
    _drrobotmotionarmdriver = new DrRobotMotionArmDriver();
}

DrRobot_Arm_Player::~DrRobot_Arm_Player()
{

}

int DrRobot_Arm_Player::start()
{

	if(_drrobotmotionarmdriver->openConnection(_robotConfig.robotIP,_robotConfig.portNum) < 0)
	{
		ROS_INFO("could not open network connection");
		return -1;
	}

	open = 1;

	cmd_arm_sub = node.subscribe<drrobot_h20_arm_player::ArmCmd>("cmd_arm", 1, boost::bind(&DrRobot_Arm_Player::cmdArmReceived, this, _1));

	return(0);
}

int DrRobot_Arm_Player::stop()
{
	if (open == 1)
	{
		_drrobotmotionarmdriver->close();
		_drrobotmotionarmdriver->sendServoCtrlAllCmd(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		return 1;
	}
	else
	{
		ROS_INFO("network connection is not open");
		return -1;
	}

	return 0;
}

void DrRobot_Arm_Player::cmdArmReceived(const drrobot_h20_arm_player::ArmCmd::ConstPtr& cmd_arm)
{
	right_arm_finger1 = cmd_arm->right_arm[0];
	right_arm_finger2 = cmd_arm->right_arm[1];
	right_arm_wrist_x_revolute = cmd_arm->right_arm[2];
	right_arm_wrist_z_revolute = cmd_arm->right_arm[3];
	right_lower_arm = cmd_arm->right_arm[4];
	right_upper_arm_z_revolute = cmd_arm->right_arm[5];
	right_upper_arm_x_revolute = cmd_arm->right_arm[6];
	right_shoulder = cmd_arm->right_arm[7];
	left_arm_finger1 = cmd_arm->left_arm[0];
	left_arm_finger2 = cmd_arm->left_arm[1];
	left_arm_wrist_x_revolute = cmd_arm->left_arm[2];
	left_arm_wrist_z_revolute = cmd_arm->left_arm[3];
	left_lower_arm = cmd_arm->left_arm[4];
	left_upper_arm_z_revolute = cmd_arm->left_arm[5];
	left_upper_arm_x_revolute = cmd_arm->left_arm[6];
	left_shoulder = cmd_arm->left_arm[7];

	_vel = cmd_arm->vel;
	_speed = cmd_arm->speed;

	_drrobotmotionarmdriver->sendServoCtrlAllCmd(right_arm_finger1, right_arm_finger2, right_arm_wrist_x_revolute, right_arm_wrist_z_revolute, right_lower_arm, right_upper_arm_z_revolute, right_upper_arm_x_revolute, right_shoulder,
			left_arm_finger1, left_arm_finger2, left_arm_wrist_x_revolute, left_arm_wrist_z_revolute, left_lower_arm, left_upper_arm_z_revolute, left_upper_arm_x_revolute, left_shoulder, _vel, _speed);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "DrRobot_Arm_Player");
    
    DrRobot_Arm_Player armPlayer;
    ros::NodeHandle node;
    // Start up the robot
    if (armPlayer.start() != 0)
    {
        exit(-1);
    }
    
    ros::Rate r(10);
    
     while (node.ok())
    {
      ros::spinOnce();

      //ROS_INFO("Arms movements");
      r.sleep();
    }


    // Stop the robot
    armPlayer.stop();

    return(0);
}

