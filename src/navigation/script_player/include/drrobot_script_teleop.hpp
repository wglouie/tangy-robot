//============================================================================
// Name        : drrobot_script_teleop.hpp
// Author      : Ananias Paiva
// Copyright   :
// Description : Control arms using poses written on a script
//============================================================================

#ifndef DRROBOT_SCRIPT_TELEOP_HPP_
#define DRROBOT_SCRIPT_TELEOP_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "DrRobotMotionArmDriver.hpp"
#include <drrobot_h20_arm_player/ArmCmd.h>
#include <stdio.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

class DrRobotScriptTeleop
{
    private:
        drrobot_h20_arm_player::ArmCmd cmdarm_;
        ros::NodeHandle node;
        ros::Publisher pub;       

        std::string to_string(int value)
		{
			stringstream ss;
			ss << value;
			return ss.str();
		}

    public:
    	xmlDoc *doc;
    	
    	DrRobotScriptTeleop();
    	
    	~DrRobotScriptTeleop();
    	
    	void getPose(std::string xmlFile);
};

#endif
