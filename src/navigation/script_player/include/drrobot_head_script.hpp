//============================================================================
// Name        : drrobot_head_script.hpp
// Author      : Ananias Paiva
// Copyright   :
// Description : Control arms using poses written on a script
//============================================================================

#ifndef DRROBOT_HEAD_SCRIPT_HPP_
#define DRROBOT_HEAD_SCRIPT_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "DrRobotMotionArmDriver.hpp"
#include <drrobot_h20_player/HeadCmd.h>
#include <stdio.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

class DrRobotHeadScript
{
    private:
        drrobot_h20_player::HeadCmd cmdhead_;
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
    	
    	DrRobotHeadScript();
    	
    	~DrRobotHeadScript();
    	
    	void getPose(std::string xmlFile);
};

#endif
