//============================================================================
// Name        : arm_movegroup.hpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Using code from DrRobotMotionArmDriver to drive arm movements
//============================================================================

#ifndef arm_movegroup_HPP_
#define arm_movegroup_HPP_
#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <geometry_msgs/Pose.h>


class arm_movegroup {
  public:
    arm_movegroup();
    ~arm_movegroup();

    xmlDoc *doc;
    void read_xml_file(std::string filePath);
    void write_to_file(std::string filePath);
    void move_right_arm(double x, double y, double z, double orientation_x, double orientation_y, double orientation_z, double orientation_w);
    void move_left_arm(double x, double y, double z, double orientation_x, double orientation_y, double orientation_z, double orientation_w);
    void pointing_gesture(std::string filePath);

    void point_at_screen();

    void celebrate();
};

#endif