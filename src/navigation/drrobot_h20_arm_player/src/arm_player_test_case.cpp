//============================================================================
// Name        : DrRobot_Arm_Broadcaster.cpp
// Author      : Ananias Paiva
// Copyright   :
// Description : Broadcast tangy model frames and make tangy model move
//============================================================================

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include "ros/ros.h"
#include <vector>
#include <iostream>
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"
#include "control_msgs/FollowJointTrajectoryActionFeedback.h"
#include "controller_manager/controller_manager.h"
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include <ros/console.h>
#include <controller_manager/controller_loader.h>
#include <controller_manager_msgs/ControllerState.h>

#define pi 3.14159265

using namespace std;

	

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle nh;

  ros::Publisher test_0=nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("r_arm_controller_1/follow_joint_trajectory/goal", 1000);
  ros::Publisher test_1=nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("r_arm_controller_2/follow_joint_trajectory/goal", 1000);
  ros::Publisher test_2=nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("r_arm_controller_3/follow_joint_trajectory/goal", 1000);
  ros::Publisher test_3=nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("r_arm_controller_4/follow_joint_trajectory/goal", 1000);
  ros::Publisher test_4=nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("r_arm_controller_5/follow_joint_trajectory/goal", 1000);
  ros::Publisher test_5=nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("r_arm_controller_6/follow_joint_trajectory/goal", 1000);
  ros::Publisher test_6=nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("l_arm_controller_1/follow_joint_trajectory/goal", 1000);
  ros::Publisher test_7=nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("l_arm_controller_2/follow_joint_trajectory/goal", 1000);
  ros::Publisher test_8=nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("l_arm_controller_3/follow_joint_trajectory/goal", 1000);
  ros::Publisher test_9=nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("l_arm_controller_4/follow_joint_trajectory/goal", 1000);
  ros::Publisher test_10=nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("l_arm_controller_5/follow_joint_trajectory/goal", 1000);
  ros::Publisher test_11=nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("l_arm_controller_6/follow_joint_trajectory/goal", 1000);
  
  control_msgs::FollowJointTrajectoryActionGoal goal0;
  goal0.goal.trajectory.joint_names.push_back("r_arm_controller_1");
  goal0.goal.trajectory.points.resize(2);
  goal0.goal.trajectory.points[0].positions.resize(1);
  goal0.goal.trajectory.points[0].positions[0]=0;
  goal0.goal.trajectory.points[1].positions.resize(1);
  goal0.goal.trajectory.points[1].positions[0]=-0.5353;
  // goal0.goal.trajectory.points[2].positions.resize(1);
  // goal0.goal.trajectory.points[2].positions[0]=1.25;
  // goal0.goal.trajectory.points[3].positions.resize(1);
  // goal0.goal.trajectory.points[3].positions[0]=0.5;
  
  control_msgs::FollowJointTrajectoryActionGoal goal1;
  goal1.goal.trajectory.joint_names.push_back("r_arm_controller_2");
  goal1.goal.trajectory.points.resize(2);
  goal1.goal.trajectory.points[0].positions.resize(1);
  goal1.goal.trajectory.points[0].positions[0]=0;
  goal1.goal.trajectory.points[1].positions.resize(1);
  goal1.goal.trajectory.points[1].positions[0]=1.2354;
  // goal1.goal.trajectory.points[2].positions.resize(1);
  // goal1.goal.trajectory.points[2].positions[0]=1.57079;
  // goal1.goal.trajectory.points[3].positions.resize(1);
  // goal1.goal.trajectory.points[3].positions[0]=0.75;
  
  control_msgs::FollowJointTrajectoryActionGoal goal2;
  goal2.goal.trajectory.joint_names.push_back("r_arm_controller_3");
  goal2.goal.trajectory.points.resize(2);
  goal2.goal.trajectory.points[0].positions.resize(1);  
  goal2.goal.trajectory.points[0].positions[0]=0;  
  goal2.goal.trajectory.points[1].positions.resize(1);  
  goal2.goal.trajectory.points[1].positions[0]=-1.6503;  
  // goal2.goal.trajectory.points[2].positions.resize(1);  
  // goal2.goal.trajectory.points[2].positions[0]=-1.8;  
  // goal2.goal.trajectory.points[3].positions.resize(1);  
  // goal2.goal.trajectory.points[3].positions[0]=-0.9;  
  
  control_msgs::FollowJointTrajectoryActionGoal goal3;
  goal3.goal.trajectory.joint_names.push_back("r_arm_controller_4");
  goal3.goal.trajectory.points.resize(2);
  goal3.goal.trajectory.points[0].positions.resize(1);
  goal3.goal.trajectory.points[0].positions[0]=0;
  goal3.goal.trajectory.points[1].positions.resize(1);
  goal3.goal.trajectory.points[1].positions[0]=-1.4002;
  // goal3.goal.trajectory.points[2].positions.resize(1);
  // goal3.goal.trajectory.points[2].positions[0]=-2.57;
  // goal3.goal.trajectory.points[3].positions.resize(1);
  // goal3.goal.trajectory.points[3].positions[0]=-2.07;
  
  control_msgs::FollowJointTrajectoryActionGoal goal4;
  goal4.goal.trajectory.joint_names.push_back("r_arm_controller_5");
  goal4.goal.trajectory.points.resize(2);
  goal4.goal.trajectory.points[0].positions.resize(1);
  goal4.goal.trajectory.points[0].positions[0]=0;
  goal4.goal.trajectory.points[1].positions.resize(1);
  goal4.goal.trajectory.points[1].positions[0]=-0.9;  
  // goal4.goal.trajectory.points[2].positions.resize(1);
  // goal4.goal.trajectory.points[2].positions[0]=-2.57; 
  // goal4.goal.trajectory.points[3].positions.resize(1);
  // goal4.goal.trajectory.points[3].positions[0]=-2.07; 
  
  control_msgs::FollowJointTrajectoryActionGoal goal5;
  goal5.goal.trajectory.joint_names.push_back("r_arm_controller_6");
  goal5.goal.trajectory.points.resize(2);
  goal5.goal.trajectory.points[0].positions.resize(1);
  goal5.goal.trajectory.points[0].positions[0]=0;
  goal5.goal.trajectory.points[1].positions.resize(1);
  goal5.goal.trajectory.points[1].positions[0]=-0.1827;
  // goal5.goal.trajectory.points[2].positions.resize(1);
  // goal5.goal.trajectory.points[2].positions[0]=2.5;
  // goal5.goal.trajectory.points[3].positions.resize(1);
  // goal5.goal.trajectory.points[3].positions[0]=1.5;
  
  control_msgs::FollowJointTrajectoryActionGoal goal6;
  goal6.goal.trajectory.joint_names.push_back("l_arm_controller_1");
  goal6.goal.trajectory.points.resize(2);
  goal6.goal.trajectory.points[0].positions.resize(1);
  goal6.goal.trajectory.points[0].positions[0]=0;
  goal6.goal.trajectory.points[1].positions.resize(1);
  goal6.goal.trajectory.points[1].positions[0]=-0.7059;
  // goal6.goal.trajectory.points[2].positions.resize(1);
  // goal6.goal.trajectory.points[2].positions[0]=0;
  // goal6.goal.trajectory.points[3].positions.resize(1);
  // goal6.goal.trajectory.points[3].positions[0]=0;
  
  control_msgs::FollowJointTrajectoryActionGoal goal7;
  goal7.goal.trajectory.joint_names.push_back("l_arm_controller_2");
  goal7.goal.trajectory.points.resize(2);
  goal7.goal.trajectory.points[0].positions.resize(1);
  goal7.goal.trajectory.points[0].positions[0]=0;
  goal7.goal.trajectory.points[1].positions.resize(1);
  goal7.goal.trajectory.points[1].positions[0]=0.9177;
  // goal7.goal.trajectory.points[2].positions.resize(1);
  // goal7.goal.trajectory.points[2].positions[0]=1.57079;
  // goal7.goal.trajectory.points[3].positions.resize(1);
  // goal7.goal.trajectory.points[3].positions[0]=1.57079;
  
  control_msgs::FollowJointTrajectoryActionGoal goal8;
  goal8.goal.trajectory.joint_names.push_back("l_arm_controller_3");
  goal8.goal.trajectory.points.resize(2);
  goal8.goal.trajectory.points[0].positions.resize(1);
  goal8.goal.trajectory.points[0].positions[0]=0;
  goal8.goal.trajectory.points[1].positions.resize(1);
  goal8.goal.trajectory.points[1].positions[0]=-1.3496;
  // goal8.goal.trajectory.points[2].positions.resize(1);
  // goal8.goal.trajectory.points[2].positions[0]=0;
  // goal8.goal.trajectory.points[3].positions.resize(1);
  // goal8.goal.trajectory.points[3].positions[0]=0;
  
  control_msgs::FollowJointTrajectoryActionGoal goal9;
  goal9.goal.trajectory.joint_names.push_back("l_arm_controller_4");
  goal9.goal.trajectory.points.resize(2);
  goal9.goal.trajectory.points[0].positions.resize(1);
  goal9.goal.trajectory.points[0].positions[0]=0;
  goal9.goal.trajectory.points[1].positions.resize(1);
  goal9.goal.trajectory.points[1].positions[0]=-1.3384;
  // goal9.goal.trajectory.points[2].positions.resize(1);
  // goal9.goal.trajectory.points[2].positions[0]=-1.57079;
  // goal9.goal.trajectory.points[3].positions.resize(1);
  // goal9.goal.trajectory.points[3].positions[0]=-1.57079;
  
  control_msgs::FollowJointTrajectoryActionGoal goal10;
  goal10.goal.trajectory.joint_names.push_back("l_arm_controller_5");
  goal10.goal.trajectory.points.resize(2);
  goal10.goal.trajectory.points[0].positions.resize(1);
  goal10.goal.trajectory.points[0].positions[0]=0;
  goal10.goal.trajectory.points[1].positions.resize(1);
  goal10.goal.trajectory.points[1].positions[0]=-0.9;
  // goal10.goal.trajectory.points[2].positions.resize(1);
  // goal10.goal.trajectory.points[2].positions[0]=-1.57079;
  // goal10.goal.trajectory.points[3].positions.resize(1);
  // goal10.goal.trajectory.points[3].positions[0]=-1.57079;
  
  control_msgs::FollowJointTrajectoryActionGoal goal11;
  goal11.goal.trajectory.joint_names.push_back("l_arm_controller_6");
  goal11.goal.trajectory.points.resize(2);
  goal11.goal.trajectory.points[0].positions.resize(1);
  goal11.goal.trajectory.points[0].positions[0]=0;
  goal11.goal.trajectory.points[1].positions.resize(1);
  goal11.goal.trajectory.points[1].positions[0]=-0.1827;
  // goal11.goal.trajectory.points[2].positions.resize(1);
  // goal11.goal.trajectory.points[2].positions[0]=0;
  // goal11.goal.trajectory.points[3].positions.resize(1);
  // goal11.goal.trajectory.points[3].positions[0]=0;

  while(ros::ok())
  {
    test_0.publish(goal0);
    test_1.publish(goal1);
    test_2.publish(goal2);
    test_3.publish(goal3);
    test_4.publish(goal4);
    test_5.publish(goal5);
    test_6.publish(goal6);
    test_7.publish(goal7);
    test_8.publish(goal8);
    test_9.publish(goal9);
    test_10.publish(goal10);
    test_11.publish(goal11);
    ros::spinOnce();

  }

  
  return 0;
}
