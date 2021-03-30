//============================================================================
// Name        : arm_movegroup.cpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Using code from DrRobotMotionArmDriver to drive arm movements
//============================================================================



#include "arm_movegroup.hpp"
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_interface/planning_interface.h>
#include <iostream>
#include <map>
#include <string>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <std_msgs/String.h>
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"
#include "control_msgs/FollowJointTrajectoryActionFeedback.h"
//#include <boost/iterator/iterator_concepts.hpp>


arm_movegroup::arm_movegroup(){
}
arm_movegroup::~arm_movegroup(){
  
}



//Read xml file
void arm_movegroup::read_xml_file(std::string filePath){
  printf("Attempting to read file...\n");
  //Read XML file
  doc=xmlReadFile(filePath.c_str(),NULL, 0);
  if(doc==NULL){
    printf("Error: could not parse file %s\n", filePath.c_str());
    exit(-1);
  }
	//Point to root node
  xmlNode *root_element = NULL;
  root_element = xmlDocGetRootElement(doc);
 	xmlChar *key;
	//Get child node, one after other getting their contents and set them into a "cmd_arm" message
	xmlNode* cur = root_element->xmlChildrenNode;
	
		for (cur; cur; cur = cur->next)
	{
		if (cur->type == XML_ELEMENT_NODE)
		{
			xmlNode* curchannel = cur->xmlChildrenNode;
			
		  double point[8]={0,0,0,0,0,0,0,0};    //Arguments are goals for position+orientation{x,y,z,orientation_x,orientation_y,
		                                        //                                             orientation_z,orientation_w,arm_group}
		  
			for (curchannel; curchannel; curchannel = curchannel->next)
			{
				if (curchannel->type == XML_ELEMENT_NODE)
				{
				  sleep(10);
					key = xmlNodeListGetString(doc, curchannel->xmlChildrenNode, 1);
					if (!xmlStrcmp(curchannel->name, (const xmlChar *)"x"))
					{
   						point[0]=atof((const char*)key);
   				}	else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"y"))	{
						  point[1]=atof((const char*)key);
         	} else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"z"))	{
						  point[2]=atof((const char*)key);
         	}	else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"orientation_x"))	{
						  point[3]=atof((const char*)key);
         	}	else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"orientation_y"))	{
						  point[4]=atof((const char*)key);
         	}	else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"orientation_z"))	{
						  point[5]=atof((const char*)key);
         	}	else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"orientation_w"))	{
						  point[6]=atof((const char*)key);
         	} else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"arm"))	{
              point[7]=atof((const char*)key);
              if(point[7]==(0)){
                move_right_arm(point[0],point[1],point[2],point[3],point[4],point[5],point[6]);
              }else if(point[7]==(1)){
                move_left_arm(point[0],point[1],point[2],point[3],point[4],point[5],point[6]);
              }
         	}
       			xmlFree(key);
		   		}
			}
					
		}
      
	}
	xmlFreeDoc(doc);
  xmlCleanupParser();
	
}


//Move right arm
//Can call this directly instead of going through the xml file
void arm_movegroup::move_right_arm(double x, double y, double z, double orientation_x, double orientation_y, double orientation_z, double orientation_w){
  printf("Goal for end effector: \n x = %f, y= %f, z= %f, orientation_x=%f, orientation_y=%f, orientation_z=%f, orientation_w=%f \n", x,y,z,orientation_x,orientation_y, orientation_z, orientation_w);
  // Part of the robot to move
  moveit::planning_interface::MoveGroup group_right_arm("right_arm");
  group_right_arm.setEndEffectorLink("right_hand");
  group_right_arm.setGoalPositionTolerance(0.01);
  group_right_arm.setGoalOrientationTolerance(0.1);
  
  geometry_msgs::Pose target_pose;
  target_pose.position.x=x;
  target_pose.position.y=y;
  target_pose.position.z=z;
  target_pose.orientation.x=orientation_x/sqrt(pow(orientation_x,2)+pow(orientation_y,2)+pow(orientation_z,2)+pow(orientation_w,2));
  target_pose.orientation.y=orientation_y/sqrt(pow(orientation_x,2)+pow(orientation_y,2)+pow(orientation_z,2)+pow(orientation_w,2));
  target_pose.orientation.z=orientation_z/sqrt(pow(orientation_x,2)+pow(orientation_y,2)+pow(orientation_z,2)+pow(orientation_w,2));
  target_pose.orientation.w=orientation_w/sqrt(pow(orientation_x,2)+pow(orientation_y,2)+pow(orientation_z,2)+pow(orientation_w,2));
  
  group_right_arm.setPoseTarget(target_pose);
  // group_right_arm.setRPYTarget(rpy_x,rpy_y,rpy_z);
  // group_right_arm.setPositionTarget(x,y,z);
    
  moveit::planning_interface::MoveGroup::Plan right_arm_plan;
  bool success_right=group_right_arm.plan(right_arm_plan);
  
  // if (1){
  //   ROS_INFO("Visualizing right arm plan (again)");
  //   display_trajectory.trajectory_start = right_arm_plan.start_state_;
  //   display_trajectory.trajectory.push_back(right_arm_plan.trajectory_);
  //   display_publisher.publish(display_trajectory);
  //   /* Sleep to give Rviz time to visualize the plan. */
  //   sleep(3.0);
  // }
  sleep(2.0);
  group_right_arm.move();
}


void arm_movegroup::move_left_arm(double x, double y, double z, double orientation_x, double orientation_y, double orientation_z, double orientation_w){

  // Part of the robot to move
  moveit::planning_interface::MoveGroup group_left_arm("left_arm");
  group_left_arm.setEndEffectorLink("left_hand");
  group_left_arm.setGoalPositionTolerance(0.01);
  group_left_arm.setGoalOrientationTolerance(0.1);
  
  geometry_msgs::Pose target_pose;
  target_pose.position.x=x;
  target_pose.position.y=y;
  target_pose.position.z=z;
  target_pose.orientation.x=orientation_x/sqrt(pow(orientation_x,2)+pow(orientation_y,2)+pow(orientation_z,2)+pow(orientation_w,2));
  target_pose.orientation.y=orientation_y/sqrt(pow(orientation_x,2)+pow(orientation_y,2)+pow(orientation_z,2)+pow(orientation_w,2));
  target_pose.orientation.z=orientation_z/sqrt(pow(orientation_x,2)+pow(orientation_y,2)+pow(orientation_z,2)+pow(orientation_w,2));
  target_pose.orientation.w=orientation_w/sqrt(pow(orientation_x,2)+pow(orientation_y,2)+pow(orientation_z,2)+pow(orientation_w,2));
  
  group_left_arm.setPoseTarget(target_pose);
  // group_left_arm.setRPYTarget(rpy_x,rpy_y,rpy_z);
  // group_left_arm.setPositionTarget(x,y,z);
  
  moveit::planning_interface::MoveGroup::Plan left_arm_plan;
  bool success_left=group_left_arm.plan(left_arm_plan);
  
  // if (1){
  //   ROS_INFO("Visualizing left arm plan (again)");
  //   display_trajectory.trajectory_start = left_arm_plan.start_state_;
  //   display_trajectory.trajectory.push_back(left_arm_plan.trajectory_);
  //   display_publisher.publish(display_trajectory);
  //   /* Sleep to give Rviz time to visualize the plan. */
  //   sleep(3.0);
  // }
  sleep(2.0);
  group_left_arm.move();
}


//Makes a up and down pointing gesture at something on a table; compilation of the above three functions
void arm_movegroup::pointing_gesture(std::string filePath){

  moveit::planning_interface::MoveGroup group_right_arm("right_arm");
  group_right_arm.setEndEffectorLink("right_hand");
  group_right_arm.setGoalPositionTolerance(0.01);
  group_right_arm.setGoalOrientationTolerance(0.1);
  
  moveit::planning_interface::MoveGroup group_left_arm("left_arm");
  group_left_arm.setEndEffectorLink("left_hand");
  group_left_arm.setGoalPositionTolerance(0.01);
  group_left_arm.setGoalOrientationTolerance(0.1);
  
  printf("Attempting to read file...\n");
  //Read XML file
  doc=xmlReadFile(filePath.c_str(),NULL, 0);
  if(doc==NULL){
    printf("Error: could not parse file %s\n", filePath.c_str());
    exit(-1);
  }
	//Point to root node
  xmlNode *root_element = NULL;
  root_element = xmlDocGetRootElement(doc);
 	xmlChar *key;
	//Get child node, one after other getting their contents and set them into a "cmd_arm" message
	xmlNode* cur = root_element->xmlChildrenNode;
	
		for (cur; cur; cur = cur->next)
	{
		if (cur->type == XML_ELEMENT_NODE)
		{
			xmlNode* curchannel = cur->xmlChildrenNode;
			
		  double point[3]={0,0,0};    //Arguments are goals for position+orientation{x,y,z}
		  
			for (curchannel; curchannel; curchannel = curchannel->next)
			{
				if (curchannel->type == XML_ELEMENT_NODE)
				{
					key = xmlNodeListGetString(doc, curchannel->xmlChildrenNode, 1);
					if (!xmlStrcmp(curchannel->name, (const xmlChar *)"x"))
					{
   						point[0]=atof((const char*)key);
   				}	else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"y"))	{
						  point[1]=atof((const char*)key);
         	} else if (!xmlStrcmp(curchannel->name, (const xmlChar *)"z"))	{
						  point[2]=atof((const char*)key);
         	}
       			xmlFree(key);
	   		}
		   		
			}
			if(point[1]<=0){
			  printf("Goal for end effector: \n x = %f, y= %f, z= %f \n", point[0],point[1],point[2]);
			  geometry_msgs::Pose right_arm_pose;
			  right_arm_pose.position.x=point[0];
			  right_arm_pose.position.y=point[1];
			  right_arm_pose.position.z=point[2];
			  right_arm_pose.orientation.x=0.707106781;
			  right_arm_pose.orientation.y=-0.471404521;
			  right_arm_pose.orientation.z=0.471404521;
			  right_arm_pose.orientation.w=0.23570226;
		    moveit::planning_interface::MoveGroup::Plan right_arm_plan;
        group_right_arm.plan(right_arm_plan);
        printf("Goal for end effector: \n x = %f, y= %f, z= %f \n", right_arm_pose.position.x,right_arm_pose.position.y,right_arm_pose.position.z);
			  group_right_arm.setPoseTarget(right_arm_pose);
			  group_right_arm.move();
			  printf("\n Moving into place... \n");
			  sleep(6.0);
			  //Pointing gesture
        //TODO: Find a way to do the pointing gesture
			  //Reset Arm Position
			  printf("\n Finished pointing... \n");
			  sleep(6.0);
			  group_right_arm.stop();
			  group_right_arm.clearPoseTargets();
			  std::vector<double> curr_state=group_right_arm.getCurrentJointValues();
        printf("\n[%f,%f,%f,%f,%f,%f] \n", curr_state.at(0), curr_state.at(1), curr_state.at(2), curr_state.at(3), curr_state.at(4), curr_state.at(5));
			  std::map<std::string, double> rightjointmap;
        rightjointmap.insert(std::pair<std::string,double>("right_shoulder_x_revolute", curr_state.at(0)));
        rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_y_revolute_joint",curr_state.at(1)));
        rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_z_revolute_joint",curr_state.at(2)));
        rightjointmap.insert(std::pair<std::string,double>("upper_to_lower_right_joint",curr_state.at(3)));
        rightjointmap.insert(std::pair<std::string,double>("wrist_right_arm_z_revolute_joint",curr_state.at(4)));
        rightjointmap.insert(std::pair<std::string,double>("lower_to_wrist_right_arm",curr_state.at(5)));
			  
			  printf("\n Resetting Position... \n");
        rightjointmap["right_shoulder_x_revolute"]=0.001;
        rightjointmap["upper_right_arm_y_revolute_joint"]=0.0;
        rightjointmap["upper_right_arm_z_revolute_joint"]=0.0;
        rightjointmap["upper_to_lower_right_joint"]=0.0;
        rightjointmap["wrist_right_arm_z_revolute_joint"]=0.0;
        rightjointmap["lower_to_wrist_right_arm"]=0.0;
			  group_right_arm.setJointValueTarget(rightjointmap);
			 // group_right_arm.plan(right_arm_plan);
       
        group_right_arm.move();
			  
			}else if (point[1]>0){
			  printf("Goal for end effector: \n x = %f, y= %f, z= %f \n", point[0],point[1],point[2]);
			  geometry_msgs::Pose left_arm_pose;
			  left_arm_pose.position.x=point[0];
			  left_arm_pose.position.y=point[1];
			  left_arm_pose.position.z=point[2];
			  left_arm_pose.orientation.x=-0.707106781;
			  left_arm_pose.orientation.y=-0.471404521;
			  left_arm_pose.orientation.z=0.471404521;
			  left_arm_pose.orientation.w=-0.23570226;
		    moveit::planning_interface::MoveGroup::Plan left_arm_plan;
        group_left_arm.plan(left_arm_plan);
        printf("Goal for end effector: \n x = %f, y= %f, z= %f \n", left_arm_pose.position.x,left_arm_pose.position.y,left_arm_pose.position.z);
			  group_left_arm.setPoseTarget(left_arm_pose);
			  group_left_arm.move();
			  printf("\n Moving into place... \n");
			  sleep(6.0);
			  //Pointing gesture
        //TODO: Find a way to do the pointing gesture
			  //Reset Arm Position
			  printf("\n Finished pointing... \n");
			  sleep(4.0);
			  group_left_arm.stop();
			  group_left_arm.clearPoseTargets();
			  std::vector<double> curr_state=group_left_arm.getCurrentJointValues();
        printf("\n[%f,%f,%f,%f,%f,%f] \n", curr_state.at(0), curr_state.at(1), curr_state.at(2), curr_state.at(3), curr_state.at(4), curr_state.at(5));
			  std::map<std::string, double> leftjointmap;
        leftjointmap.insert(std::pair<std::string,double>("left_shoulder_x_revolute", curr_state.at(0)));
        leftjointmap.insert(std::pair<std::string,double>("upper_left_arm_y_revolute_joint",curr_state.at(1)));
        leftjointmap.insert(std::pair<std::string,double>("upper_left_arm_z_revolute_joint",curr_state.at(2)));
        leftjointmap.insert(std::pair<std::string,double>("upper_to_lower_left_joint",curr_state.at(3)));
        leftjointmap.insert(std::pair<std::string,double>("wrist_left_arm_z_revolute_joint",curr_state.at(4)));
        leftjointmap.insert(std::pair<std::string,double>("lower_to_wrist_left_arm",curr_state.at(5)));
			  
			  printf("\n Resetting Position... \n");
        leftjointmap["left_shoulder_x_revolute"]=0.001;
        leftjointmap["upper_left_arm_y_revolute_joint"]=0.0;
        leftjointmap["upper_left_arm_z_revolute_joint"]=0.0;
        leftjointmap["upper_to_lower_left_joint"]=0.0;
        leftjointmap["wrist_left_arm_z_revolute_joint"]=0.0;
        leftjointmap["lower_to_wrist_left_arm"]=0.0;
			  group_left_arm.setJointValueTarget(leftjointmap);
			 // group_left_arm.plan(left_arm_plan);
       
        group_left_arm.move();
			}
			
			
		}
      
	}
	xmlFreeDoc(doc);
  xmlCleanupParser();
	
}

void arm_movegroup::point_at_screen()
{
  moveit::planning_interface::MoveGroup group_right_arm("right_arm");
  group_right_arm.setEndEffectorLink("right_hand");
  group_right_arm.setGoalPositionTolerance(0.01);
  group_right_arm.setGoalOrientationTolerance(0.1);
  
  geometry_msgs::Pose right_arm_pose;
  right_arm_pose.position.x=-0.21;
  right_arm_pose.position.y=-0.2;
  right_arm_pose.position.z=0.95;
  right_arm_pose.orientation.x=0.554700196;
  right_arm_pose.orientation.y=-0.277350098;
  right_arm_pose.orientation.z=0.554700196;
  right_arm_pose.orientation.w=-0.554700196;
  moveit::planning_interface::MoveGroup::Plan right_arm_plan;
  group_right_arm.plan(right_arm_plan);
  printf("\n Pointing at screen \n");
  group_right_arm.setPoseTarget(right_arm_pose);
  group_right_arm.move();
  sleep(9.0);
  
  right_arm_pose.position.x=-0.2;
  right_arm_pose.position.y=0.1;
  right_arm_pose.position.z=0.575;
  right_arm_pose.orientation.x=0;
  right_arm_pose.orientation.y=1;
  right_arm_pose.orientation.z=0;
  right_arm_pose.orientation.w=0;
  group_right_arm.plan(right_arm_plan);
  printf("\n Returning to home position \n");
  group_right_arm.setPoseTarget(right_arm_pose);
  group_right_arm.move();
  sleep(4.0);
  
  std::vector<double> curr_state=group_right_arm.getCurrentJointValues();
  std::map<std::string, double> rightjointmap;
  rightjointmap.insert(std::pair<std::string,double>("right_shoulder_x_revolute", curr_state.at(0)));
  rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_y_revolute_joint",curr_state.at(1)));
  rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_z_revolute_joint",curr_state.at(2)));
  rightjointmap.insert(std::pair<std::string,double>("upper_to_lower_right_joint",curr_state.at(3)));
  rightjointmap.insert(std::pair<std::string,double>("wrist_right_arm_z_revolute_joint",curr_state.at(4)));
  rightjointmap.insert(std::pair<std::string,double>("lower_to_wrist_right_arm",curr_state.at(5)));
  
  printf("\n Resetting Position... \n");
  rightjointmap["right_shoulder_x_revolute"]=0.001;
  rightjointmap["upper_right_arm_y_revolute_joint"]=0.0;
  rightjointmap["upper_right_arm_z_revolute_joint"]=0.0;
  rightjointmap["upper_to_lower_right_joint"]=0.0;
  rightjointmap["wrist_right_arm_z_revolute_joint"]=0.0;
  rightjointmap["lower_to_wrist_right_arm"]=0.0;
  group_right_arm.setJointValueTarget(rightjointmap);
  // group_right_arm.plan(right_arm_plan);
  
  group_right_arm.move();
}

void arm_movegroup::celebrate()
{
  moveit::planning_interface::MoveGroup group_right_arm("right_arm");
  group_right_arm.setEndEffectorLink("right_hand");
  moveit::planning_interface::MoveGroup group_left_arm("left_arm");
  group_left_arm.setEndEffectorLink("left_hand");
  
  std::vector<double> curr_state=group_right_arm.getCurrentJointValues();
  std::map<std::string, double> rightjointmap;
  rightjointmap.insert(std::pair<std::string,double>("right_shoulder_x_revolute", 0.001));
  rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_y_revolute_joint",0.0));
  rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_z_revolute_joint",0.0));
  rightjointmap.insert(std::pair<std::string,double>("upper_to_lower_right_joint",0.0));
  rightjointmap.insert(std::pair<std::string,double>("wrist_right_arm_z_revolute_joint",0.0));
  rightjointmap.insert(std::pair<std::string,double>("lower_to_wrist_right_arm",0.0));
  
  std::map<std::string, double> leftjointmap;
  leftjointmap.insert(std::pair<std::string,double>("left_shoulder_x_revolute", 0.001));
  leftjointmap.insert(std::pair<std::string,double>("upper_left_arm_y_revolute_joint",0.0));
  leftjointmap.insert(std::pair<std::string,double>("upper_left_arm_z_revolute_joint",0.0));
  leftjointmap.insert(std::pair<std::string,double>("upper_to_lower_left_joint",0.0));
  leftjointmap.insert(std::pair<std::string,double>("wrist_left_arm_z_revolute_joint",0.0));
  leftjointmap.insert(std::pair<std::string,double>("lower_to_wrist_left_arm",0.0));

  rightjointmap["right_shoulder_x_revolute"]=2.6;
  rightjointmap["upper_right_arm_y_revolute_joint"]=-0.6;
  rightjointmap["upper_right_arm_z_revolute_joint"]=-1.6;
  rightjointmap["upper_to_lower_right_joint"]=-1.0;
  rightjointmap["wrist_right_arm_z_revolute_joint"]=0.4;
  rightjointmap["lower_to_wrist_right_arm"]=0.0;
  group_right_arm.setJointValueTarget(rightjointmap);
			  
  leftjointmap["left_shoulder_x_revolute"]=2.8;
  leftjointmap["upper_left_arm_y_revolute_joint"]=-0.8;
  leftjointmap["upper_left_arm_z_revolute_joint"]=-1.6;
  leftjointmap["upper_to_lower_left_joint"]=-1.0;
  leftjointmap["wrist_left_arm_z_revolute_joint"]=0.4;
  leftjointmap["lower_to_wrist_left_arm"]=0.0;
  group_left_arm.setJointValueTarget(leftjointmap);
  
  group_left_arm.move();
  group_right_arm.move();
  
  sleep(2.0);
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_movegroup");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  arm_movegroup move_client;
  printf("Move group client created...\n");
  
  //Use the following to move the end effector to a specific set of points
  move_client.read_xml_file("/home/tangerine/ros/tangy/src/navigation/drrobot_h20_arm_player/config/points.xml");
  
  //Use the following to make a pointing gesture at a specific set of points
  // move_client.pointing_gesture("/home/tangerine/ros/tangy/src/navigation/drrobot_h20_arm_player/config/pointing_gesture.xml");
  
  //Use the following to point at screen
  // move_client.point_at_screen();
  
  //Use the following to celebrate
  // move_client.celebrate();
  
  /**Set the goal position of all joints in the joint groups
  std::map<std::string, double> rightjointmap;
  std::map<std::string, double> leftjointmap;
  // leftjointmap.insert(std::pair<std::string,double>("left_shoulder_x_revolute",0.0));
  // leftjointmap.insert(std::pair<std::string,double>("upper_left_arm_y_revolute_joint",-0.8));
  // leftjointmap.insert(std::pair<std::string,double>("upper_left_arm_z_revolute_joint",1.6));
  // leftjointmap.insert(std::pair<std::string,double>("upper_to_lower_left_joint",1.4));
  // leftjointmap.insert(std::pair<std::string,double>("wrist_left_arm_z_revolute_joint",0.0));
  // leftjointmap.insert(std::pair<std::string,double>("lower_to_wrist_left_arm",0.0));
  // jointmap.insert(std::pair<std::string,double>("wrist_to_finger1_left_arm",0.0));
  // jointmap.insert(std::pair<std::string,double>("wrist_to_finger2_left_arm",0.0));
  rightjointmap.insert(std::pair<std::string,double>("right_shoulder_x_revolute",3.1));
  rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_y_revolute_joint",-0.7));
  rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_z_revolute_joint",-1.5));
  rightjointmap.insert(std::pair<std::string,double>("upper_to_lower_right_joint",-1.2));
  rightjointmap.insert(std::pair<std::string,double>("wrist_right_arm_z_revolute_joint",0.0));
  rightjointmap.insert(std::pair<std::string,double>("lower_to_wrist_right_arm",0.0));
  jointmap.insert(std::pair<std::string,double>("wrist_to_finger1_left_arm",0.0));
  jointmap.insert(std::pair<std::string,double>("wrist_to_finger2_left_arm",0.0));
  jointmap.insert(std::pair<std::string,double>("neck_to_z_revolute",0.0));
  jointmap.insert(std::pair<std::string,double>("z_revolute_to_x_revolute",0.0));*/

  return 0;
}
