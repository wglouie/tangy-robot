//============================================================================
// Name        : arm_actlib.hpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Using code from DrRobotMotionArmDriver to drive arm movements
//============================================================================

#include <ros/ros.h>

#include "stdlib.h"
#include <vector>
#include <string>

//Include headers for ROS service
#include <actionlib/server/simple_action_server.h>
#include <face_detection/identifyAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_interface/planning_interface.h>
#include <iostream>
#include <map>
#include <string>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <time.h>
#include <actionlib/server/simple_action_server.h>
#include <drrobot_h20_arm_player/armAction.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"
#include "control_msgs/FollowJointTrajectoryActionFeedback.h"
#include "arm_player_eff.hpp"
#include "controller_manager/controller_manager.h"
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include <ros/console.h>
#include <controller_manager/controller_loader.h>
#include <controller_manager_msgs/ControllerState.h>
#include <unistd.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <signal.h>
#include <unistd.h>
#include <ArmOptimizer.h>

using namespace std;

class ArmMovement: public hardware_interface::RobotHW{
  
  ros::NodeHandle nh_;
  /*Action library variables*/
  actionlib::SimpleActionServer<drrobot_h20_arm_player::armAction> as_;
  std::string action_name_;
  drrobot_h20_arm_player::armFeedback feedback_;
  drrobot_h20_arm_player::armResult result_;
  moveit::planning_interface::MoveGroup::Plan go_to_neutral_plan;
  moveit::planning_interface::MoveGroup::Plan point_to_plan;
  moveit::planning_interface::MoveGroup::Plan celebrate_plan;
  moveit::planning_interface::MoveGroup::Plan wave_goodbye_plan;
  moveit::planning_interface::MoveGroup::Plan point_to_screen_plan;
  moveit::planning_interface::MoveGroup::Plan laugh_plan;
  vector<moveit::planning_interface::MoveGroup::Plan> convo_gesture_plans;
  moveit::planning_interface::MoveGroup::Plan clap_plan;
  
  // ArmOptimizer point_to_screen_opt;
  // ArmOptimizer wave_opt;
  // ArmOptimizer celebrate_opt;
  // ArmOptimizer laugh_opt;
  
  public:
  
    double pos_x;
    double pos_y;
    double pos_z;
    string behavior;
    string planFilePath;
    moveit::planning_interface::MoveGroup both_arms;
    moveit::planning_interface::MoveGroup group_right_arm;
    moveit::planning_interface::MoveGroup group_left_arm;
  
  	struct DrRobotArmConfig
		{
			std::string robotID;
			std::string robotIP;
			int portNum;
			
		};
  
    std::string robotID;
    std::string robotIP;
    int portNum;
    
    
    ArmMovement(std::string name): as_(nh_, name, false),	action_name_(name), both_arms("both_arms"), group_right_arm("right_arm"), group_left_arm("left_arm")/*, point_to_screen_opt("PSO_Opt"), wave_opt("W_Opt"), celebrate_opt("C_Opt"), laugh_opt("L_Opt")*/
    {
		  //register the goal and feeback callbacks
  		as_.registerGoalCallback(boost::bind(&ArmMovement::goalCB, this));
  		as_.registerPreemptCallback(boost::bind(&ArmMovement::preemptCB, this));
  		as_.start();
  		feedback_.state=0;
  		
  		  //   /*  FAKE TABLE CODE */
      // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      // moveit_msgs::CollisionObject collision_object;
      
      // collision_object.header.frame_id=both_arms.getPlanningFrame();
      // collision_object.id="table";
      // /* Define a box to add to the world. */
      // shape_msgs::SolidPrimitive primitive;
      // primitive.type = primitive.BOX;
      // primitive.dimensions.resize(3);
      // primitive.dimensions[0] = 4;
      // primitive.dimensions[1] = 4;
      // primitive.dimensions[2] = 1;
      
      // /* A pose for the box (specified relative to frame_id) */
      // geometry_msgs::Pose box_pose;
      // box_pose.orientation.w = 1.0;
      // box_pose.position.x =  0;
      // box_pose.position.y = 0.5;
      // box_pose.position.z = 0;
      
      // collision_object.primitives.push_back(primitive);
      // collision_object.primitive_poses.push_back(box_pose);
      // collision_object.operation = collision_object.ADD;
      
      // std::vector<moveit_msgs::CollisionObject> collision_objects;
      // collision_objects.push_back(collision_object);
      
      // ROS_INFO("Add an object into the world");
      // planning_scene_interface.addCollisionObjects(collision_objects);
    
      // /* Sleep so we have time to see the object in RViz */
      // sleep(2.0);
      // both_arms.setStartState(*both_arms.getCurrentState());
      // group_left_arm.setStartState(*group_left_arm.getCurrentState());
      // group_right_arm.setStartState(*group_right_arm.getCurrentState());
      
      planFilePath="/home/tangy/tangy-robot/src/navigation/drrobot_h20_arm_player/plans.txt";
		}
		
		
    ~ArmMovement()
    {
    }
    
    
	/* Execute this code when a goal is provided from action client */
	/* This function will move an arm according the location, orientation or behavior supplied */
    void goalCB()
    {
      ROS_INFO("Received goal");
      boost::shared_ptr<const drrobot_h20_arm_player::armGoal> newGoal = as_.acceptNewGoal();
      pos_x=newGoal->x;
      pos_y=newGoal->y;
      pos_z=newGoal->z;
      behavior=newGoal->behavior;
      
      if(behavior.compare("plan neutral")!=0){
        if(behavior=="plan go to neutral"){
            ROS_INFO("Planning go to neutral");
            plan_go_to_neutral();
        }
        if(behavior=="plan point at screen"){
              ROS_INFO("Planning for point at screen");
              plan_point_to_screen();
        }else if(behavior.compare("plan celebrate")==0){
              ROS_INFO("Planning for celebrate");
              plan_celebrate();
        }else if(behavior.compare("plan wave goodbye")==0){
              ROS_INFO("Planning for wave goodbye");
              plan_wave_goodbye();
        }else if(behavior.compare("plan convo gesture")==0){
              ROS_INFO("Planning for convo gesture");
              plan_convo_gesture();
        }else if(behavior.compare("plan laugh gesture")==0){
              ROS_INFO("Planning for laugh gesture");
              plan_laugh_gesture();
        }else if(behavior.compare("plan clap")==0){
              ROS_INFO("Planning for clap");
              plan_clap();
        }else if(behavior.compare("plan go to neutral")==0){
              ROS_INFO("Executing movement!");
              execute_go_to_neutral();
        }else if(behavior.compare("execute point at screen")==0){
              ROS_INFO("Executing movement!");
              execute_point_to_screen();
        }else if(behavior.compare("execute celebrate")==0){
              ROS_INFO("Executing movement!");
              execute_celebrate();
        }else if(behavior.compare("execute wave goodbye")==0){
              ROS_INFO("Executing movement!");
              execute_wave_goodbye();
        }else if(behavior.compare("execute point to")==0){
              ROS_INFO("Executing movement!");
              execute_point_to();
        }else if(behavior.compare("execute convo gesture")==0){
              ROS_INFO("Executing movement!");
              execute_convo_gesture();
        }else if(behavior.compare("execute laugh gesture")==0){
              ROS_INFO("Executing movement!");
              execute_laugh_gesture();
        }else if(behavior.compare("execute clap")==0){
              ROS_INFO("Executing clap!");
              execute_clap();
        }else{
          ROS_ERROR("INVALID BEHAVIOR");
        }
      }
      
      if(pos_x!=100){
          plan_point_to(pos_x,pos_y,pos_z);
      }
      
    }
    //Higher level pointing behaviors
    void plan_point_to(int x, int y, int z)
    {
      ros::AsyncSpinner spinner(0);
      spinner.start();
      group_right_arm.setEndEffectorLink("right_hand");
      group_right_arm.setGoalPositionTolerance(0.03);
      group_right_arm.setGoalOrientationTolerance(0.2);
      group_right_arm.allowReplanning(true);
      
      group_left_arm.setEndEffectorLink("left_hand");
      group_left_arm.setGoalPositionTolerance(0.03);
      group_left_arm.setGoalOrientationTolerance(0.2);
      group_left_arm.allowReplanning(true);
      
      if(pos_x<=0){
    		  geometry_msgs::Pose right_arm_pose;
    		  right_arm_pose.position.x=pos_x;
    		  right_arm_pose.position.y=pos_y;
    		  right_arm_pose.position.z=pos_z;
    		  right_arm_pose.orientation.x=0.707106781;
    		  right_arm_pose.orientation.y=-0.471404521;
    		  right_arm_pose.orientation.z=0.471404521;
    		  right_arm_pose.orientation.w=0.23570226;
    		  group_right_arm.setPoseTarget(right_arm_pose);
          group_right_arm.plan(point_to_plan);
          for(int i=0; i<10; i++){
              if(point_to_plan.trajectory_.joint_trajectory.points.size()>75||point_to_plan.trajectory_.joint_trajectory.points.size()<20){
                group_right_arm.plan(point_to_plan);
              }
          }
    		  
    		  // group_right_arm.asyncMove();
    		  // printf("\n Moving into place... \n");
          spinner.stop();
        
      }else{
    		  geometry_msgs::Pose left_arm_pose;
    		  left_arm_pose.position.x=pos_x;
    		  left_arm_pose.position.y=pos_y;
    		  left_arm_pose.position.z=pos_z;
    		  left_arm_pose.orientation.x=-0.707106781;
    		  left_arm_pose.orientation.y=-0.471404521;
    		  left_arm_pose.orientation.z=0.471404521;
    		  left_arm_pose.orientation.w=-0.23570226;

    		  group_left_arm.setPoseTarget(left_arm_pose);
          group_left_arm.plan(point_to_plan);
          for(int i=0; i<3; i++){
              if(point_to_plan.trajectory_.joint_trajectory.points.size()>75||point_to_plan.trajectory_.joint_trajectory.points.size()<20){
                group_left_arm.plan(point_to_plan);
              }
          }
    		  // group_left_arm.asyncMove();
    		  // printf("\n Moving into place... \n");
          spinner.stop();
      }
    }

    void plan_point_to_screen()
    {
        ros::AsyncSpinner spinner(0);
        spinner.start();
        std::vector<double> curr_state=group_right_arm.getCurrentJointValues();
        std::map<std::string, double> rightjointmap;
        rightjointmap.insert(std::pair<std::string,double>("right_shoulder_x_revolute", 0.5));
        rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_y_revolute_joint",-1.1));
        rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_z_revolute_joint",-0.5));
        rightjointmap.insert(std::pair<std::string,double>("upper_to_lower_right_joint",-2.0));
        rightjointmap.insert(std::pair<std::string,double>("wrist_right_arm_z_revolute_joint",0.75));
        rightjointmap.insert(std::pair<std::string,double>("lower_to_wrist_right_arm",1.0));
        group_right_arm.setJointValueTarget(rightjointmap);
    
        /*int num_plans=0;
        int debug_incre=0;
        while(num_plans<10){
            debug_incre++;
            group_right_arm.plan(point_to_screen_plan);
            if(point_to_screen_plan.trajectory_.joint_trajectory.points.size()<75 && point_to_screen_plan.trajectory_.joint_trajectory.points.size()>05){
                num_plans++;
                point_to_screen_opt.add_plan(point_to_screen_plan);
                ROS_INFO("Adding point to screen plan %d", num_plans);
            }
            ROS_INFO("Point to Screen Plan Attempt %d", debug_incre);
            
        }
        ROS_INFO("Beginning finding optimal point to screen plan!");
        point_to_screen_plan=point_to_screen_opt.getBestPlan();

        ROS_INFO("Point to screen plan has %d number of points.",(int) point_to_screen_plan.trajectory_.joint_trajectory.points.size());
        point_to_screen_opt.clearPlans();*/
        
        while(point_to_screen_plan.trajectory_.joint_trajectory.points.size()>75|| point_to_screen_plan.trajectory_.joint_trajectory.points.size()<15){
            group_right_arm.plan(point_to_screen_plan);
        }
        
        spinner.stop();
    }
    void plan_go_to_neutral()
    {
        ros::AsyncSpinner spinner(0);
        spinner.start();

        std::vector<double> curr_state=group_right_arm.getCurrentJointValues();
        std::map<std::string, double> botharmsjointmap;
        botharmsjointmap.insert(std::pair<std::string,double>("right_shoulder_x_revolute", 0));
        botharmsjointmap.insert(std::pair<std::string,double>("upper_right_arm_y_revolute_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("upper_right_arm_z_revolute_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("upper_to_lower_right_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("wrist_right_arm_z_revolute_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("lower_to_wrist_right_arm",0));
        botharmsjointmap.insert(std::pair<std::string,double>("left_shoulder_x_revolute", 0));
        botharmsjointmap.insert(std::pair<std::string,double>("upper_left_arm_y_revolute_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("upper_left_arm_z_revolute_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("upper_to_lower_left_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("wrist_left_arm_z_revolute_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("lower_to_wrist_left_arm",0));

        both_arms.setJointValueTarget(botharmsjointmap);

        both_arms.plan(go_to_neutral_plan);

        spinner.stop();

    }


    void plan_celebrate()
    {
        ros::AsyncSpinner spinner(0);
        spinner.start();
        // both_arms.allowReplanning(true);
        
        std::map<std::string, double> botharmsjointmap;
        botharmsjointmap.insert(std::pair<std::string,double>("right_shoulder_x_revolute", 3));
        botharmsjointmap.insert(std::pair<std::string,double>("upper_right_arm_y_revolute_joint",-0.1));
        botharmsjointmap.insert(std::pair<std::string,double>("upper_right_arm_z_revolute_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("upper_to_lower_right_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("wrist_right_arm_z_revolute_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("lower_to_wrist_right_arm",0.0));
        botharmsjointmap.insert(std::pair<std::string,double>("left_shoulder_x_revolute", 3.14));
        botharmsjointmap.insert(std::pair<std::string,double>("upper_left_arm_y_revolute_joint",-0.25));
        botharmsjointmap.insert(std::pair<std::string,double>("upper_left_arm_z_revolute_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("upper_to_lower_left_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("wrist_left_arm_z_revolute_joint",0));
        botharmsjointmap.insert(std::pair<std::string,double>("lower_to_wrist_left_arm",0.0));
        
        both_arms.setJointValueTarget(botharmsjointmap);
        
/*        int num_plans=0;
        int debug_incre=0;
        while(num_plans<10){
            debug_incre++;
            both_arms.plan(celebrate_plan);
            if(celebrate_plan.trajectory_.joint_trajectory.points.size()<65&&celebrate_plan.trajectory_.joint_trajectory.points.size()>15){
              num_plans++;
              celebrate_opt.add_plan(celebrate_plan);
              ROS_INFO("Adding celebrate plan %d", num_plans);
            }
            ROS_INFO("Celebrate Plan Attempt %d", debug_incre);
        }
        ROS_INFO("Beginning finding optimal celebrate plan!");
        celebrate_plan=celebrate_opt.getBestPlan();
        // write_to_file(planFilePath,"celebrate plan", celebrate_plan);
        
        ROS_INFO("Celebrate plan has %d number of points.", (int) celebrate_plan.trajectory_.joint_trajectory.points.size());
        celebrate_opt.clearPlans();*/
        
        while(celebrate_plan.trajectory_.joint_trajectory.points.size()>65||celebrate_plan.trajectory_.joint_trajectory.points.size()<15){
            both_arms.plan(celebrate_plan);
        }
        spinner.stop();
        
    }
    void plan_wave_goodbye()
    {
        ros::AsyncSpinner spinner(0);
        spinner.start();
        // group_right_arm.allowReplanning(true);
        
        std::vector<double> curr_state=group_right_arm.getCurrentJointValues();
        std::map<std::string, double> rightjointmap;
        rightjointmap.insert(std::pair<std::string,double>("right_shoulder_x_revolute", 2.4));
        rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_y_revolute_joint",-1.3));
        rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_z_revolute_joint",-1.6));
        rightjointmap.insert(std::pair<std::string,double>("upper_to_lower_right_joint",-1.6));
        rightjointmap.insert(std::pair<std::string,double>("wrist_right_arm_z_revolute_joint",0.4));
        rightjointmap.insert(std::pair<std::string,double>("lower_to_wrist_right_arm",0.0));
        group_right_arm.setJointValueTarget(rightjointmap);
        
/*        int num_plans=0;
        int debug_incre=0;
        while(num_plans<10){
            debug_incre++;
            group_right_arm.plan(wave_goodbye_plan);
            if(wave_goodbye_plan.trajectory_.joint_trajectory.points.size()<80&&wave_goodbye_plan.trajectory_.joint_trajectory.points.size()>15){
              num_plans++;
              wave_opt.add_plan(wave_goodbye_plan);
              ROS_INFO("Adding waving plan %d", num_plans);
            }
            ROS_INFO("Waving Plan Attempt %d", debug_incre);
        }
        ROS_INFO("Beginning finding optimal waving plan!");
        // write_to_file(planFilePath,"wave plan", wave_goodbye_plan);
        wave_goodbye_plan=wave_opt.getBestPlan();
        ROS_INFO("Wave plan has %d number of points.",(int) wave_goodbye_plan.trajectory_.joint_trajectory.points.size());
        wave_opt.clearPlans();*/
        
        while(wave_goodbye_plan.trajectory_.joint_trajectory.points.size()>80||wave_goodbye_plan.trajectory_.joint_trajectory.points.size()<15){
            group_right_arm.plan(wave_goodbye_plan);
        }
        spinner.stop();
    }
    
    void plan_laugh_gesture()
    {
        ros::AsyncSpinner spinner(0);
        spinner.start();
        std::vector<double> curr_state=group_right_arm.getCurrentJointValues();
        std::map<std::string, double> rightjointmap;
        rightjointmap.insert(std::pair<std::string,double>("right_shoulder_x_revolute", 1.75));
        rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_y_revolute_joint",-0.315));
        rightjointmap.insert(std::pair<std::string,double>("upper_right_arm_z_revolute_joint",-0.9266));
        rightjointmap.insert(std::pair<std::string,double>("upper_to_lower_right_joint",-1.6811));
        rightjointmap.insert(std::pair<std::string,double>("wrist_right_arm_z_revolute_joint",1.57));
        rightjointmap.insert(std::pair<std::string,double>("lower_to_wrist_right_arm",0.161));
        group_right_arm.setJointValueTarget(rightjointmap);
        
        group_right_arm.setJointValueTarget(rightjointmap);
         	
/*        int num_plans=0;
        int debug_incre=0;
        while(num_plans<10){
            debug_incre++;
            group_right_arm.plan(laugh_plan);
            if(laugh_plan.trajectory_.joint_trajectory.points.size()<55&&laugh_plan.trajectory_.joint_trajectory.points.size()>15){
              num_plans++;
              laugh_opt.add_plan(laugh_plan);
              ROS_INFO("Adding laugh plan %d", num_plans);
            }
            ROS_INFO("Laugh Plan Attempt %d", debug_incre);
        }
        ROS_INFO("Beginning finding optimal laugh plan!");
        laugh_plan=laugh_opt.getBestPlan();
        // write_to_file(planFilePath,"celebrate plan", celebrate_plan);
        
        ROS_INFO("Laugh plan has %d number of points.", (int) laugh_plan.trajectory_.joint_trajectory.points.size());
        laugh_opt.clearPlans();*/
        
        while(laugh_plan.trajectory_.joint_trajectory.points.size()>75||laugh_plan.trajectory_.joint_trajectory.points.size()<15){
            group_right_arm.plan(laugh_plan);
        }
        spinner.stop();
        
    }
    
    void plan_convo_gesture()
    {
        ros::AsyncSpinner spinner(0);
        spinner.start();
        // both_arms.allowReplanning(true);
        for(int num_plans=0; num_plans<10; num_plans++){
          double rnum0=(double (rand()%300))/1000+0.2;
          double rnum1=(double (rand()%300))/1000+0.1;
          double rnum2=(double (rand()%500))/1000;
          double rnum3=(double (rand()%600))/800;
          double rnum4=(double (rand()%600))/800;
          moveit::planning_interface::MoveGroup::Plan convoplan;
          std::map<std::string, double> botharmsjointmap;
          botharmsjointmap.insert(std::pair<std::string,double>("right_shoulder_x_revolute", rnum0));
          botharmsjointmap.insert(std::pair<std::string,double>("upper_right_arm_y_revolute_joint",-rnum1));
          botharmsjointmap.insert(std::pair<std::string,double>("upper_right_arm_z_revolute_joint",rnum2*pow(-1,rand()%10)));
          botharmsjointmap.insert(std::pair<std::string,double>("upper_to_lower_right_joint",-rnum3));
          botharmsjointmap.insert(std::pair<std::string,double>("wrist_right_arm_z_revolute_joint",0));
          botharmsjointmap.insert(std::pair<std::string,double>("lower_to_wrist_right_arm",double (rand()%4)/10));
          botharmsjointmap.insert(std::pair<std::string,double>("left_shoulder_x_revolute", rnum0));
          botharmsjointmap.insert(std::pair<std::string,double>("upper_left_arm_y_revolute_joint",-rnum1));
          botharmsjointmap.insert(std::pair<std::string,double>("upper_left_arm_z_revolute_joint",rnum2*pow(-1,rand()%10)));
          botharmsjointmap.insert(std::pair<std::string,double>("upper_to_lower_left_joint",-rnum4));
          botharmsjointmap.insert(std::pair<std::string,double>("wrist_left_arm_z_revolute_joint",0));
          botharmsjointmap.insert(std::pair<std::string,double>("lower_to_wrist_left_arm",double (rand()%4)/10));
          
          both_arms.setJointValueTarget(botharmsjointmap);
          both_arms.plan(convoplan);
          while(convoplan.trajectory_.joint_trajectory.points.size()>30||convoplan.trajectory_.joint_trajectory.points.size()<3){
            both_arms.plan(convoplan);
          }
          
          convo_gesture_plans.push_back(convoplan);
        }

        spinner.stop();
        
    }
    
    void plan_clap()
    {
        ros::AsyncSpinner spinner(0);
        spinner.start();
        // both_arms.allowReplanning(true);
        for(int num_plans=0; num_plans<10; num_plans++){
          moveit::planning_interface::MoveGroup::Plan clap_plan;
          std::map<std::string, double> botharmsjointmap;
          botharmsjointmap.insert(std::pair<std::string,double>("right_shoulder_x_revolute", 0.233));
          botharmsjointmap.insert(std::pair<std::string,double>("upper_right_arm_y_revolute_joint",-0.2588));
          botharmsjointmap.insert(std::pair<std::string,double>("upper_right_arm_z_revolute_joint",-0.9266));
          botharmsjointmap.insert(std::pair<std::string,double>("upper_to_lower_right_joint",-0.9613));
          botharmsjointmap.insert(std::pair<std::string,double>("wrist_right_arm_z_revolute_joint",0.4765));
          botharmsjointmap.insert(std::pair<std::string,double>("lower_to_wrist_right_arm",0.0902));
          botharmsjointmap.insert(std::pair<std::string,double>("left_shoulder_x_revolute", 0.233));
          botharmsjointmap.insert(std::pair<std::string,double>("upper_left_arm_y_revolute_joint",-0.2588));
          botharmsjointmap.insert(std::pair<std::string,double>("upper_left_arm_z_revolute_joint",-0.9266));
          botharmsjointmap.insert(std::pair<std::string,double>("upper_to_lower_left_joint",-0.9613));
          botharmsjointmap.insert(std::pair<std::string,double>("wrist_left_arm_z_revolute_joint",0.4765));
          botharmsjointmap.insert(std::pair<std::string,double>("lower_to_wrist_left_arm",0.0902));
          
          both_arms.setJointValueTarget(botharmsjointmap);
          while(clap_plan.trajectory_.joint_trajectory.points.size()<10){
            
            both_arms.plan(clap_plan);
          }
          
        }

        spinner.stop();
        
    }
    
    void execute_point_to_screen(){
      group_right_arm.asyncExecute(point_to_screen_plan);
    }
    
    void execute_celebrate(){
      both_arms.asyncExecute(celebrate_plan);
    }
    
    void execute_go_to_neutral(){
      both_arms.asyncExecute(go_to_neutral_plan);
    }

    void execute_laugh_gesture(){
      group_right_arm.asyncExecute(laugh_plan);
    }
    void execute_wave_goodbye(){
      group_right_arm.asyncExecute(wave_goodbye_plan);
    }
    void execute_clap(){
      both_arms.asyncExecute(clap_plan);
    }
    void execute_point_to(){
      if(pos_x<=0){
        group_right_arm.asyncExecute(point_to_plan);
      }else if (pos_x>0){
        group_left_arm.asyncExecute(point_to_plan);
      }
      
    }
    
    void execute_convo_gesture(){
      int sz=convo_gesture_plans.size();
      int rand_n=rand()%sz;
      both_arms.asyncExecute(convo_gesture_plans[rand_n]);
    }
    
    void preemptCB()
    {
  		ROS_INFO("%s: Preempted", action_name_.c_str());
  		// set the action state to preempted
  		as_.setPreempted();
    }
    
    void feedbackCB(const std_msgs::String::ConstPtr& msg)
    {
      ROS_INFO("Feedback received by action server!");
      
      if(msg->data.compare("Executing plan")==0){
        feedback_.state=1;
        as_.publishFeedback(feedback_);
        ROS_INFO("Feedback is: [%s]", msg->data.c_str());
        
      }else if(msg->data.compare("Finished executing plan")==0){
        feedback_.state=0;
        as_.publishFeedback(feedback_);
        ROS_INFO("Feedback is: [%s]", msg->data.c_str());
      }
    }
    
    void write_to_file(std::string filePath,std::string plan_name,moveit::planning_interface::MoveGroup::Plan plan){
      ROS_INFO("Attempting to write to file!");
      ofstream writeNumberStream;
      writeNumberStream.open(filePath.c_str(), ios::in | ios::app);
      writeNumberStream<<"\n"<<plan_name;
      for(int i=0;i<plan.trajectory_.joint_trajectory.points.size();i++){
        char str[50];
        std::string line;
        for(int n=0; n<plan.trajectory_.joint_trajectory.points[i].positions.size();n++){
          sprintf(str, "%.5g", plan.trajectory_.joint_trajectory.points[i].positions[n]);
          line.append(str);
          line.append(" ");
        }
        char dur[20];
        sprintf(dur, "%g", plan.trajectory_.joint_trajectory.points[i].time_from_start.toSec());
        line.append(dur);
        writeNumberStream<<"\n"<<line;
        
      }
      // writeNumberStream<<"\n"<<curr_num;
      writeNumberStream.close();
    }
    
  private:
  	std::string to_string(int value)
  	{
  		stringstream ss;
  		ss << value;
  		return ss.str();
  	}
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "movement_example1");
  ros::NodeHandle n;
  srand (time(NULL));
  ArmMovement server("ArmMovement");

  /*for(int i=0;i<50;i++){
    server.plan_point_to_screen();
    server.execute_point_to_screen();
    ROS_INFO("Executed Point to Screen Plan %d", i);
    sleep(15);
  }*/
  ros::Subscriber feedback_sub = n.subscribe("feedback", 10, &ArmMovement::feedbackCB, &server);

  ros::spin();
  return 0;
}
