//============================================================================
// Name        : ik_server_planned.cpp
// Author      : Ananias Paiva
// Copyright   :
// Description : Receives a position, find an ik solution and plann a 
//				 movement avoiding obstacles
//============================================================================

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/move_group_interface/move_group.h>

//Position message
#include <geometry_msgs/Pose.h>
#include <drrobot_h20_arm_player/ArmCmd.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/DisplayTrajectory.h>

//Include headers for ROS service
#include <actionlib/server/simple_action_server.h>
#include <drrobot_h20_arm_player/ikPoseAction.h>

//Constants
#define pi 3.14159265

using namespace std;

class ikServer
{
private:
	ros::NodeHandle nh_;
	ros::Publisher cmd_arm_pub;	
	ros::Subscriber planner_sub;
	
	/*Action library variables*/
	actionlib::SimpleActionServer<drrobot_h20_arm_player::ikPoseAction> as_;
	std::string action_name_;
	drrobot_h20_arm_player::ikPoseFeedback feedback_;					
	drrobot_h20_arm_player::ikPoseResult result_;
	
	//Position messages
	geometry_msgs::Pose pose;
	std::string group_name;
	drrobot_h20_arm_player::ArmCmd cmdarm_;
		
	//Angs and commands
	float ang[16];
	float c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16;
	float planned_ang[6];
	int checker;
	bool finish;
	
	//Robot model and kinematics variables
	robot_model_loader::RobotModelLoader *robot_model_loader;
	robot_model::RobotModelPtr kinematic_model;
	robot_state::RobotStatePtr kinematic_state;
	robot_state::JointStateGroup* joint_state_group_right;
	robot_state::JointStateGroup* joint_state_group_left;
	move_group_interface::MoveGroup *right_arm_group;
	move_group_interface::MoveGroup *left_arm_group;
	
public:
	ikServer(std::string name): as_(nh_, name, boost::bind(&ikServer::goalCB, this, _1), false), action_name_(name)
	{
		//register the goal and feeback callbacks
		//as_.registerGoalCallback(boost::bind(&ikServer::goalCB, this, _1));
		//as_.registerPreemptCallback(boost::bind(&ikServer::preemptCB, this));
		
		//Incialize model and kinematics
		/* Load the robot model */		
		robot_model_loader = new robot_model_loader::RobotModelLoader("robot_description");
		/* Get a shared pointer to the model */
		kinematic_model = robot_model_loader->getModel();
		/* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
		kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
		/* Set all joints in this state to their default values */
		kinematic_state->setToDefaultValues();
		/* Get the configuration for the joints in the given group*/
		joint_state_group_right = kinematic_state->getJointStateGroup("right_arm");
	  	joint_state_group_left = kinematic_state->getJointStateGroup("left_arm");
	  	/* Set the joint to their inicial position */
	  	joint_state_group_right->setToDefaultState("inicial_right");
	  	joint_state_group_left->setToDefaultState("inicial_left");
	  	/* Get the configuration for the joints in the given group for the move_group */
	  	right_arm_group = new move_group_interface::MoveGroup("right_arm");
	  	left_arm_group = new move_group_interface::MoveGroup("left_arm");
		
		//Inicial position commands
		c1 = 1700;
		c2 = 1269;
		c3 = 1625;
		c4 = 1545;
		c5 = 1710;
		c6 = 2000;
		c7 = 2300;
		c8 = 760;
		c9 = 1290;
		c10 = 1720;
		c11 = 1570;
		c12 = 1520;
		c13 = 1940;
		c14 = 2010;
		c15 = 1240;
		c16 = 2200;
		
		ang[0] = -1.5707;
		ang[1] = -1.5707;
		ang[2] = 0;
		ang[3] = -1.5707;
		ang[4] = -1.5707;
		ang[5] = 0;
		ang[6] = 1.5707;		
		ang[7] = 0;
		ang[8] = -1.5707;
		ang[9] = -1.5707;
		ang[10] = 0;
		ang[11] = -1.5707;
		ang[12] = -1.5707;
		ang[13] = 0;
		ang[14] = 1.5707;		
		ang[15] = 0;
		
		checker = 0;
		
		//Publish and start server
		cmd_arm_pub = nh_.advertise<drrobot_h20_arm_player::ArmCmd>("cmd_arm", 1);
		as_.start();
		
		//Subscribe to the planner
		planner_sub = nh_.subscribe<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, boost::bind(&ikServer::PlannedCB, this, _1));
			
	}
	
	//Callback that get all movements planned and send them to be publish
	void PlannedCB(const moveit_msgs::DisplayTrajectory::ConstPtr& planned_pose)
	{
		//get all poses that the planner calculate
		for(int i=0;i < planned_pose->trajectory[0].joint_trajectory.points.size(); i++)
		{
			//For each pose get all the joint value
			for(int j=0; j < planned_pose->trajectory[0].joint_trajectory.points[i].positions.size(); j++)
			{
				planned_ang[j] = planned_pose->trajectory[0].joint_trajectory.points[i].positions[j];
			}
			if(checker == 1)
			{
				for(int k=0;k < planned_pose->trajectory[0].joint_trajectory.points[i].positions.size(); k++)
				{
					ang[k+2] = planned_ang[k];
				}
			}
			else if(checker == 2)
			{
				for(int l=0;l < planned_pose->trajectory[0].joint_trajectory.points[i].positions.size(); l++)
				{
					ang[l+10] = planned_ang[l];					
				}
			}		
			sendServoCtrlAllAng(radToDegree(ang[0]), radToDegree(ang[1]), radToDegree(ang[7]), radToDegree(ang[6]), radToDegree(ang[5]), radToDegree(ang[4]), radToDegree(ang[3]), radToDegree(ang[2]), radToDegree(ang[8]), radToDegree(ang[9]), radToDegree(ang[15]), radToDegree(ang[14]), radToDegree(ang[13]), radToDegree(ang[12]), radToDegree(ang[11]), radToDegree(ang[10]), 100, false);
			
					
		}
		finish = true;
	}
	
	//TODO:This function calculate IK soulution just to see if it exist, because if a position is send to the planner
	//and there is no solution for it the planner stops and never back to this sever.
	//With a better understand of the planner this function can be erased because it is a big waste of performance
	bool checkIk(geometry_msgs::Pose pose,std::string group_name)
	{

		const geometry_msgs::Pose &position = pose;
		/* Check if the given position have a valid IK before sent it to the planner */
		bool found_ik;
		/* Get the names of the joints*/
	  	const std::vector<std::string> &right_joint_names = joint_state_group_right->getJointModelGroup()->getJointModelNames();
	  	const std::vector<std::string> &left_joint_names = joint_state_group_left->getJointModelGroup()->getJointModelNames();	  	
	  	
		/* Get the joint states*/
	  	std::vector<double> right_joint_values;
	  	std::vector<double> left_joint_values;
	  	//Getting joint angles
		joint_state_group_right->getVariableValues(right_joint_values);
		joint_state_group_left->getVariableValues(left_joint_values);
		
		/* Get joint names and positison to restore them later */
		sensor_msgs::JointState curr_right_position;
		sensor_msgs::JointState curr_left_position;
		/* Rezise name and position arrays and fill them */
	  	curr_right_position.name.resize(right_joint_names.size());
	  	curr_right_position.position.resize(right_joint_values.size());
	  	curr_left_position.name.resize(left_joint_names.size());
	  	curr_left_position.position.resize(left_joint_values.size());
	
	  	for(int i=0;i < right_joint_names.size();i++)
	  	{
	  		curr_right_position.name[i] = right_joint_names[i];
	  		curr_right_position.position[i] = right_joint_values[i];
	  	}
	  	for(int j=0;j < left_joint_names.size();j++)
	  	{
	  		curr_left_position.name[j] = left_joint_names[j];
	  		curr_left_position.position[j] = left_joint_values[j];
	  	}	  	
	  	
	  	if(group_name == "right_arm") 
	  	{
	  		found_ik = joint_state_group_right->setFromIK(position, 10, 0.1, false);	  		
	  	}
	  	else if (group_name == "left_arm") 
	  	{
	  		found_ik = joint_state_group_left->setFromIK(position, 10, 0.1, false);  		
	  	}
	  	
	  	/* Restore previous position */
	  	const sensor_msgs::JointState &restore_right_position = curr_right_position;
	  	const sensor_msgs::JointState &restore_left_position = curr_left_position;
	  	joint_state_group_right->setVariableValues(restore_right_position);
	  	joint_state_group_left->setVariableValues(restore_left_position);
	  	
		if(found_ik == false) return false;
		return true;
		
	}
	
	//Calculate IK solution and plann all the movements avoiding obstacles that can be publish to "moveit_msgs/CollisionObject" (use for tests)
	//and obstacles seen by a point cloud sensor
	bool calcIk(geometry_msgs::Pose pose,std::string group_name)
	{ 	
		
		std::vector<geometry_msgs::Pose> position(1);
		position.at(0) = pose;
		finish = false;
		
		if(checkIk(pose,group_name) == false) return false;	  		

	  	/*Calculate Ik for given position*/
	  	if(group_name == "right_arm") 
	  	{
	  		right_arm_group->setPoseTargets(position,"wrist_right_arm_x_revolute");
	  		checker = 1;
	  		right_arm_group->move();	  		
	  	}
	  	else if (group_name == "left_arm") 
	  	{
	  		left_arm_group->setPoseTargets(position,"wrist_left_arm_x_revolute");
	  		checker = 2;
	  		left_arm_group->move();		
	  	}
	
		for(int k = 0; k < 16; k++)
		{
			result_.result[k] = ang[k];
		}
		
		while(true) 
		{
			if(finish == true) return true;
		}
		
		return false;
	  	
	}
	
	void goalCB(const drrobot_h20_arm_player::ikPoseGoalConstPtr &goal)
	{
		//Accept the new goal
		pose = goal->pose;
		group_name = goal->group_name;
		bool goalCheck = calcIk(pose, group_name);
		
		if(goalCheck == true)
		{
			ROS_INFO("Successfully reach the point");
			as_.setSucceeded(result_);
		}
		else if (goalCheck == false)
		{
			ROS_INFO("Can't reach the point");
			as_.setAborted(result_);
		}
	}
	
	void preemptCB()
	{
		ROS_INFO("%s: Preempted", action_name_.c_str());
		// set the action state to preempted
		as_.setPreempted();
	}
   	
   	//This funciton transform commands in angles
   	//TODO: Arm driver has same function, so IF you DON'T want to create a DrRobot_Arm_Player node you can instatiate the driver and use it function making tangy moves (it could improve tangy response for commands)
   	//(keep in mind that using directly the driver you can't use DrRobot_Arm_Player, so if you need the player for other node dont change this function)
   	void sendServoCtrlAllAng(int ang1, int ang2, int ang3, int ang4, int ang5, int ang6, int ang7, int ang8, int ang9, int ang10, int ang11, int ang12, int ang13, int ang14, int ang15, int ang16, int vel, bool speed)
	{
		int i;
		float cmd1, cmd2, cmd3, cmd4, cmd5, cmd6, cmd7, cmd8, cmd9, cmd10, cmd11, cmd12, cmd13, cmd14, cmd15, cmd16;


		for (i = 0; i < 16; i++)
		{
			if (i == 0)
			{
				if (ang1 < -90)
				{
					cmd1 = 1700 + (ang1 - (-90))*((1700.0 - 590.0)/(-90.0 - (30.0)));
				}
				else
				{
					cmd1 = 1700 + (ang1 - (-90))*((2110.0 - 1700.0)/(-150.0 - (-90.0)));
				}
			}
			if (i == 1)
			{
				if (ang2 < -90)
				{
					cmd2 = 1269 + (ang2 - (-90))*((1269.0 - 950.0)/(-90.0 - (-60.0)));
				}
				else
				{
					cmd2 = 1269 + (ang2 - (-90))*((2400.0 - 1269.0)/(-210.0 - (-90.0)));
				}
			}
			if (i == 2)
			{
				if (ang3 < 0)
				{
					cmd3 = 1625 + (ang3 - (0))*((2250.0 - 1625.0)/(-60.0 - (0.0)));
				}
				else
				{
					cmd3 = 1625 + (ang3 - (0))*((1625.0 - 680.0)/(0.0 - (85.0)));
				}
			}
			if (i == 3)
			{
				if (ang4 < 90)
				{
					cmd4 = 1545 + (ang4 - (90))*((2350.0 - 1545.0)/(0.0 - (90.0)));
				}
				else
				{
					cmd4 = 1545 + (ang4 - (90))*((1545.0 - 580.0)/(90.0 - (180.0)));
				}
			}
			if (i == 4)
			{
				if (ang5 < 0)
				{
					cmd5 = 1710 + (ang5 - (0))*((2110.0 - 1710.0)/(-129.0 - (0.0)));
				}
				else
				{
					cmd5 = 1710 + (ang5 - (0))*((1710.0 - 1400.0)/(0.0 - (90.0)));
				}
			}
			if (i == 5)
			{
				if (ang6 < -90)
				{
					cmd6 = 2000 + (ang6 - (-90))*((2000.0 - 1670.0)/(-90.0 - (-195.0)));
				}
				else
				{
					cmd6 = 2000 + (ang6 - (-90))*((2350.0 - 2000.0)/(15.0 - (-90.0)));
				}
			}
			if (i == 6)
			{
				if (ang7 < -90)
				{
					cmd7 = 2300 + (ang7 - (-90))*((2300.0 - 1460.0)/(-90.0 - (-200.0)));
				}
				else
				{
					cmd7 = 2300 + (ang7 - (-90))*((2400.0 - 2300.0)/(-85.0 - (-90.0)));
				}
			}
			if (i == 7)
			{
				if (ang8 < 0)
				{
					cmd8 = 760 + (ang8 - (0))*((760.0 - 525.0)/(0 - (-20.0)));
				}
				else
				{
					cmd8 = 760 + (ang8 - (0))*((2450.0 - 760.0)/(192.0 - (0)));
				}
			}
			if (i == 8)
			{
				if (ang9 > -90)
				{
					cmd9 = 1290 + (ang9 - (-90))*((1290.0 - 960.0)/(-90.0 - (-30.0)));
				}
				else
				{
					cmd9 = 1290 + (ang9 - (-90))*((2380.0 - 1290.0)/(-210.0 - (-90.0)));
				}
			}
			if (i == 9)
			{
				if (ang10 > -90)
				{
					cmd10 = 1720 + (ang10 - (-90))*((1720.0 - 565.0)/(-90.0 - (30.0)));
				}
				else
				{
					cmd10 = 1720 + (ang10 - (-90))*((2015.0 - 1720.0)/(-120.0 - (-90.0)));
				}
			}
			if (i == 10)
			{
				if (ang11 > 0)
				{
					cmd11 = 1570 + (ang11 - (0))*((1570.0 - 650.0)/(0 - (-60.0)));
				}
				else
				{
					cmd11 = 1570 + (ang11 - (0))*((2250.0 - 1625.0)/(85.0 - (0)));
				}
			}
			if (i == 11)
			{
				if (ang12 > 90)
				{
					cmd12 = 1520 + (ang12 - (90))*((2430.0 - 1520.0)/(180.0 - (90.0)));
				}
				else
				{
					cmd12 = 1520 + (ang12 - (90))*((1520.0 - 580.0)/(90.0 - (0.0)));
				}
			}
			if (i == 12)
			{
				if (ang13 > 0)
				{
					cmd13 = 1940 + (ang13 - (0))*((2255.0 - 1940.0)/(90.0 - (0.0)));
				}
				else
				{
					cmd13 = 1940 + (ang13 - (0))*((1940.0 - 1510.0)/(0.0 - (-129.0)));
				}
			}
			if (i == 13)
			{
				if (ang14 > -90)
				{
					cmd14 = 2010 + (ang14 - (-90))*((2010.0 - 1670.0)/(-90.0 - (15.0)));
				}
				else
				{
					cmd14 = 2010 + (ang14 - (-90))*((2350.0 - 2010.0)/(-195.0 - (-90.0)));
				}
			}
			if (i == 14)
			{
				if (ang15 > -90)
				{
					cmd15 = 1240 + (ang15 - (-90))*((1240.0 - 1170.0)/(-90.0 - (-85.0)));
				}
				else
				{
					cmd15 = 1240 + (ang15 - (-90))*((2015.0 - 1240.0)/(-200.0 - (-90)));
				}
			}
			if (i == 15)
			{
				if (ang16 > 0)
				{
					cmd16 = 2200 + (ang16 - (0))*((2200.0 - 560.0)/(0 - (192.0)));
				}
				else
				{
					cmd16 = 2200 + (ang16 - (0))*((2450.0 - 2200.0)/(-20.0 - (0)));
				}
			}
		}
		
		angBreaker(cmd1, cmd2, cmd3, cmd4, cmd5, cmd6, cmd7, cmd8, cmd9, cmd10, cmd11, cmd12, cmd13, cmd14, cmd15, cmd16, vel, speed); 
	}
	
	float radToDegree(float rad)
	{
		float degree;
		
		degree = (rad*180)/(2*1.5707);
		
		return degree;
	}
	
	//When you find an IK solution using this class you dont have a planner, so instead of have an unique movement this function break the command and send small movements
	void angBreaker(int a1, int a2, int a3, int a4, int a5, int a6, int a7, int a8, int a9, int a10, int a11, int a12, int a13, int a14, int a15, int a16, int vel, bool speed)
	{
		vel = vel/100;
		float dif1 = 0, dif2 = 0, dif3 = 0, dif4 = 0, dif5 = 0, dif6 = 0, dif7 = 0, dif8 = 0, dif9 = 0, dif10 = 0, dif11 = 0, dif12 = 0, dif13 = 0, dif14 = 0, dif15 = 0, dif16 = 0;
        
		
		if(c1 != a1 || c2 != a2 || c3 != a3 || c4 != a4 || c5 != a5 || c6 != a6 || c7 != a7 || c8 != a8 || c9 != a9 || c10 != a10 || c11 != a11 || c12 != a12 || 
		   c13 != a13 || c14 != a14 || c15 != a15 || c16 != a16)
		{
			dif1 = ((float)(a1 - c1)/(float)vel);
			dif2 = ((float)(a2 - c2)/(float)vel);
			dif3 = ((float)(a3 - c3)/(float)vel);
			dif4 = ((float)(a4 - c4)/(float)vel);
			dif5 = ((float)(a5 - c5)/(float)vel);
			dif6 = ((float)(a6 - c6)/(float)vel);
			dif7 = ((float)(a7 - c7)/(float)vel);
			dif8 = ((float)(a8 - c8)/(float)vel);
			dif9 = ((float)(a9 - c9)/(float)vel);
			dif10 = ((float)(a10 - c10)/(float)vel);
			dif11 = ((float)(a11 - c11)/(float)vel);
			dif12 = ((float)(a12 - c12)/(float)vel);
			dif13 = ((float)(a13 - c13)/(float)vel);
			dif14 = ((float)(a14 - c14)/(float)vel);
			dif15 = ((float)(a15 - c15)/(float)vel);
			dif16 = ((float)(a16 - c16)/(float)vel);

			for (int i = 1; i <= vel; i++)
			{
				c1 = c1 + dif1;
				c2 = c2 + dif2;
				c3 = c3 + dif3;
				c4 = c4 + dif4;
				c5 = c5 + dif5;
				c6 = c6 + dif6;
				c7 = c7 + dif7;
				c8 = c8 + dif8;
				c9 = c9 + dif9;
				c10 = c10 + dif10;
				c11 = c11 + dif11;
				c12 = c12 + dif12;
				c13 = c13 + dif13;
				c14 = c14 + dif14;
				c15 = c15 + dif15;
				c16 = c16 + dif16;
				
				feedback_.feedback[0] = c1;
				feedback_.feedback[1] = c2;
				feedback_.feedback[2] = c3;
				feedback_.feedback[3] = c4;
				feedback_.feedback[4] = c5;
				feedback_.feedback[5] = c6;
				feedback_.feedback[6] = c7;
				feedback_.feedback[7] = c8;
				feedback_.feedback[8] = c9;
				feedback_.feedback[9] = c10;
				feedback_.feedback[10] = c11;
				feedback_.feedback[11] = c12;
				feedback_.feedback[12] = c13;
				feedback_.feedback[13] = c14;
				feedback_.feedback[14] = c15;
				feedback_.feedback[15] = c16;

				cmdarm_.right_arm[0] = c1;
				cmdarm_.right_arm[1] = c2;
				cmdarm_.right_arm[2] = c3;
				cmdarm_.right_arm[3] = c4;
				cmdarm_.right_arm[4] = c5;
				cmdarm_.right_arm[5] = c6;
				cmdarm_.right_arm[6] = c7;
				cmdarm_.right_arm[7] = c8;
				cmdarm_.left_arm[0] = c9;
				cmdarm_.left_arm[1] = c10;
				cmdarm_.left_arm[2] = c11;
				cmdarm_.left_arm[3] = c12;
				cmdarm_.left_arm[4] = c13;
				cmdarm_.left_arm[5] = c14;
				cmdarm_.left_arm[6] = c15;
				cmdarm_.left_arm[7] = c16;
				cmdarm_.vel = 100;
				cmdarm_.speed = speed;
				
				
				cmd_arm_pub.publish(cmdarm_);
				as_.publishFeedback(feedback_);
				
				ros::Duration(0.1).sleep();
			}
		}
	}
};

int main(int argc, char **argv)
{
	ros::init (argc, argv, "kinematics");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	ikServer ikServer_("kinematics");
	ros::spin();
	
	return 0;
}
