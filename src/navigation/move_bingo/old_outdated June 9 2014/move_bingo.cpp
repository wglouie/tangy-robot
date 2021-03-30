#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <queue>
#include <algorithm>
#include <iterator>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>



#define PI 3.14159265

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SimpleNavigationGoals{
public:
	MoveBaseClient ac;
	double current_x, current_y, goal_x, goal_y, dist_x_goal, dist_y_goal, dist_from_goal;

	//angular z,w
	double current_orientation_z, current_orientation_w;
	double goal_orientation_z, goal_orientation_w;
	double current_angle;
	double goal_angle;
	double orientation_distance;

	bool isDone;
	std::queue<double> locations[2];

	SimpleNavigationGoals():
	//Set up the client. It's publishing to topic "move_base", and is set to auto-spin
	ac("move_base", true)
	{
		//wait for the action server to come up
		while(!ac.waitForServer(ros::Duration(5.0)))
		{
			ROS_INFO("Waiting for the move_base action server to come up");
		}
	}

	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResult::ConstPtr& result)
	{
		std::string line; 

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			std::string message = "Hooray, the base moved reached the goal - "; 
			ROS_INFO(message.c_str());
			line = "1 - Using its own tolerance.\n";
		}
		else
		{
			std::string message = "The base failed to move to the goal for some reason - "; 
				
			ROS_INFO(message.c_str());
			line = "0\n";

			if(isDone == true)
			line = "1 - Using my tolerance.\n";
		}

		//write in the file
		//std::ofstream outfile;
		//outfile.open("/home/tangerine/ros/tangy/src/navigation/navigation_experiment/experiment/navigation_experiment.txt", 			std::ios_base::app);
		//outfile << line;
		//outfile.close();
	}

	// Called once when the goal becomes active
	void activeCb()
	{
		ROS_INFO("Goal just went active...");
		isDone = false;

		while(!locations[0].empty())
			locations[0].pop();

		while(!locations[1].empty())
			locations[1].pop();
	}
	
	// Called every time feedback is received for the goal
	void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
	{
		// Get the current position
		current_x = feedback->base_position.pose.position.x;
		current_y = feedback->base_position.pose.position.y;

		// Get the current orientation
		current_orientation_z = feedback->base_position.pose.orientation.z;
		current_orientation_w = feedback->base_position.pose.orientation.w;

		// Store feedback for later sampling
		locations[0].push(current_x);
		locations[1].push(current_y);

		// Calculate the current distance (x,y) from the goal
		dist_x_goal = goal_x - current_x;
		dist_y_goal = goal_y - current_y; 

		dist_from_goal = sqrt(dist_x_goal*dist_x_goal + dist_y_goal*dist_y_goal);

		ROS_INFO("Current position: x = [%lf], y = [%lf]\n"
			"Goal position: x = [%lf], y = [%lf]\n"
			"Distance from goal: [%lf]\n"
			"Orientation: current = [%.2lf] degrees, goal = [%.2lf] degrees\n"
			"Orientation distance = [%.2lf] degrees\n", current_x, current_y, goal_x, goal_y, dist_from_goal,current_orientation_z, current_orientation_w, current_angle, goal_angle, orientation_distance);


		// Calculate the current distance (orientation) from the goal
		getAngles();

		orientation_distance = goal_angle - current_angle; 

		

		
		  
		if(dist_from_goal <= 0.15 && orientation_distance <= 15.0 && orientation_distance >= -15.0)
		{

			isDone = true;
			ac.cancelGoal();
			ROS_INFO("I'm within the tolerance.");
		}
		
 
		/*  
		if(locations[0].size() > 100 && locations[1].size() > 100)
		{
			if(current_x == locations[0].front() && current_y == locations[1].front())
			//instead of cancelGoal, we could randomly turn the direction for like 15 degrees, this might activate the motion again.			

			//feedback->base_position.pose.orientation.z+10;

			//ac.cancelGoal();

			locations[0].pop();
			locations[1].pop();
			ROS_INFO("I'm stuck here for 100 frames.");
		}
*/
	}

	//Send a goal to the server
	void send(const move_base_msgs::MoveBaseGoal goal)
	{		
		goal_x = goal.target_pose.pose.position.x;
		goal_y = goal.target_pose.pose.position.y;

		goal_orientation_z = goal.target_pose.pose.orientation.z;
		goal_orientation_w = goal.target_pose.pose.orientation.w;

		//Once again, have to used boost::bind because you are inside a class
		ac.sendGoal(goal, boost::bind(&SimpleNavigationGoals::doneCb, this, _1, _2),
               	boost::bind(&SimpleNavigationGoals::activeCb, this),
		boost::bind(&SimpleNavigationGoals::feedbackCb, this, _1));	
	}

	void getAngles()
	{
		// Get goal angle
		if(goal_orientation_z >= 0)
		goal_angle = 2*acos(goal_orientation_w) * 180.0/PI;

		if(goal_orientation_z < 0 && goal_orientation_w >= 0)
		goal_angle = 360.0 + (2*asin(goal_orientation_z) * 180.0/PI);

		if(goal_orientation_z < 0 && goal_orientation_w < 0)
		goal_angle = 360.0 - (2*acos(goal_orientation_w) * 180.0/PI);

		// Get current angle
		if(current_orientation_z >= 0)
		current_angle = 2*acos(current_orientation_w) * 180.0/PI;

		if(current_orientation_z < 0 && current_orientation_w >= 0)
		current_angle = 360.0 + (2*asin(current_orientation_z) * 180.0/PI);

		if(current_orientation_z < 0 && current_orientation_w < 0)
		current_angle = 360.0 - (2*acos(current_orientation_w) * 180.0/PI);
	}
};

			
int main (int argc, char **argv)
{
	ros::init(argc, argv, "simple_navigation_goals");
 	

	double locations[1][7] =
	{
		//{0.800,0.000,0.000,0.000,0.000,-0.785,0.692},
		//{0.000,0.000,0.000,0.000,0.000,0.000,1.000},
		{32.000,9.000,0.000,0.000,0.000,0.000,1.000},
		//{-0.400,-0.010,0.000,0.000,0.000,0.000,1},
		//{4.249,3.377,0.000,0.000,-0.028,0.000,1.586},
		//{1.287, 1.021, 0.000, 0.000, 0.000, 0.177, 0.984},
		//{3.889, 1.391, 0.000, 0.000, 0.000, 0.441, 0.898},
		//{4.481, 4.113, 0.00, 0.000, 0.000, -0.779, 0.627},
		//{3.149, 1.266, 0.000,0.000, 0.000, 1.000, -0.017},
		//{00.325, 0.710, 0.000, 0.000, 0.000, 0.992, -0.130}
	//	{0.000, 1.000, 0.000, 0.000, 0.000, 1.000, 0.000},
		//{1.250, 0.000, 0.000, 0.000, 0.000, 1.000, 1.000},
		//{2.0, 1.250, 0.000, 0.000, 0.000, 1.000, 0.000},
	};	


	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::StrParameter str_param;
	dynamic_reconfigure::Config conf;

	std::string navfn = "navfn/NavfnROS";
	std::string carrot_planner = "carrot_planner/CarrotPlanner";

	//Initialize the client
	SimpleNavigationGoals client;

	for(int id = 0; id < 1; ++id)
	{
		//std::string voicecommand = "echo \"Next move\" | festival --tts";
		//system(voicecommand.c_str());


		printf("Starting new navigation to a mystery location");
		move_base_msgs::MoveBaseGoal goal;

		goal.target_pose.header.frame_id = "/map";
		goal.target_pose.header.stamp = ros::Time::now();

/*		goal.target_pose.pose.position.x = 1;
		goal.target_pose.pose.position.y = 0;
		goal.target_pose.pose.position.z = 0;
		goal.target_pose.pose.orientation.x = 0;
		goal.target_pose.pose.orientation.y = 0;
		goal.target_pose.pose.orientation.z = -0.785;
		goal.target_pose.pose.orientation.w = 0.692;
*/
		goal.target_pose.pose.position.x = locations[id][0]-1.25;
		//get close to the goal position, let carrot planner approach
		goal.target_pose.pose.position.y = locations[id][1];
		goal.target_pose.pose.position.z = locations[id][2];
		goal.target_pose.pose.orientation.x = 0;//locations[id][3];
		goal.target_pose.pose.orientation.y = 0;//locations[id][4];
		goal.target_pose.pose.orientation.z = 0;//locations[id][5];
		goal.target_pose.pose.orientation.w = 1;//locations[id][6];	  
		
		ROS_INFO("Sending goal");
		client.send(goal);      
		client.ac.waitForResult();
		printf("now finished point %d of 2 points/n", id) ;	
	}
	
	//std::string voicecommand = "echo \"I am getting dizzy! I am done with it!\" | festival --tts";
	//system(voicecommand.c_str());
	printf("print stuff - now attempting to change parameters with dynamic reconfigure\n");
/*	
	std::string Something = " ";
	something = 
	double aa;
	aa = getparam();
*/

	str_param.name = "base_global_planner";
	str_param.value = carrot_planner;
	conf.strs.push_back(str_param);

	srv_req.config = conf;

	ros::service::call("/move_base/set_parameters", srv_req, srv_resp);	

	double_param.name = "planner_frequency";
	double_param.value = 5.0;
	conf.doubles.push_back(double_param);

	srv_req.config = conf;

	ros::service::call("/move_base/set_parameters", srv_req, srv_resp);	

	//planner_frequency (double, 3.0)


	printf("print stuff - now attempting to move to final point\n");

		move_base_msgs::MoveBaseGoal goal;

		goal.target_pose.header.frame_id = "/map";
		goal.target_pose.header.stamp = ros::Time::now();


		goal.target_pose.pose.position.x = locations[0][0];
		goal.target_pose.pose.position.y = locations[0][1];
		goal.target_pose.pose.position.z = locations[0][2];
		goal.target_pose.pose.orientation.x = 0;//locations[id][3];
		goal.target_pose.pose.orientation.y = 0;//locations[id][4];
		goal.target_pose.pose.orientation.z = 0;//locations[id][5];
		goal.target_pose.pose.orientation.w = 1;//locations[id][6];

		ROS_INFO("Sending goal");
		client.send(goal);      
		client.ac.waitForResult();

	printf("print stuff - now attempting to revert parameters back to original configuration\n");

	str_param.name = "base_global_planner";
	str_param.value = navfn;
	conf.strs.push_back(str_param);

	srv_req.config = conf;

	ros::service::call("/move_base/set_parameters", srv_req, srv_resp);

	double_param.name = "planner_frequency";
	double_param.value = 0;
	conf.doubles.push_back(double_param);

	srv_req.config = conf;

	ros::service::call("/move_base/set_parameters", srv_req, srv_resp);	

	

	return 0;
}

