//============================================================================
// Name        : ik_client.cpp
// Author      : Ananias Paiva
// Copyright   :
// Description : Send end effector position to ik_server
//============================================================================

#include "ik_client.hpp" 

ikClient::ikClient(std::string name):ac("kinematics", true),action_name(name)
{
	//Get connection to a server
	ROS_INFO("%s Waiting For Server...", action_name.c_str());
	//Wait for the connection to be valid
	ac.waitForServer();
	ROS_INFO("%s Got a Server...", action_name.c_str());
}
// Called once when the goal completes
void ikClient::doneCb(const actionlib::SimpleClientGoalState& state, const drrobot_h20_arm_player::ikPoseResultConstPtr& result)
{
	if(state.toString() == "SUCCEEDED")
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ROS_INFO("Result: right_arm [%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f]", result->result[0], result->result[1], result->result[7], result->result[6], result->result[5], result->result[4], result->result[3], result->result[2]);
		ROS_INFO("Result: left_arm [%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f]", result->result[8], result->result[9], result->result[15], result->result[14], result->result[13], result->result[12], result->result[11], result->result[10]);
		ros::shutdown();
	}
	else
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ros::shutdown();
	}
}
// Called once when the goal becomes active
void ikClient::activeCb()
{
	ROS_INFO("Goal just went active...");
}

// Called every time feedback is received for the goal
void ikClient::feedbackCb(const drrobot_h20_arm_player::ikPoseFeedbackConstPtr& feedback)
{
	ROS_INFO("Got Feedback of Progress to Goal:");
	ROS_INFO("right_arm [%d %d %d %d %d %d %d %d]",feedback->feedback[0], feedback->feedback[1], feedback->feedback[2], feedback->feedback[3], feedback->feedback[4], feedback->feedback[5], feedback->feedback[6], feedback->feedback[7]);
	ROS_INFO("left_arm [%d %d %d %d %d %d %d %d]\n",feedback->feedback[8], feedback->feedback[9], feedback->feedback[10], feedback->feedback[11], feedback->feedback[12], feedback->feedback[13], feedback->feedback[14], feedback->feedback[15]);  
}

//Send a goal to the server
void ikClient::send(geometry_msgs::Pose pose,std::string group_name)
{
	drrobot_h20_arm_player::ikPoseGoal newGoal;
	newGoal.group_name = group_name;
	newGoal.pose = pose;
	
	//Once again, have to used boost::bind because you are inside a class
	ac.sendGoal(newGoal, boost::bind(&ikClient::doneCb, this, _1, _2),
				boost::bind(&ikClient::activeCb, this),
				boost::bind(&ikClient::feedbackCb, this, _1));
}

//Send a goal to the server without define the orientation(this is not recommended because it almost never find an IK and if find it can stay in a strange position)
void ikClient::send(float x, float y, float z,std::string group_name)
{
	geometry_msgs::Pose pose;
	
	pose.position.x = x; 
	pose.position.y = y; 
	pose.position.z = z; 
	  
	pose.orientation.x = 0; 
	pose.orientation.y = 0; 
	pose.orientation.z = 0; 
	pose.orientation.w = 1;
	
	send(pose, group_name);
}

//TODO: Use rosparam to send end effector position		
int main (int argc, char **argv)
{

	ros::init(argc, argv, "ik_client");

	ros::NodeHandle nh_;
	
	int num = atoi(argv[1]);

	//Initialize the client
	ikClient client("ik_client");
	
	geometry_msgs::Pose pose;
	
	if(num == 1)
	{		
		pose.position.x = 0.346; 
		pose.position.y = -0.196; 
		pose.position.z = 0.498; 
		  
		pose.orientation.x = 0.205; 
		pose.orientation.y = 0.905; 
		pose.orientation.z = 0.261; 
		pose.orientation.w = -0.266;
		
		client.send(pose,"right_arm");
	}
	
	else if(num == 2)
	{
		pose.position.x = 0.004; 
		pose.position.y = -0.488; 
		pose.position.z = 0.872; 
		  
		pose.orientation.x = 0.007; 
		pose.orientation.y = 0.634; 
		pose.orientation.z = 0.773; 
		pose.orientation.w = -0.007;
		
		client.send(pose,"right_arm");
	}
	
	else if(num == 3)
	{
		pose.position.x = 0.0; 
		pose.position.y = -0.205; 
		pose.position.z = 0.199; 
		  
		pose.orientation.x = 0.5; 
		pose.orientation.y = -0.5; 
		pose.orientation.z = 0.5; 
		pose.orientation.w = -0.5;
		
		client.send(pose,"right_arm");
	}
	
	else if(num == 4)
	{
		pose.position.x = 0.033; 
		pose.position.y = 0.531; 
		pose.position.z = 0.423; 
		  
		pose.orientation.x = -0.094; 
		pose.orientation.y = -0.676; 
		pose.orientation.z = -0.011; 
		pose.orientation.w = 0.731;
		
		client.send(pose,"left_arm");	
	}
	
	else if(num == 5)
	{
		client.send(0.346, -0.196, 0.498, "right_arm");
	}
	
	else if(num == 6)
	{
		client.send(0.004, -0.488, 0.872, "right_arm");
	}
	
	else if(num == 7)
	{
		client.send(0.0, -0.205, 0.199, "right_arm");
	}
	
	else if(num == 8)
	{
		client.send(0.033, 0.531, 0.423, "left_arm");
	}				
	
	ros::spin();


	return 0;
}
