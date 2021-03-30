#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <user_tracking/UserTrackingAction.h>
#include <user_tracking/user_tracking_client.h>
			
int main (int argc, char **argv)
{
	//ros::init(argc, argv, "test_userTracking_client");

	ros::init(argc, argv, "user_tracking_client");
	//Usage check to make sure the client is being used properly

	ros::NodeHandle nh;

	std::string user; 

        nh.getParam("user", user);
        ROS_INFO("System will track user '%s'.", user.c_str());
	

	//Initialize UserTracking client
	UserTrackingClient client(ros::this_node::getName());

    	//client.send();
	client.sendGoal(user);

        ROS_INFO("Sent Goal (%s) To Server...", user.c_str());

	ros::spin();


	return 0;
}
