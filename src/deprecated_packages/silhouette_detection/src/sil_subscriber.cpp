#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tangy_move.h>
#include <stdlib.h>
#include <cmath>

float theta = 0; // degrees
float distance = 0; // meters to move

void chatterCallbackDistance(const std_msgs::String::ConstPtr& msg)
{
	//ROS_INFO("I heard distance: [%s]", msg->data.c_str());
	distance = atof(msg->data.c_str());
}

void chatterCallbackX(const std_msgs::String::ConstPtr& msg)
{
	//ROS_INFO("I heard x: [%s]", msg->data.c_str());
	theta = atof(msg->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "silhouettes_listener");
	ros::NodeHandle n;

	// initialize move_client class
	navigationClient navClient;
	navClient.initialize(n);

 	ros::Subscriber sub_distance = n.subscribe("sil_distance", 1000, chatterCallbackDistance);
	ros::Subscriber sub_x = n.subscribe("sil_x", 1000, chatterCallbackX);
	// IS THIS HOW YOU DO IT?
	while( ros::ok() )
	{
		if( theta != theta || distance != distance )
		{
			std::cout << "NaN" << std::endl;
		} else if( theta < -0.1 || theta > 0.1 ) // rotate to the right heading first, theta==theta checks if theta is actually a number
		{
			std::cout << "Rotating" << std::endl;			
			navClient.rotate_manual(theta/std::abs(theta)*-10);// theta/theta gives direction, then we want to rotate a set value in that direction
		}else{ // start moving to the right distance to the target once we have the correct heading
			if( distance > 0.1 || distance < -0.1 ) // too far || too close
			{
				std::cout << "Moving straight" << std::endl;
				navClient.move_straight_manual(distance/std::abs(distance)*0.5); 
			}else{ // we are in the correct position, send 0 signals
				std::cout << "Stopping" << std::endl;
				navClient.move_straight_manual(0);
			}
		}
		ros::spinOnce();
	}
  	return 0;
}
