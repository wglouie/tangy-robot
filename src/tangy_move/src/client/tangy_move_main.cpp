#include <ros/ros.h>
#include <tangy_move/tangy_move.h>
#include <iostream>
int main(int argc, char **argv) {
	ros::init(argc, argv, "tangy_move_main");
	ros::NodeHandle nh;

	navigationClient navClient;
	navClient.initialize(nh);

	ROS_INFO("STARTING CLIENT");
/*'
	navClient.move_straight(0.4);

	navClient.move("map", 1, 0.9);
	navClient.move("map", 1, -0.9);

	while(ros::ok()) {
		sleep(1);
		ROS_INFO("Rotating to PI");
		navClient.rotate("map", 3.14150/2);
		sleep(1);
		ROS_INFO("Rotating to 0");
		navClient.rotate("map", 0);
	}
*/

	return 0;
}
