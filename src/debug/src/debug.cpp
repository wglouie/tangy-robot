#include <ros/ros.h>
#include <std_msgs/String.h>

void debug_moveCb(const std_msgs::StringConstPtr& str) {
	ROS_INFO("Move: %s", str->data.c_str());
}


void debug_bingoCb(const std_msgs::StringConstPtr& str) {
	
	ROS_INFO("Bingo: %s", str->data.c_str());
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "simple_navigation_goals");
	ros::NodeHandle nh;
	ros::Subscriber debug_move_sub = nh.subscribe("debug_move", 10, debug_moveCb);
	ros::Subscriber debug_bingo_sub = nh.subscribe("debug_bingo", 10, debug_bingoCb);
	ros::spin();
}
