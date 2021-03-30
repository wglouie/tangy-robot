#include <ros/ros.h>
#include <std_msgs/String.h>
#include <test_package/nodeClass.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "node3");
	ros::NodeHandle nh;
	nodeClass nodeObject;
	ros::Subscriber nodeSub = nh.subscribe("test_string", 1, &nodeClass::nodeCb, &nodeObject);
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
}
