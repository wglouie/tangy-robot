#include <ros/ros.h>
#include <std_msgs/String.h>
#include <test_package/nodeClass.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "node2");
	ros::NodeHandle nh;
	nodeClass nodeObject;
	nodeClass nodeObject2;
	ros::Subscriber nodeSub = nh.subscribe("test_string", 1, &nodeClass::nodeCb, &nodeObject);
	ros::Subscriber nodeSub2= nh.subscribe("test_string", 1, &nodeClass::nodeCb, &nodeObject2);
	ros::Rate loop_rate(100);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
}
