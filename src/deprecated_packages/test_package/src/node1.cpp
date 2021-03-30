#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <test_package/nodeClass.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "node1");
	ros::NodeHandle nh;
	ros::Publisher string_pub = nh.advertise<std_msgs::String>("test_string", 1000);
	ros::Publisher string_pub2 = nh.advertise<std_msgs::String>("test_string", 1000);

	nodeClass nodeObject;
	nodeClass nodeObject2;
	ros::Subscriber nodeSub = nh.subscribe("test_string", 10, &nodeClass::nodeCb, &nodeObject);
	ros::Subscriber nodeSub2= nh.subscribe("test_string", 10, &nodeClass::nodeCb, &nodeObject2);


	int count = 0;
	ros::Rate loop_rate(100);
	sleep(1);
	while (ros::ok()) {
		if(count%2 == 0) {
			std_msgs::String msg;
			std::stringstream ss;
			ss << (int) (count);
			msg.data = ss.str();
			string_pub.publish(msg);
		} 
			std_msgs::String msg;
			std::stringstream ss;
			ss << (int) (count);
			msg.data = ss.str();
			string_pub2.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}
	
}
