#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <test_package/nodeClass.h>

void nodeClass::nodeCb(const std_msgs::StringConstPtr& str) {
	std::cout << "The string is: [" << str->data << "]" << std::endl;
}
