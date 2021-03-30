#include <tangy_move/tangy_move.h>

navigationClient::navigationClient()
{

}

navigationClient::~navigationClient() {

}


void navigationClient::initialize(ros::NodeHandle n) {
	ROS_INFO("initializing navigation_client");
	nh = n;
	navServerCmdPub = n.serviceClient<tangy_move::navServerCmd>("navServerCmd");
}
