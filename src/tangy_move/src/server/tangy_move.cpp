#include <tangy_move/navigationServer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

void navigationServer::debug_print(std::string string) {
	debug_string.data = string;
	debug_pub.publish(debug_string);
}

navigationServer::navigationServer():
actionClient("move_base", true)
{
	ROS_INFO("Starting navigation client");
	while(!actionClient.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for move_base action server");
	}
	sleep(1);
	paused = false;
}

navigationServer::~navigationServer() {
	debug_print("Closing navigation client");
}


void navigationServer::initialize(ros::NodeHandle n) {
	ROS_INFO("initializing navigation_client");
	nh = n;
	velCommand = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	//debug publisher
	debug_pub = nh.advertise<std_msgs::String>("debug_move", 1000);


	//set up RVIZ marker stuff
	marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 1000);
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "test_shapes";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.6;
	marker.color.r = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration();
	sleep(1);



}
