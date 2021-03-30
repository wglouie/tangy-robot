#include <help_indicators/help_indicators.h>

// convert new_help_indicators from remote_kinect frame to base_link frame before adding it to the help_indicators list
/*
void Help::transform_help_indicator(help_indicators::triangle &help_indicator) {
	geometry_msgs::PoseStamped kinect_frame_XYZ;
	geometry_msgs::PoseStamped base_link_frame_XYZ;
	kinect_frame_XYZ.header.frame_id = "remote_kinect";
	kinect_frame_XYZ.pose.position.x = help_indicator.x;
	kinect_frame_XYZ.pose.position.y = help_indicator.y;
	kinect_frame_XYZ.pose.position.z = help_indicator.z;
	kinect_frame_XYZ.pose.orientation.x = 0;
	kinect_frame_XYZ.pose.orientation.y = 0;
	kinect_frame_XYZ.pose.orientation.z = 0;
	kinect_frame_XYZ.pose.orientation.w = 1;
	kinect_frame_XYZ.header.stamp = ros::Time::now();
	base_link_frame_XYZ = transformPose("map", kinect_frame_XYZ);

	help_indicator.x = base_link_frame_XYZ.pose.position.x;
	help_indicator.y = base_link_frame_XYZ.pose.position.y;
	help_indicator.z = base_link_frame_XYZ.pose.position.z;
}
*/
