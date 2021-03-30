#include <tangy_move/navigationServer.h>

geometry_msgs::Twist navigationServer::createTwist(double theta) {
	geometry_msgs::Twist twist;
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = theta;
	return twist;
}

geometry_msgs::PoseStamped navigationServer::createPose(std::string frame, double x, double y, double z, double ax, double ay, double az) {
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = frame;
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = z;
	pose.pose.orientation.x = ax;
	pose.pose.orientation.y = ay;
	pose.pose.orientation.z = az;
	pose.pose.orientation.w = 1.0;
	pose.header.stamp = ros::Time::now();
	return pose;
}

geometry_msgs::PoseStamped navigationServer::transformPose(std::string frameTo, geometry_msgs::PoseStamped poseFrom) {
	geometry_msgs::PoseStamped poseTo;

	//tf::StampedTransform transform;
	try {
		ros::Time current_time = ros::Time::now();
		poseFrom.header.stamp = current_time;
		poseTo.header.stamp = current_time;
		tfListener.waitForTransform(frameTo, poseFrom.header.frame_id, current_time, ros::Duration(1.0));
		tfListener.transformPose(frameTo, poseFrom, poseTo);
	} catch (tf::TransformException ex){
		debug_print(ex.what());
	}
	return poseTo;
}

geometry_msgs::PoseStamped navigationServer::getRobotPose(std::string frame) {
	//fill baseLink with values of robots position and angle (as origin at 0,0)
	geometry_msgs::PoseStamped robotPose = createPose("base_link", 0, 0, 0, 0, 0, 0);
	robotPose = transformPose(frame, robotPose);
	return robotPose;
}
