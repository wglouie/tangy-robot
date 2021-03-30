#include <help_indicators/help_indicators.h>

// get the euclidean distance based on the two points given
double Help::get_distance(double x1, double y1, double x2, double y2) {
    double x = x1 - x2;
    double y = y1 - y2;
    double dist = sqrt(x*x+y*y);
    return dist;
}

// compute the number of times seen for a new_help_indicators
int Help::times_seen(help_indicators::triangle new_help_indicator) {
    int count = 0;
    for(int i = 0; i < 20; i++) {
        if(new_help_indicator.seen[i]) {
            count ++;
        }
    } 
    return count;
}


// transform a pose from one frame to another (used for transforming from kinect pose to base_link pose)
/*
geometry_msgs::PoseStamped Help::transformPose(std::string frameTo, geometry_msgs::PoseStamped poseFrom) {
	geometry_msgs::PoseStamped poseTo;
	try {
		ros::Time current_time = ros::Time::now();
		poseFrom.header.stamp = current_time;
		poseTo.header.stamp = current_time;
		while( !(tfListener.waitForTransform(frameTo, poseFrom.header.frame_id, current_time, ros::Duration(1.0))) ){}
		tfListener.transformPose(frameTo, poseFrom, poseTo);
	} catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	return poseTo;
}
*/
