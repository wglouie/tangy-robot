#include <tangy_move/navigationServer.h>
#include <iostream>
#include <math.h>

void navigationServer::rotate(std::string frame, double goalTheta) {
	// rotate to get close to point
	rotate_loop(frame, goalTheta);

	// print debug message
	debug_print("The robot has succesfully rotated");
}

void navigationServer::rotate_loop(std::string frame, double goalTheta) {
	// print debug message
	debug_print("Rotating to " + boost::to_string(goalTheta) + " in the " + frame + " frame");

	// calculate angle robot needs to turn to on the map
	geometry_msgs::PoseStamped robotPose = getRobotPose("map");
	geometry_msgs::PoseStamped robotRelativePose = getRobotPose(frame);
	double robotTheta = tf::getYaw(robotPose.pose.orientation);
	double robotRelativeTheta = tf::getYaw(robotRelativePose.pose.orientation);
	goalTheta +=  robotTheta -robotRelativeTheta;
	double angle = goalTheta - robotTheta;
	double old_angle = angle;
	angle = atan2(sin(angle), cos(angle));

	// print debug message
	debug_print("Tangy must turn by " + boost::to_string(angle));
	
	// check if robot is within threshold of goalTheta
	geometry_msgs::Twist spin = createTwist(0);

	bool done_rotating = false;
	ros::Duration half_second(0.3);
	double done_time = ros::Time::now().toSec() + half_second.toSec();
	double multiplier = 0.5;
	int count = -1;
	double old_time = ros::Time::now().toSec();
	double current_time = ros::Time::now().toSec();
	ros::Time tf_time = ros::Time::now();
    while(!done_rotating && paused == false) {
	    current_time = ros::Time::now().toSec();
	    // ROS_ERROR("The time of one loop is: %f", current_time - old_time);
	    old_time = current_time;
		if(angle > 0.05 || angle < -0.05) {			
			if(old_angle == angle) {
				multiplier *= 1.3;
			}
			else{
				if(angle < 0.1 && angle > 0) {
					multiplier = 2;
				} else if(angle < 0 && angle > -0.1) {
					multiplier = 4;
				} else {
					multiplier = 0.5;
				}
			}
			float speed;
			if(angle > 0) {
		    		speed = multiplier*angle;
			} else {
		    		speed = multiplier*angle;
			}

			spin = createTwist(speed);
			velCommand.publish(spin);
            		
			// char temp_key;
           		// ROS_INFO("The speed is %f, press a key", speed);
            		// std::cin >> temp_key;			
			// 0.5 seconds more before it can finish
			done_time = ros::Time::now().toSec() + half_second.toSec();
        } else if(ros::Time::now().toSec() > done_time && old_angle == angle) {
			done_rotating = true;
			// stop spinning
			spin = createTwist(0);
			velCommand.publish(spin);
		} else {
			// stop spinning
			spin = createTwist(0);
			velCommand.publish(spin);
		}
		// update angle
		old_angle = angle;
		geometry_msgs::PoseStamped robotPose = getRobotPose("map");
/*
		geometry_msgs::PoseStamped poseFrom = createPose("base_link", 0, 0, 0, 0, 0, 0);

		geometry_msgs::PoseStamped poseTo;

		double time_diff = ros::Time::now().toSec() - tf_time.toSec();
		try {	
			poseFrom.header.stamp = tf_time;
			poseTo.header.stamp = tf_time;
			tfListener.transformPose("map", poseFrom, poseTo);
			robotTheta = tf::getYaw(poseTo.pose.orientation);
			// robotTheta += 0.005*time_diff; 
			tf_time = ros::Time::now();
		} catch (tf::TransformException ex){
			// robotTheta += 0.005*time_diff;
		}
*/
		robotTheta = tf::getYaw(robotPose.pose.orientation);
		angle = goalTheta - robotTheta;
		angle = atan2(sin(angle), cos(angle));
	}
}

void navigationServer::rotate_manual(double angle) {
	// print debug message
	debug_print("Rotate manually by " + boost::to_string(angle));

	// input distance for robot to travel
	geometry_msgs::Twist turn;
	turn.linear.x = 0;
	turn.linear.y = 0;
	turn.angular.x = 0;
	turn.angular.y = 0;
	turn.angular.z = angle;
	velCommand.publish(turn);
}

void navigationServer::face(std::string frame, double goalX, double goalY) {
	// print debug message
	debug_print("Turning to face point (" + boost::to_string(goalX) + boost::to_string(goalY) + ") in the " + frame + " frame");

	// transform point into map frame
	geometry_msgs::PoseStamped goalPose = createPose(frame, goalX, goalY, 0, 0, 0, 0);
	goalPose = transformPose("base_link", goalPose);
	debug_print("Turning to face point (" + boost::to_string(goalPose.pose.position.x) + boost::to_string(goalPose.pose.position.y) + ") in the base_link frame");

	// add the two angles together to get the necessary angle relative to map for robot to face goal
	double yaw = atan2(goalPose.pose.position.y, goalPose.pose.position.x);
	debug_print("Turning " + boost::to_string(yaw*180/3.1415) + " degrees");

	//call for robot to rotate according to angle deltaYaw
	rotate("base_link", yaw);
}
