#include <tangy_move/navigationServer.h>

void navigationServer::move_straight(double distance) {

	geometry_msgs::Twist move;
	float dist_travelled = -1;

	// loop until the robot has travelled within a tolerance of the required distance
	bool done = false;
	ros::Duration half_second(0.3);
	double done_time = ros::Time::now().toSec() + half_second.toSec();
	while(!done) {
		// start pose
		geometry_msgs::PoseStamped start;
		start = createPose("base_link", 0, 0, 0, 0, 0, 0);
		start = transformPose("map", start);
		if (distance < -0.01 || distance > 0.01) {
		  	// speed at which robot moves forward and backward
			if(dist_travelled == 0) {
				move.linear.x = 2*move.linear.x;
			} else if(distance > 0.2)
				move.linear.x = 0.7*sqrt(0.2);
			else if(distance >-0.2)
				if(distance > 0)
					move.linear.x = 0.7*sqrt(distance);
				else
					move.linear.x = -0.7*sqrt(-distance);
			else
				move.linear.x = -0.7*sqrt(0.2);		
			move.linear.y = 0;
			move.angular.x = 0;
			move.angular.y = 0;
			move.angular.z = 0;

			// send the move command
			velCommand.publish(move);

			done_time = ros::Time::now().toSec() + half_second.toSec();
		} else if(ros::Time::now().toSec() > done_time && dist_travelled == 0) {
			done = true;
			move.linear.x = 0.0;
			velCommand.publish(move);
		} else {
			move.linear.x = 0.0;
			velCommand.publish(move);
		}

		// update distance values
		dist_travelled = checkDistTravelled(start);
		if(distance > 0) {
			distance -= dist_travelled;
		} else {
			distance += dist_travelled;
		}
	} 

	// stop when within tolerance of distance
	move.linear.x = 0.0;
	velCommand.publish(move);

	// print debug message
	debug_print("Move straight successful");
}

double navigationServer::checkDistTravelled(geometry_msgs::PoseStamped start){
	// return distance the robot has travelled
	geometry_msgs::PoseStamped robotPose = getRobotPose(start.header.frame_id);
	double distX = start.pose.position.x  - robotPose.pose.position.x;
	double distY = start.pose.position.y  - robotPose.pose.position.y;
	double distTravelled = sqrt(distX*distX + distY*distY);
	return distTravelled;
}


void navigationServer::move_straight_manual (double distance) {
	// print debug message
	std::string string =  "Moving straight by ";
	string += boost::to_string(distance);
	debug_print(string);

  	// input distance for robot to travel
	geometry_msgs::Twist move;
	move.linear.x = distance;		
	move.linear.y = 0;
	move.angular.x = 0;
	move.angular.y = 0;
	move.angular.z = 0;
	velCommand.publish(move);
}
