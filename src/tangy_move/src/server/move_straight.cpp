#include <tangy_move/navigationServer.h>

void navigationServer::move_straight(double distance) {
	// start pose
	
	//do {
		geometry_msgs::PoseStamped start;
		start = createPose("base_link", 0, 0, 0, 0, 0, 0);
		start = transformPose("map", start);
		// move close to distance 
		move_straight_loop(distance, start);

        	if(distance > 0) {
			distance -= checkDistTravelled(start);
		} else {
			distance += checkDistTravelled(start);	
		}
		move_straight_loop(distance, start);
	// } while(distance < -0.01 || distance > 0.01);
	// print debug message
	debug_print("Move straight successful");
}

void navigationServer::move_straight_loop(double distance, geometry_msgs::PoseStamped start) {
	// print debug message
	debug_print("Moving straight by " + boost::to_string(distance));

  	// speed at which robot moves forward and backward
	geometry_msgs::Twist move;
	if(distance > 0.4)
		move.linear.x = 0.4;
	else if(distance >-0.4)
		move.linear.x = distance;
	else
		move.linear.x = -0.4;		
	move.linear.y = 0;
	move.angular.x = 0;
	move.angular.y = 0;
	move.angular.z = 0;

	// loop until the robot has travelled the required distance
	double distTravelled = 0;
	while(distTravelled <= fabs(distance)) {
		velCommand.publish(move);
		distTravelled = checkDistTravelled(start);
	}
	move.linear.x = 0.0;
	velCommand.publish(move);	
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
