#include <tangy_move/navigationServer.h>

void navigationServer::move(std::string frame, double goalX, double goalY){ //move robot to absolute/preset goals
	move_base_msgs::MoveBaseGoal goal;
	//send a goal for the robot to move to
	goal.target_pose = createPose(frame, goalX, goalY, 0, 0, 0, 0);
	goal.target_pose = navigationServer::transformPose("map", goal.target_pose);
	
	// send marker to RVIZ
	marker.pose = goal.target_pose.pose;
	marker_pub.publish(marker);
	
	// print debug message
	std::string string = "moving to (";
	string += boost::to_string(marker.pose.position.x) +", ";
	string += boost::to_string(marker.pose.position.y) +")";
	debug_print(string);
	
	// send goal to move_base
	actionClient.sendGoal(goal);
	actionlib::SimpleClientGoalState state = actionClient.getState();
	//ROS_INFO("Goal has been recieved! The robot is now %s", state.toString().c_str());

	bool doneDist = false;
	while(actionClient.getState() == actionlib::SimpleClientGoalState::PENDING) {
	}

	// Print the robots state after pending (should be active)
	state = actionClient.getState();
	// print debug message
	string = "Goal is no longer pending! The robot is now ";
	string += boost::to_string(state.toString());
	debug_print(string);
	
	while(actionClient.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
		//check if we reached within a radius of our goal
        	doneDist = checkGoalDist(goal.target_pose, 0.4);
		if(paused == true) {
			// if paused cancel gaol
			debug_print("Cancelling navigation as pause is true");
			actionClient.cancelGoal();
		}
		else if(doneDist == true) {
			// if goal distance is within desired radius cancel goal
			actionClient.cancelGoal();
			// face where you were planning to move
			face(frame, goalX, goalY);
			// move the remaining distance
			geometry_msgs::PoseStamped robot_pose = getRobotPose(frame);
			double distance_remaining = sqrt((goalX-robot_pose.pose.position.x)*(goalX-robot_pose.pose.position.x) + (goalY-robot_pose.pose.position.y)*(goalY-robot_pose.pose.position.y));
			debug_print("Moving the remaining distance: " + boost::to_string(distance_remaining));
			move_straight(distance_remaining);
		}
	}

	// if it leaves the loop and doneDist is false it means the program might have failed to execute properly
	if(doneDist == false) {
		if(actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			// if successful it canceled because it got within the goal tolerance not ours (failed to execute properly).
			debug_print("Navigation succeeded all on its own, you should set the costmap config values properly!");
		} else {
			// if unsuccesful the navigation may not have been able to even get within our tolerance
    	string = "Navigation failed all on its own! The state was ";
    	string += boost::to_string(state.toString());
	    debug_print(string);
		}
	}
	debug_print("Move Successful");
}

bool navigationServer::checkGoalDist(geometry_msgs::PoseStamped goalPose, float tolerance){
	//Is goal within distance? (true if yes, false if no)
	geometry_msgs::PoseStamped robotPose = getRobotPose(goalPose.header.frame_id);

	double distXGoal = goalPose.pose.position.x  - robotPose.pose.position.x;
	double distYGoal = goalPose.pose.position.y  - robotPose.pose.position.y;
	double distFromGoal = sqrt(distXGoal*distXGoal + distYGoal*distYGoal);
	// print debug message
	std::string string = "Distance from goal: ";
	string += boost::to_string(distFromGoal);
	debug_print(string);
	if(distFromGoal <= tolerance) {
		return true;
	}
	return false;
}

