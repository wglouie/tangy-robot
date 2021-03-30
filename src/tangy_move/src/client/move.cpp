#include <tangy_move/tangy_move.h>

void navigationClient::move(std::string frame, double goalX, double goalY){ //move robot to absolute/preset goals
	navServerCmd.request.command = "move";
	navServerCmd.request.frame = frame;
	navServerCmd.request.goalX = goalX;
	navServerCmd.request.goalY = goalY;
	if(!navServerCmdPub.call(navServerCmd)) {
		ROS_ERROR("Failed to call service navServerCmd");
	} 
}
