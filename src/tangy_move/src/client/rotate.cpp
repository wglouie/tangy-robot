#include <tangy_move/tangy_move.h>
#include <math.h> 


void navigationClient::rotate_no_wait(std::string frame, double goalTheta) {
	navServerCmd.request.command = "rotate_no_wait";
	navServerCmd.request.frame = frame;
	navServerCmd.request.goalTheta = goalTheta;
	if(!navServerCmdPub.call(navServerCmd)) {
		ROS_ERROR("Failed to call service nacvServerCmd");
	}
}

void navigationClient::rotate(std::string frame, double goalTheta) {
    navServerCmd.request.command = "rotate";
	navServerCmd.request.frame = frame;
	navServerCmd.request.goalTheta = goalTheta;
	if(!navServerCmdPub.call(navServerCmd)) {
		ROS_ERROR("Failed to call service nacvServerCmd");
	}
}

void navigationClient::rotate_manual(double angle) {
	navServerCmd.request.command = "rotate_manual";
	navServerCmd.request.angle = angle;
	if(!navServerCmdPub.call(navServerCmd)) {
		ROS_ERROR("Failed to call service nacvServerCmd");
	}
}

void navigationClient::face(std::string frame, double goalX, double goalY) {
	navServerCmd.request.command = "face";
	navServerCmd.request.goalX = goalX;
	navServerCmd.request.goalY = goalY;
	if(!navServerCmdPub.call(navServerCmd)) {
		ROS_ERROR("Failed to call service nacvServerCmd");
	}
}
