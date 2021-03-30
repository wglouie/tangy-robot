#include <tangy_move/tangy_move.h>

void navigationClient::move_straight(double distance) {
	navServerCmd.request.command = "move_straight";
	navServerCmd.request.distance = distance;
	if(!navServerCmdPub.call(navServerCmd)) {
		ROS_ERROR("Failed to call service nacvServerCmd");
	}
}

void navigationClient::move_straight_manual (double distance) {
	navServerCmd.request.command = "move_straight_manual";
	navServerCmd.request.distance = distance;
	if(!navServerCmdPub.call(navServerCmd)) {
		ROS_ERROR("Failed to call service nacvServerCmd");
	}
}
