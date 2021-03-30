#include <tangy_move/tangy_move.h>

void navigationClient::pause() {
	navServerCmd.request.command = "pause";
	if(!navServerCmdPub.call(navServerCmd)) {
		ROS_ERROR("Failed to call service nacvServerCmd");
	}
}

void navigationClient::unpause() {
	navServerCmd.request.command = "unpause";
	if(!navServerCmdPub.call(navServerCmd)) {
		ROS_ERROR("Failed to call service nacvServerCmd");
	}
}
