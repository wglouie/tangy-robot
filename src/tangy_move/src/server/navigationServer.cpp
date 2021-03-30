#include <ros/ros.h>
#include <tangy_move/navigationServer.h>

navigationServer *navServer;

bool navServerCmdCb(tangy_move::navServerCmd::Request  &req,
         tangy_move::navServerCmd::Response &res)
{
	if(req.command == "move") {
		navServer->move(req.frame, req.goalX, req.goalY);
	} else if(req.command == "move_straight") {
		navServer->move_straight(req.distance);
	} else if(req.command == "move_straight_manual") {
		navServer->move_straight_manual(req.distance);
	} else if(req.command == "pause") {
		navServer->pause();
	} else if(req.command == "unpause") {
		navServer->unpause();
	} else if(req.command == "rotate_no_wait") {
        navServer->rotate(req.frame, req.goalTheta);
	} else if(req.command == "rotate") {
		navServer->rotate(req.frame, req.goalTheta);
	} else if(req.command == "rotate_manual") {
		navServer->rotate_manual(req.angle);
	} else if(req.command == "face") {
		navServer->face(req.frame, req.goalX, req.goalY);
	} else {
	}
	res.done = true;
	return true;
}

void pauseCb(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == "pause") {
         navServer->pause();
    } else if (msg->data == "unpause") {
         navServer->unpause();
    } else {
        ROS_ERROR("INVALID STRING FOR PAUSESUB");
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "navigationServer");
	ros::NodeHandle nh;
	navServer = new navigationServer();

	navServer->initialize(nh);
	ros::ServiceServer service = nh.advertiseService("navServerCmd", navServerCmdCb);
	ros::Subscriber pauseSub = nh.subscribe("pauseSub", 1000, pauseCb);
	ros::spin();
	delete navServer;
	return 0;
}
