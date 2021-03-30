#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <drrobot_h20_arm_player/ikPoseAction.h>
#include <moveit/move_group_interface/move_group.h>
//Position message
#include <geometry_msgs/Pose.h>

#ifndef IKCLIENT_HPP_
#define IKCLIENT_HPP_

using namespace std;

class ikClient
{
public:
	ikClient(std::string name);
		
	void doneCb(const actionlib::SimpleClientGoalState& state, const drrobot_h20_arm_player::ikPoseResultConstPtr& result);
	
	void activeCb();
	
	void feedbackCb(const drrobot_h20_arm_player::ikPoseFeedbackConstPtr& feedback);
	
	void send(geometry_msgs::Pose pose,std::string group_name);
	
	void send(float x, float y, float z,std::string group_name);
	
private:
	actionlib::SimpleActionClient<drrobot_h20_arm_player::ikPoseAction> ac;
	std::string action_name;
};

#endif
