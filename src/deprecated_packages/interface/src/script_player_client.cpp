#include "script_player_client.hpp"

ScriptClient::ScriptClient(std::string name):
	//Set up the client. It's publishing to topic "test_action", and is set to auto-spin
	ac("script_player", true),
	//Stores the name
	action_name(name)
{
	//Get connection to a server
	ROS_INFO("%s Waiting For Server...", action_name.c_str());
	//Wait for the connection to be valid
	ac.waitForServer();
	ROS_INFO("%s Got a Server...", action_name.c_str());
}
ScriptClient::~ScriptClient(){}
	
// Called once when the goal completes
void ScriptClient::doneCb(const actionlib::SimpleClientGoalState& state,
		const script_player::ScriptPlayResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	ROS_INFO("Result: %d", result->success);
	//ros::shutdown();
}

// Called once when the goal becomes active
void ScriptClient::activeCb()
{
	ROS_INFO("Goal just went active...");
}
	
// Called every time feedback is received for the goal
void ScriptClient::feedbackCb(const script_player::ScriptPlayFeedbackConstPtr& feedback)
{
	//ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->currentSampleNumber);
}

//Send a goal to the server
void ScriptClient::send(std::string goal)
{
	script_player::ScriptPlayGoal newGoal;
	newGoal.script_folder = goal;
		
	//Once again, have to used boost::bind because you are inside a class
	ac.sendGoal(newGoal, boost::bind(&ScriptClient::doneCb, this, _1, _2),
					boost::bind(&ScriptClient::activeCb, this),
					boost::bind(&ScriptClient::feedbackCb, this, _1));
}
