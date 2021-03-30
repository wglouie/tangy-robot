#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <script_player/ScriptPlayAction.h>
#include <string>


class ScriptClient{
public:
	ScriptClient(std::string name):
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
	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,
			const script_player::ScriptPlayResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ROS_INFO("Result: %d", result->success);
		ros::shutdown();
	}

	// Called once when the goal becomes active
	void activeCb()
	{
		ROS_INFO("Goal just went active...");
	}
	
	// Called every time feedback is received for the goal
	void feedbackCb(const script_player::ScriptPlayFeedbackConstPtr& feedback)
	{
		//ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->currentSampleNumber);
	}

	//Send a goal to the server
	void send(std::string goal)
	{
		script_player::ScriptPlayGoal newGoal;
		newGoal.script_folder = goal;
		
		//Once again, have to used boost::bind because you are inside a class
		ac.sendGoal(newGoal, boost::bind(&ScriptClient::doneCb, this, _1, _2),
					boost::bind(&ScriptClient::activeCb, this),
					boost::bind(&ScriptClient::feedbackCb, this, _1));
	}
private:
	actionlib::SimpleActionClient<script_player::ScriptPlayAction> ac;
	std::string action_name;
};
			
int main (int argc, char **argv)
{


	ros::init(argc, argv, "script_player_client");
	std::string test_path;
	ros::NodeHandle nh_;


	//Usage check to make sure the client is being used properly

	//Initialize the client
	ScriptClient client("script_player_client");


	nh_.getParam("/script_folder", test_path);	//Grab album file from parameter server


	client.send(test_path);

	//ROS_INFO("Sent Goal %d To Server...", sampleGoal);

	ros::spin();


	return 0;
}
