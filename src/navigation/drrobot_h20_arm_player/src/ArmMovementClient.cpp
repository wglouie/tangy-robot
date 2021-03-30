#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <drrobot_h20_arm_player/armAction.h>
#include <string>

class ArmMovementClient{


public:

  int state;

	ArmMovementClient(std::string name):
		//Set up the client. It's publishing to topic "test_action", and is set to auto-spin
		ac("ArmMovement", true),
		//Stores the name
		action_name(name)
	{
	  state=0;
		//Get connection to a server
		ROS_INFO("%s Waiting For Server...", action_name.c_str());
		//Wait for the connection to be valid
		ac.waitForServer();
		ROS_INFO("%s Got a Server...", action_name.c_str());
	}
	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,
			const drrobot_h20_arm_player::armResultConstPtr& result)
	{
		ROS_INFO("Feedback worked! Finished moving arms");

		// ros::shutdown();
	}

	// Called once when the goal becomes active
	void activeCb()
	{
		ROS_INFO("Goal just went active...");
	}
	
	// Called every time feedback is received for the goal
	void feedbackCb(const drrobot_h20_arm_player::armFeedbackConstPtr& feedback)
	{
	  ROS_INFO("Client has received Feedback from server!");
	  if(feedback->state==0){
	    ROS_INFO("Arms are not moving!");
	    state=0;
	  }else if(feedback->state==1){
	    ROS_INFO("Arms are moving!");
	    state=1;
	  }
	  
	}

	//Send a goal to the server
	void send(double x, double y, double z)
	{
		drrobot_h20_arm_player::armGoal newGoal;
		
		newGoal.x = x;
		newGoal.y = y;
		newGoal.z = z;
		newGoal.behavior="neutral";
		
    //Once again, have to used boost::bind because you are inside a class
    ac.sendGoal(newGoal, boost::bind(&ArmMovementClient::doneCb, this, _1, _2),
    boost::bind(&ArmMovementClient::activeCb, this),
    boost::bind(&ArmMovementClient::feedbackCb, this, _1));
	}
	void send(std::string behavior)
	{
		drrobot_h20_arm_player::armGoal newGoal;
		
		newGoal.behavior=behavior;
		newGoal.x=100;
		newGoal.y=100;
		newGoal.z=100;
		
    //Once again, have to used boost::bind because you are inside a class
    ac.sendGoal(newGoal, boost::bind(&ArmMovementClient::doneCb, this, _1, _2),
    boost::bind(&ArmMovementClient::activeCb, this),
    boost::bind(&ArmMovementClient::feedbackCb, this, _1));
	}
/*	void move()
	{
	  std::string msg;
	  msg="move";
    //Once again, have to used boost::bind because you are inside a class
    ac.sendGoal(msg, boost::bind(&ArmMovementClient::doneCb, this, _1, _2),
    boost::bind(&ArmMovementClient::activeCb, this),
    boost::bind(&ArmMovementClient::feedbackCb, this, _1));
	}
	*/
private:
	actionlib::SimpleActionClient<drrobot_h20_arm_player::armAction> ac;
	std::string action_name;
};
			
int main (int argc, char **argv)
{
	ros::init(argc, argv, "test_ArmMovementClient");

  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
	ArmMovementClient client(ros::this_node::getName());
	
  /* Sending a command to the server:
  Send either a x,y,z coordinate using send(double x, double y, double z)
  Or send a behavior using send(string behavior).
  List of behaviors: "plan point at screen", "plan celebrate", "plan wave goodbye"
  and "execute point at screen", "execute celebrate", "execute wave goodbye", "execute point to"*/
  // if(client.state==0){
  //   client.send("test");
  //   client.state=1;
  // }
  
//	client.send("plan celebrate");
  //client.send("execute celebrate");
//  sleep(30);
// 	client.send("wave goodbye");
// 	sleep(20);
	client.send("plan convo gesture");
	sleep(5);
	client.send("execute convo gesture");
	sleep(10);
	client.send("execute convo gesture");
	sleep(10);
	client.send("execute convo gesture");
	sleep(10);
	client.send("execute convo gesture");
	sleep(10);
	client.send("execute convo gesture");
	sleep(10);
// 	client.send(-0.15,-0.23,0.84);
  ros::spin();
	return 0;
}
