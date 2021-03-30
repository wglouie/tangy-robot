#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include <queue>
#include <algorithm>
#include <iterator>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavigationClient{

public:

    //Client's attributes
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
    //states
    bool done;
    bool running;
    bool ready;
    bool succeeded;

    double current_x, current_y;
    double goal_x, goal_y;
    double dist_x_goal, dist_y_goal;
    double dist_from_goal;
    std::queue<double> locations[2];


    /**
     * Constructor
     * @param name
     */
    NavigationClient(std::string name):
        //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
        ac("move_base", true),
        //Stores the name
        action_name_(name)
    {
        //Get connection to a server
        ROS_INFO("%s Waiting For Server...", action_name_.c_str());
		//Wait for the connection to be valid
        ac.waitForServer();
        ROS_INFO("%s Got a Server...", action_name_.c_str());

        done = false;
        running = false;
        ready = true;
                
    }

    /**
     * Destructor
     */
    ~NavigationClient(){
	//ac.cancelAllGoals();
    }

    
    
    /**
     * Called once when the goal completes
     * @param state
     * @param result
     */
    void doneCb(const actionlib::SimpleClientGoalState& state,
				const move_base_msgs::MoveBaseResult::ConstPtr& result)
    {
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		succeeded = true;
	}

	done = true;

        ROS_INFO("[%s] Finished in state [%s]",action_name_.c_str(), state.toString().c_str());
        //S_INFO("Result: %d", result->plan);
        reset();
    }

    
    /**
     * Called once when the goal becomes active
     */
    void activeCb()
    {

	ROS_INFO("Goal just went active...");
	done = false;
        succeeded = false;

	while(!locations[0].empty())
		locations[0].pop();

	while(!locations[1].empty())
		locations[1].pop();
       

        running = true;
        ready = false;
    }

    
    /**
     * Called every time feedback is received for the goal
     * @param feedback
     */
    void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
    {
        //ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->status);
        //ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->status);
        //ROS_INFO("Got Feedback!"); 

	// Get the current position
	current_x = feedback->base_position.pose.position.x;
	current_y = feedback->base_position.pose.position.y;

	// Store feedback for later sampling
	locations[0].push(current_x);
	locations[1].push(current_y); 

	// Calculate the current distance (x,y) from the goal
	dist_x_goal = goal_x - current_x;
	dist_y_goal = goal_y - current_y; 

	dist_from_goal = sqrt(dist_x_goal*dist_x_goal + dist_y_goal*dist_y_goal);

    	//ROS_INFO("Current position: x = [%lf], y = [%lf]\n"
	//	"Goal position: x = [%lf], y = [%lf]\n"
	//	"Distance from goal: [%lf]\n", feedback->base_position.pose.position.x, 
	//	feedback->base_position.pose.position.y, goal_x, goal_y, dist_from_goal);
		
    	if(dist_from_goal <= 0.15)
	{
		done = true;
		succeeded = true;
		ac.cancelGoal();
		//ROS_INFO("Distance from goal: [%lf]\n",dist_from_goal);
		ROS_INFO("I'm cancelling the goal.");
	}

	if(locations[0].size() > 100 && locations[1].size() > 100)
	{
		if(current_x == locations[0].front() && current_y == locations[1].front())
			ac.cancelGoal();
		
		locations[0].pop();
		locations[1].pop();
	}   
    }


    /**
     * Send a goal to the server
     * @param newGoal
     */
    void sendGoal(const move_base_msgs::MoveBaseGoal newGoal)
    {
	done = false;
	succeeded = false;
        goal_x = newGoal.target_pose.pose.position.x;
	goal_y = newGoal.target_pose.pose.position.y;

        //Once again, have to used boost::bind because you are inside a class
        ac.sendGoal(newGoal, boost::bind(&NavigationClient::doneCb, this, _1, _2),
                                boost::bind(&NavigationClient::activeCb, this),
                                boost::bind(&NavigationClient::feedbackCb, this, _1));
    }

    /**
     * Called for reset the action client (it cannot be running)
     */
    void reset()
    {
        //done = false;
        running = false;
        ready = true;
    }


    bool waitForResult(){
	ROS_INFO("[%s] Waiting for result...", action_name_.c_str());
	bool result = ac.waitForResult();
	return result;
    }



private:
    string action_name_;
	
};
