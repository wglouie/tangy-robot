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

#define PI 3.14159265


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

    //linear x,y
    double current_x, current_y;
    double goal_x, goal_y;
    double dist_x_goal, dist_y_goal;
    double dist_from_goal;

    //angular z,w
    double current_orientation_z, current_orientation_w;
    double goal_orientation_z, goal_orientation_w;
    double current_angle;
    double goal_angle;
    double orientation_distance;

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

	// Get the current linear position
	current_x = feedback->base_position.pose.position.x;
	current_y = feedback->base_position.pose.position.y;
	// Get the current angular position
	current_orientation_z = feedback->base_position.pose.orientation.z;
	current_orientation_w = feedback->base_position.pose.orientation.w;

	// Store feedback for later sampling
	locations[0].push(current_x);
	locations[1].push(current_y); 

	// Calculate the current distance (x,y) from the goal
	dist_x_goal = goal_x - current_x;
	dist_y_goal = goal_y - current_y; 

	dist_from_goal = sqrt(dist_x_goal*dist_x_goal + dist_y_goal*dist_y_goal);

	// Calculate the current distance (orientation) from the goal
	getAngles();

	orientation_distance = goal_angle - current_angle; 

    	//ROS_INFO("Current position: x = [%lf], y = [%lf]\n"
	//     "Goal position: x = [%lf], y = [%lf]\n"
	//     "Distance from goal: [%lf]\n"
	//     "Orientation: z = [%lf], w = [%lf]\n"
	//     "Orientation: current = [%.2lf] degrees, goal = [%.2lf] degrees\n"
	//     "Orientation distance = [%.2lf] degrees\n", current_x, current_y, goal_x, goal_y, dist_from_goal, current_orientation_z, current_orientation_w, current_angle, goal_angle, orientation_distance);
		
    	if(dist_from_goal <= 0.15)
	//if(dist_from_goal <= 0.15 && orientation_distance <= 15.0 && orientation_distance >= -15.0)
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

    	goal_orientation_z = newGoal.target_pose.pose.orientation.z;
    	goal_orientation_w = newGoal.target_pose.pose.orientation.w;

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


    void getAngles()
    {
	// Get goal angle
	if(goal_orientation_z >= 0)
	goal_angle = 2*acos(goal_orientation_w) * 180.0/PI;

	if(goal_orientation_z < 0 && goal_orientation_w >= 0)
	goal_angle = 360.0 + (2*asin(goal_orientation_z) * 180.0/PI);

	if(goal_orientation_z < 0 && goal_orientation_w < 0)
	goal_angle = 360.0 - (2*acos(goal_orientation_w) * 180.0/PI);

	// Get current angle
	if(current_orientation_z >= 0)
	current_angle = 2*acos(current_orientation_w) * 180.0/PI;

	if(current_orientation_z < 0 && current_orientation_w >= 0)
	current_angle = 360.0 + (2*asin(current_orientation_z) * 180.0/PI);

	if(current_orientation_z < 0 && current_orientation_w < 0)
	current_angle = 360.0 - (2*acos(current_orientation_w) * 180.0/PI);
    }



private:
    string action_name_;
	
};
