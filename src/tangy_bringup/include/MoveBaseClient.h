#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>


using namespace std;

class MoveBaseClient{

public:

    //Client's attributes
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
    //states
    bool done;
    bool running;
    bool ready;
    float goal_x;
    float goal_y;

    int sessionStatus;


    /**
     * Constructor
     * @param name
     */
    MoveBaseClient(std::string name):
        //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
        ac("move_base", true),
        //Stores the name
        action_name_(name)
    {
      	//Get connection to a server
      	ROS_INFO("[%s] Waiting For Server...", action_name_.c_str());
      	//Wait for the connection to be valid
      	ac.waitForServer();
      	ROS_INFO("[%s] Got a Server...", action_name_.c_str());
      	sessionStatus = -1;
    
        done = false;
        running = false;
        ready = true;
    }

    MoveBaseClient(std::string name, float wait_time):
        //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
        ac("move_base", true),
        //Stores the name
        action_name_(name)
    {
        //Get connection to a server
        ROS_INFO("[%s] Waiting For Server...", action_name_.c_str());
        //Wait for the connection to be valid
        if(ac.waitForServer(ros::Duration(wait_time))){
            ROS_INFO("[%s] Got a Server...", action_name_.c_str());
        } else {
            ROS_INFO("[%s] Failed to get a Server ...", action_name_.c_str());
        }
        sessionStatus = -1;

        done = false;
        running = false;
        ready = true;
    }

    /**
     * Destructor
     */
    ~MoveBaseClient(){
    //ac.cancelAllGoals();
    }

    
    
    /**
     * Called once when the goal completes
     * @param state
     * @param result
     */
    void doneCb(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResult::ConstPtr& result)
    {
        ROS_INFO("Hooray, I have reached the goal");
        if(state== actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Using my own tolerance.");
            done = true;
        }
        else
            ROS_INFO("Using your tolerance");

        reset();
    }

    
    /**
     * Called once when the goal becomes active
     */
    void activeCb()
    {
        ROS_INFO("[%s] Goal just went active...", action_name_.c_str());
      	sessionStatus = 1;

        done = false;
        running = true;
        ready = false;
    }

    
    /**
     * Called every time feedback is received for the goal
     * @param feedback
     */
    void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
    {
        // Get the current position
        float current_x, current_y,dist_x_goal,dist_y_goal,dist_from_goal;
        current_x = feedback->base_position.pose.position.x;
        current_y = feedback->base_position.pose.position.y;

        // Calculate the current distance (x,y) from the goal
        dist_x_goal = goal_x - current_x;
        dist_y_goal = goal_y - current_y;
        dist_from_goal = sqrt(dist_x_goal*dist_x_goal + dist_y_goal*dist_y_goal);
        if(dist_from_goal <= 0.05)
        {
            ac.cancelGoal();
            ac.waitForResult();
            reset();
        }
    }


    /**
     * Send a goal to the server
     * @param newGoal
     */
     
    void sendGoal(const move_base_msgs::MoveBaseGoal goal)
    {
      
        goal_x = goal.target_pose.pose.position.x;
        goal_y = goal.target_pose.pose.position.y;
        ROS_ERROR("Attempting to move to X:%f, Y:%f", goal_x, goal_y);
        ROS_INFO("[%s] Looking for move_base server..." , action_name_.c_str());
        ac.waitForServer();

        ROS_INFO("Found Server");
        ac.sendGoal(goal,boost::bind(&MoveBaseClient::doneCb, this, _1, _2),
                       boost::bind(&MoveBaseClient::activeCb, this),
                       boost::bind(&MoveBaseClient::feedbackCb, this, _1));

    }
    void sendGoalAndWait(const move_base_msgs::MoveBaseGoal goal)
    {
      
        goal_x = goal.target_pose.pose.position.x;
        goal_y = goal.target_pose.pose.position.y;
        ROS_ERROR("Attempting to move to X:%f, Y:%f", goal_x, goal_y);
        ROS_INFO("[%s] Looking for move_base server..." , action_name_.c_str());
        ac.waitForServer();

        ROS_INFO("Found Server");
        ac.sendGoal(goal,boost::bind(&MoveBaseClient::doneCb, this, _1, _2),
                       boost::bind(&MoveBaseClient::activeCb, this),
                       boost::bind(&MoveBaseClient::feedbackCb, this, _1));

        ac.waitForResult();
    }
    
    /**
     * Called for reset the action client (it cannot be running)
     */
    void reset()
    {
        done = false;
        running = false;
        ready = true;
    }


    void waitForResult(){
      	ROS_INFO("[%s] Waiting for result...", action_name_.c_str());
      	ac.waitForResult();
    }



private:
    string action_name_;
	
};
