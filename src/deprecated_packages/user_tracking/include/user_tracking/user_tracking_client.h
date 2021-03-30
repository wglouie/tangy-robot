#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <user_tracking/UserTrackingAction.h>


using namespace std;

class UserTrackingClient{

public:

    //Client's attributes
    actionlib::SimpleActionClient<user_tracking::UserTrackingAction> ac;
    //states
    bool done;
    bool running;
    bool ready;

    string action_name_;

    //user_tracking::UserTrackingActionFeedback feedback_;
    int inRange;
    bool userFound;



    /**
     * Constructor
     * @param name
     */
    UserTrackingClient(std::string name):
        //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
        ac("user_tracking", true),
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
    ~UserTrackingClient(){
	//ac.cancelAllGoals();
    }

    
    
    /**
     * Called once when the goal completes
     * @param state
     * @param result
     */
    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const user_tracking::UserTrackingResultConstPtr& result)
    {
        ROS_INFO("[%s] Finished in state [%s]",action_name_.c_str(), state.toString().c_str());
        //S_INFO("Result: %d", result->plan);
        reset();
        done = true;
    }

    
    /**
     * Called once when the goal becomes active
     */
    void activeCb()
    {
        //ROS_INFO("Goal just went active...[%s])",action_name_.c_str());
        done = false;
        running = true;
        ready = false;
        inRange = 0;
        userFound = false;
    }

    
    /**
     * Called every time feedback is received for the goal
     * @param feedback
     */
    void feedbackCb(const user_tracking::UserTrackingFeedbackConstPtr& feedback)
    {
        //ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->status);
        //ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->status);
        //ROS_INFO("Got Feedback from User tracking!");
        inRange = feedback->inRange;
        ROS_INFO("[%s] inRange: %d\n\n",action_name_.c_str(), inRange);
        //userFound = feedback->userFound;
    }


    /**
     * Send a goal to the server
     * @param newGoal
     */
    //void sendGoal(user_tracking::UserTrackingGoal newGoal)
    void sendGoal(string goal)
    {
        user_tracking::UserTrackingGoal newGoal;
        newGoal.userName = goal;
        done = false;
        inRange = 0;
        userFound = false;
        ROS_INFO("User: [%s]",newGoal.userName.c_str());

        //Once again, have to used boost::bind because you are inside a class
        ac.sendGoal(newGoal, boost::bind(&UserTrackingClient::doneCb, this, _1, _2),
                                boost::bind(&UserTrackingClient::activeCb, this),
                                boost::bind(&UserTrackingClient::feedbackCb, this, _1));
    }

    /**
      * Send a goal to the server
      */
	void send()
	{
		user_tracking::UserTrackingGoal newGoal;
		newGoal.userName = "Matthew";
		done = false;
		inRange = 0;
		userFound = false;
		//ROS_INFO("Goal: %i\n", goal);
		//Once again, have to used boost::bind because you are inside a class
		ac.sendGoal(newGoal, boost::bind(&UserTrackingClient::doneCb, this, _1, _2),
					boost::bind(&UserTrackingClient::activeCb, this),
					boost::bind(&UserTrackingClient::feedbackCb, this, _1));
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

    /**
     * This function cancel all the goals
     */
    void stopTracking(){
        ac.cancelGoal();
        reset();
        inRange = 0;
        userFound = false;
    }

private:
    //string action_name_;
	
};
