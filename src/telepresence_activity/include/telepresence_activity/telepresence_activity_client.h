#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <telepresence_activity/Telepresence_activity_serverAction.h>


using namespace std;

class TelepresenceActivityClient{

public:

    //Client's attributes
    actionlib::SimpleActionClient<telepresence_activity::Telepresence_activity_serverAction> ac;
    //states
    bool done;
    bool running;
    bool ready;
    bool succeeded;

    string action_name_;



    /**
     * Constructor
     * @param name
     */
    TelepresenceActivityClient(std::string name):
        //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
        ac("telepresence_activity_server", true),
        //Stores the name
        action_name_(name)
    {
        //Get connection to a server
        ROS_INFO("[%s] Waiting For Server...", action_name_.c_str());
	//Wait for the connection to be valid
        ac.waitForServer();
        ROS_INFO("[%s] Got a Server...", action_name_.c_str());

        done = false;
        running = false;
        ready = true;
                
    }

    /**
     * Destructor
     */
    ~TelepresenceActivityClient(){
	//ac.cancelAllGoals();
    }

    
    
    /**
     * Called once when the goal completes
     * @param state
     * @param result
     */
    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const telepresence_activity::Telepresence_activity_serverResultConstPtr& result)
    {
	
        ROS_INFO("[%s] Finished in state [%s]", action_name_.c_str(), state.toString().c_str());
        reset();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		succeeded = true;
	}

        //ROS_INFO("Result: %d", result->plan);
	done = true;
    }

    
    /**
     * Called once when the goal becomes active
     */
    void activeCb()
    {
        ROS_INFO("Goal just went active...");
        done = false;
        running = true;
        ready = false;
	succeeded = false;
    }

    
    /**
     * Called every time feedback is received for the goal
     * @param feedback
     */
    void feedbackCb(const telepresence_activity::Telepresence_activity_serverFeedbackConstPtr& feedback)
    {
        //ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->status);
        //ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->status);
        //ROS_INFO("Got Feedback!");    
    }


    /**
     * Send a goal to the server
     * @param newGoal
     */
    void sendGoal(telepresence_activity::Telepresence_activity_serverGoal newGoal)
    {
	done = false;
	succeeded = false;
        //telepresence_activity::RequestManagerGoal newGoal;
        //newGoal.start = goal;

        //Once again, have to used boost::bind because you are inside a class
        ac.sendGoal(newGoal, boost::bind(&TelepresenceActivityClient::doneCb, this, _1, _2),
                                boost::bind(&TelepresenceActivityClient::activeCb, this),
                                boost::bind(&TelepresenceActivityClient::feedbackCb, this, _1));
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


    void waitForResult(){
	ROS_INFO("[%s] Waiting for result...", action_name_.c_str());
	ac.waitForResult();
    }



private:
    //string action_name_;
	
};
