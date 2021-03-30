#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <execution/ExecutionAction.h>


using namespace std;

class ExecutionClient{

public:

    //Client's attributes
    actionlib::SimpleActionClient<execution::ExecutionAction> ac;
    //states
    bool done;
    bool running;
    bool ready;

    /**
     * Constructor
     * @param name
     */
    ExecutionClient(std::string name):
        //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
        ac(name, true),
        //Stores the name
        action_name_(name)
    {
	//Get connection to a server
        //ROS_INFO("%s Waiting For Server...", action_name.c_str());
		//Wait for the connection to be valid
        //ac.waitForServer();
        //ROS_INFO("%s Got a Server...", action_name.c_str());

        done = false;
        running = false;
        ready = true;
                
    }

    /**
     * Destructor
     */
    ~ExecutionClient(){
	//ac.cancelAllGoals();
    }

    
    
    /**
     * Called once when the goal completes
     * @param state
     * @param result
     */
    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const execution::ExecutionResultConstPtr& result)
    {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        //S_INFO("Result: %d", result->plan);
        reset();
    }

    
    /**
     * Called once when the goal becomes active
     */
    void activeCb()
    {
        ROS_INFO("Goal just went active...(Start Execution Process)");
        done = false;
        running = true;
        ready = false;
    }

    
    /**
     * Called every time feedback is received for the goal
     * @param feedback
     */
    void feedbackCb(const execution::ExecutionFeedbackConstPtr& feedback)
    {
        ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->status);
        //ROS_INFO("Got Feedback!");    
    }


    /**
     * Send a goal to the server
     * @param newGoal
     */
    void sendGoal(execution::ExecutionGoal newGoal)
    {
        //execution::ExecutionGoal newGoal;
        //newGoal.start = goal;

        //Once again, have to used boost::bind because you are inside a class
        ac.sendGoal(newGoal, boost::bind(&ExecutionClient::doneCb, this, _1, _2),
                                boost::bind(&ExecutionClient::activeCb, this),
                                boost::bind(&ExecutionClient::feedbackCb, this, _1));
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



private:
	std::string action_name_;
};
