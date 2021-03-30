#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <planning/PlanningAction.h>


using namespace std;

class PlanningClient{

public:

    //Client's attributes
    actionlib::SimpleActionClient<planning::PlanningAction> ac;
    //states
    bool done;
    bool running;
    bool ready;

    /**
     * Constructor
     * @param name
     */
    PlanningClient(std::string name):
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
    ~PlanningClient(){
	//ac.cancelAllGoals();
    }

    
    
    /**
     * Called once when the goal completes
     * @param state
     * @param result
     */
    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const planning::PlanningResultConstPtr& result)
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
        ROS_INFO("Goal just went active...(Start Planning Process)");
        done = false;
        running = true;
        ready = false;
    }

    
    /**
     * Called every time feedback is received for the goal
     * @param feedback
     */
    void feedbackCb(const planning::PlanningFeedbackConstPtr& feedback)
    {
        ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->status);
        //ROS_INFO("Got Feedback!");    
    }


    /**
     * Send a goal to the server
     * @param newGoal
     */
    void sendGoal(planning::PlanningGoal newGoal)
    {
	done = false;        
	//planning::PlanningGoal newGoal;
        //newGoal.start = goal;

        //Once again, have to used boost::bind because you are inside a class
        ac.sendGoal(newGoal, boost::bind(&PlanningClient::doneCb, this, _1, _2),
                                boost::bind(&PlanningClient::activeCb, this),
                                boost::bind(&PlanningClient::feedbackCb, this, _1));
    }

    /**
     * Called for reset the action client (it cannot be running)
     */
    void reset()
    {
        done = true;
        running = false;
        ready = true;
    }



private:
	std::string action_name_;
};
