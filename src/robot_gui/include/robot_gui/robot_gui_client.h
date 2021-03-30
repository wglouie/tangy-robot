#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot_gui/Robot_guiAction.h>
#include <string>


using namespace std;

class RobotGuiClient{

public:

    //Client's attributes
    actionlib::SimpleActionClient<robot_gui::Robot_guiAction> ac;
    //states
    bool done;
    bool running;
    bool ready;

    int sessionStatus;


    /**
     * Constructor
     * @param name
     */
    RobotGuiClient(std::string name):
        //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
        ac("robot_gui_server", true),
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

    RobotGuiClient(std::string name,float wait_time):
        //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
        ac("robot_gui_server", true),
        //Stores the name
        action_name_(name)
    {
        //Get connection to a server
        ROS_INFO("[%s] Waiting For Server...", action_name_.c_str());
        //Wait for the connection to be valid
        ac.waitForServer(ros::Duration(wait_time));
        ROS_INFO("[%s] Got a Server...", action_name_.c_str());
        sessionStatus = -1;

        done = false;
        running = false;
        ready = true;
    }

    /**
     * Destructor
     */
    ~RobotGuiClient(){
    //ac.cancelAllGoals();
    }

    
    
    /**
     * Called once when the goal completes
     * @param state
     * @param result
     */
    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const robot_gui::Robot_guiResultConstPtr& result)
    {


      	if(result->end == 1){

      		sessionStatus = 3;
      	}
      	else if(result->end == 2){

      		sessionStatus = 4;
      	}
      	else if(result->end == 3){

      	}

        reset();
    }

    
    /**
     * Called once when the goal becomes active
     */
    void activeCb()
    {
      	sessionStatus = 1;

        done = false;
        running = true;
        ready = false;
    }

    
    /**
     * Called every time feedback is received for the goal
     * @param feedback
     */
    void feedbackCb(const robot_gui::Robot_guiFeedbackConstPtr& feedback)
    {
      	
      	if(feedback->feedback == 0){
      	}
      	else if(feedback->feedback == 1){
      		sessionStatus = 1;
      	}
      	else if(feedback->feedback == 2){
      		sessionStatus = 2;
      	}
      	else if(feedback->feedback == 3){
      	}
    }


    /**
     * Send a goal to the server
     * @param newGoal
     */
    void sendGoal(robot_gui::Robot_guiGoal newGoal)
    {
        ac.sendGoal(newGoal, boost::bind(&RobotGuiClient::doneCb, this, _1, _2),
                                boost::bind(&RobotGuiClient::activeCb, this),
                                boost::bind(&RobotGuiClient::feedbackCb, this, _1));
    }
    void sendGoalAndWait(robot_gui::Robot_guiGoal newGoal)
    {
        ac.sendGoal(newGoal, boost::bind(&RobotGuiClient::doneCb, this, _1, _2),
                                boost::bind(&RobotGuiClient::activeCb, this),
                                boost::bind(&RobotGuiClient::feedbackCb, this, _1));
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
