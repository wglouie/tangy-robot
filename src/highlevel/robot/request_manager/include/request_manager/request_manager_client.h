#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <request_manager/RequestManagerAction.h>


using namespace std;

class RequestManagerClient{

public:

    //Client's attributes
    actionlib::SimpleActionClient<request_manager::RequestManagerAction> ac;
    //states
    bool done;
    bool running;
    bool ready;
    bool succeeded;


    request_manager::RequestManagerFeedback currentFeedback;
    request_manager::RequestManagerResult currentResult;


    /**
     * Constructor
     * @param name
     */
    RequestManagerClient(std::string name):
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
        succeeded = false;
                
    }

    /**
     * Destructor
     */
    ~RequestManagerClient(){
	//ac.cancelAllGoals();
    }

    
    
    /**
     * Called once when the goal completes
     * @param state
     * @param result
     */
    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const request_manager::RequestManagerResultConstPtr& result)
    {
        ROS_INFO("[%s] Finished in state [%s]", action_name_.c_str(), state.toString().c_str());
        reset();

	currentResult.statusCode = result->statusCode;

	//if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	//	succeeded = true;
	//}
	if (currentResult.statusCode == 1){
		succeeded = true;
	}
	else{
		succeeded = false;
	}

        //S_INFO("Result: %d", result->plan);

        done = true;
    }

    
    /**
     * Called once when the goal becomes active
     */
    void activeCb()
    {
        ROS_INFO("Goal just went active...(Start Robot Request Manager Process)");
        done = false;
        running = true;
        ready = false;
        succeeded = false;
    }

    
    /**
     * Called every time feedback is received for the goal
     * @param feedback
     */
    void feedbackCb(const request_manager::RequestManagerFeedbackConstPtr& feedback)
    {
        //ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->status);
        //ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->status);
        //ROS_INFO("Got Feedback!");    
	
	// Print and assign values to the variables
	//ROS_INFO("Got Feedback of Progress to Goal: status = %d", feedback->status);
	//ROS_INFO("Got Feedback of Progress to Goal: ready = %s", (feedback->ready)?"true":"false");
	//ROS_INFO("Got Feedback of Progress to Goal: action = %s", feedback->action.c_str());

	//ROS_INFO("Got Feedback of Progress to Goal: command = %s", feedback->command.c_str());
	//ROS_INFO("Got Feedback of Progress to Goal: battery level = %.3f", feedback->batterylevel);
	//ROS_INFO("Got Feedback of Progress to Goal: battery capacity = %.3f", feedback->batterycapacity);
	//ROS_INFO("Got Feedback of Progress to Goal: x position = %.3lf",(double) feedback->x_position);
	//ROS_INFO("Got Feedback of Progress to Goal: y position = %.3lf \n",(double) feedback->y_position);
	//ROS_INFO("Location: [%s] \n",positionToLocation().c_str());

	currentFeedback.status = feedback->status;
   	currentFeedback.ready = feedback->ready;
    	currentFeedback.action = feedback->action;			
    	currentFeedback.command = feedback->command;
    	currentFeedback.batterylevel = feedback->batterylevel;
    	currentFeedback.batterycapacity = feedback->batterycapacity;
    	currentFeedback.x_position = feedback->x_position;
	currentFeedback.y_position = feedback->y_position;

	//printf("\n[%s]\n", positionToLocation().c_str());
    }


    /**
     * Send a goal to the server
     * @param newGoal
     */
    void sendGoal(request_manager::RequestManagerGoal newGoal)
    {
        //request_manager::RequestManagerGoal newGoal;
        //newGoal.start = goal;
	done = false;
        succeeded = false;

        //Once again, have to used boost::bind because you are inside a class
        ac.sendGoal(newGoal, boost::bind(&RequestManagerClient::doneCb, this, _1, _2),
                                boost::bind(&RequestManagerClient::activeCb, this),
                                boost::bind(&RequestManagerClient::feedbackCb, this, _1));
    }

    /**
     * Called for reset the action client (it cannot be running)
     */
    void reset()
    {
        done = false;
        running = false;
        ready = true;
        succeeded = false;
    }

	/**
     * this function check in which location of the map the robot is
     */
    std::string positionToLocation(){

		if(currentFeedback.x_position >= -1.400 && currentFeedback.x_position <= 3.250 &&
			currentFeedback.y_position >= -2.200 && currentFeedback.y_position <= 1.900) {
				//printf("\nl0\n");
				return "l0";
		}
		else if(currentFeedback.x_position > 3.250 && currentFeedback.x_position <= 7.100 &&
currentFeedback.y_position >= -2.200 && currentFeedback.y_position <= 2.700 ){
				//printf("\nl1\n");
				return "l1";
		}
		else if(currentFeedback.x_position > 3.250 && currentFeedback.x_position <= 7.100 && 
currentFeedback.y_position > 2.700 && currentFeedback.y_position <= 5.400){
				//printf("\nl2\n");
				return "l2";
		}
		else if(currentFeedback.x_position > 3.250 && currentFeedback.x_position <= 7.100 && 
currentFeedback.y_position > 5.400 && currentFeedback.y_position <= 8.200){
				//printf("\nl3\n");
				return "l3";
		}
		else
			printf("\nthe robot is lost... LOL\n");
			
		return " ";

	}



private:
	std::string action_name_;
};
