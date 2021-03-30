
/**
 * Author: Tiago Vaquero
 * Date: 2013
 * 
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
//#include <sound_play/sound_play.h>

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <cstring>
#include <fstream>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <unistd.h>

#include <sys/types.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
//#include <time.h>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>

#include <planning/mylib.h>
#include <planning/plan.h>
#include <planning/planner.h>

//Action specification file 
#include <planning/PlanningAction.h>

//Messages
//#include <planning/Plan.h>
#include <server_msgs/Plan.h>
#include <server_msgs/PlanRequest.h>
//#include <server_msgs/CurrentStateRequest.h> //Used for requesting current state through topics
#include <server_msgs/InitialStateRequest.h>

//Clients
//#include "../../execution/include/execution/execution_client.h"



using namespace std;


enum PlanningStates { IDLE , GETTINGINPUT , REQUESTING , PLANNING , FINISHED };


class PlanningAgent
{
    
protected:
    ros::NodeHandle nh_;
    
    // create the Planner server
    actionlib::SimpleActionServer<planning::PlanningAction> as_;
    std::string action_name_;
    
public:
    
    // attributes
    string filepath;    
    bool ready;
    bool finished;
    bool success;
    bool newrequest;
    //state variables
    int currentstateid;
    int previousstateid;

    string requester;

    
    //planner's variables
    Planner planner;

          
    // create messages that are used to publish feedback/result
    planning::PlanningFeedback feedback_;
    planning::PlanningResult result_;
    //planning::Plan plan_;
    server_msgs::Plan plan_;
    //server_msgs::CurrentStateRequest current_state_request_;
    
    //Clients
	//planning
	//ExecutionClient executionclient;
    
    //Subscribers and Publishers
    ros::Publisher plan_pub;
    ros::Subscriber plan_request_sub;
    //ros::Publisher current_state_request_pub;   //use this subscriber if you want to request the current state through a topic
    
    //Service clients
    ros::ServiceClient initial_state_request_client; //use this subscriber if you want to request the current state through a service


    /**
     * Constructor
     **/
    PlanningAgent(string name, Planner selectedplanner):
            as_(nh_ , name , boost::bind(&PlanningAgent::executeCB , this , _1) , false) ,
            action_name_(name)//,
            //executionclient("execution")
	{
        printf("Initializing planning agent.\n");
        ready = true;
        currentstateid = IDLE;//0;
        previousstateid = -1;
        
        finished = true;
        success = false;
        
        newrequest = false;

        //sets the selected planner
        planner = selectedplanner;
       
        //starts the server
        as_.start();


        //plan_pub = nh_.advertise<planning::Plan>("plan", 5);
        plan_pub = nh_.advertise<server_msgs::Plan>("plan", 5);
        
        //subscribe to the plan request topic
        plan_request_sub = nh_.subscribe<server_msgs::PlanRequest>("plan_request", 5, boost::bind(&PlanningAgent::planRequestCallback, this, _1));        
        //current_state_request_pub = nh_.advertise<server_msgs::CurrentStateRequest>("current_state_request", 5);
        
        //service client
        initial_state_request_client = nh_.serviceClient<server_msgs::InitialStateRequest>("initial_state_request");

    }
    
    /**
     * Destructor
     */
    ~PlanningAgent(){}
    
    /**
     * This is the main callback. It is called when the server receives
     * a goal, and it is responsible for doing everything related to the 
     * planning application.
     * @param goal
     */
    void executeCB(const planning::PlanningGoal::ConstPtr &goal)
    {
        printf("\nGOAL RECEIVED");
                 

        // Rate = 10 Hz
        ros::Rate r(10);
        
        newrequest = true;
        finished = false;
        success = false;

        requester = goal->sender;
        
        feedback_.status = GETTINGINPUT; //REQUESTING;
        //publish the feedback
        as_.publishFeedback(feedback_);
        
        //currentstateid = GETTINGINPUT; //REQUESTING; 	
         
        // While the the planning process is not finished       
        while (!finished){

            // If the bingo game has been preempted
            if (as_.isPreemptRequested() || !ros::ok())
            {                
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                finished = true;
                currentstateid = IDLE;                                
                planner.stop(); // Kill planner process
                printf("\nStatus: (%i) Idle. Waiting for plan request \n",currentstateid);
                
                break;
            }            
            //doUpdate(); 
            
            //publish the feedback
            as_.publishFeedback(feedback_);

            // Guarantees the 10Hz rate
            r.sleep();
        }
        
        // If the planning is over and everything went as expected
        if(success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());

            // set the action state to succeeded
            as_.setSucceeded(result_);

            //publish the plan
            //plan_pub.publish(plan_);

        }

    }
        
    
    /**
	  * This function is called when a plan request is sent/published.
	  * TODO: Manage several planning requests.
	  */
	//void planCallback(const planning::Plan::ConstPtr& planmsg)
	void planRequestCallback(const server_msgs::PlanRequest::ConstPtr& planmsg)
	{
		printf("\n(!!!) Plan Request Received: code [%i]\n", planmsg->code);

		planner.stop(); // Kill planner process
		requester = planmsg->requester;
		//finished = false;
		//success = false;           
		newrequest = true;
		
	}
    
    


    /**
     * This function updated the status of the planning agent
     **/
    void doUpdate(){
        
       
        switch (currentstateid) {

        
			case IDLE: //Status: Idle
			{    //Print state
				if (previousstateid != IDLE){
				   printf("\nStatus: (%i) Idle. Waiting for plan request \n",currentstateid);
				}
				feedback_.status = IDLE;
				
				//set default viariables
				//finished = true;
				//success = false;
				
				if(newrequest){				
				   currentstateid = GETTINGINPUT;//REQUESTING;
				}
				
				previousstateid = IDLE;
				break;
			}
			
			case GETTINGINPUT: //Status: Checking the input for the planner 
			{    //Print state
				if (previousstateid != GETTINGINPUT){
				   printf("\nStatus: (%i) Getting Input for planner. \n",currentstateid);
				}
				feedback_.status = GETTINGINPUT;								
				
				//1) Get current state and the goal (they are saved in the problem.pddl file)
				//TODO: Should this module get the information and generated the file? 
				
				
				//USING SERVICE
				
				//send the request to the service
				server_msgs::InitialStateRequest srv;
				srv.request.code = 1;
				srv.request.requester = action_name_;
				//chek if the server is on
				bool service_timeout = initial_state_request_client.waitForExistence (ros::Duration(5.0));				
				if (service_timeout)
				{ 					
					if (initial_state_request_client.exists() && initial_state_request_client.call(srv)) // calls the service
					{
						
						//Save content to file
						string outputproblemfile = planner.path+ "/problem.pddl";
						//get the content
						string content = srv.response.state;
						ROS_INFO("Problem Instance: \n%s", content.c_str());
						
						//Save pddl problem file
						//if the file is already created by another program or manually then comment the code below
						ofstream myfile;
						myfile.open(outputproblemfile.c_str());
						myfile << content;
						myfile.close();			
						
						if (boost::filesystem::exists( outputproblemfile ) )
						{
							std::cout << "Problem Instance file found." << std::endl;
						}
						currentstateid = REQUESTING;
					}
					else
					{
						ROS_ERROR("Failed to call service service initial_state_request");
					}
					
				}
				else
				{
					ROS_ERROR("Service initial_state_request does not exists!");
				}
				
		
				
				//USING MESSAGES/TOPICS
				/*
				//check if the problem file already exists (someone already put it there to solve it
				string probleminstancefile = planner.path + "/problem.pddl";
				if (boost::filesystem::exists( probleminstancefile ) )
				{
					std::cout << "Problem Instance file found." << std::endl;
					currentstateid = REQUESTING;
					
				}else{
					std::cout << "Can't find problem file!" << std::endl;
					current_state_request_.code = 1;
					current_state_request_.requester = action_name_;
					//requesting anyone to add the problem file in the planner's folder.
					current_state_request_pub.publish(current_state_request_);
					
					int counter = 0;
					std::cout << "Waiting problem file.\n" << std::endl;
					while(!boost::filesystem::exists(probleminstancefile) || counter < 500){
						//printf(".");
						//waiting
						counter++;
					}
	
				}
				*/
				
					
				
				//USING CLIENT
								
				
				/*
				 * //createProblemInstance();
				printf("Getting the Current state with from the Execution module")
				ROS_INFO("Waiting for execution server to start.");
				// wait for the action server to start
				executionclient.ac.waitForServer(); //will wait for infinite time
				
				ROS_INFO("Execution server started, sending goal.");
				// send a goal to the action
				execution::ExecutionGoal execution_goal;
				execution_goal.newrequest = 1;
				//execution_goal.request = "replan";
				execution_goal.request = "current_state";
				a.sendGoal(execution_goal);				
				
				execution::ExecutionResult result;
				
				//Wait for the action to return
				bool finished_before_timeout = a.ac.waitForResult(ros::Duration(120.0));
				if (finished_before_timeout)
				{
				actionlib::SimpleClientGoalState state = a.ac.getState();
				ROS_INFO("Getting Result\n");
				result = *a.ac.getResult();
				
				ROS_INFO("Action finished: %s",state.toString().c_str());
				//ROS_INFO("Result (Plan): \n%s",result.plan.c_str());  
		
				 
				}
				else{
				ROS_INFO("Action did not finish before the time out.");    
				//Cancelling the goals
				a.ac.cancelAllGoals();
				ROS_INFO("Goal Canceled");
				
				}
				*/				
				
				previousstateid = GETTINGINPUT;
				break;
			}
				
        
            //Status: Requesting Planning
            case REQUESTING:
            {
                //Print state
                if (previousstateid != REQUESTING){
                    printf("\nStatus: (%i) Requesting Planning \n",currentstateid);
                }
                feedback_.status = REQUESTING;

                //2) Call the planner
                planner.run();

                currentstateid = PLANNING;
                previousstateid = REQUESTING;
                break;
            }

            //Status: Planning
            case PLANNING:
            {
                //Print state
                if (previousstateid != PLANNING){
                    printf("\nStatus: (%i) Planning \n",currentstateid);
                }
                feedback_.status = PLANNING;


                //When the planner has responded to the planning request we read
                // the output
                if (planner.plannerResponded){
                    printf(" >> Planner has responded! \n");

                    //Get the Plan from the planner's output
                    Plan p(planner.output, planner);

                    if (p.solutionExists){
                        p.print();
                        //printf("Plan:\n%s",p.toString().c_str());
                    }
                    else{
                        printf(" (!) Problem Unsolvable! \n");
                    }

                    //set the output plan as the result of a client request
                    result_.plan = p.toString();
                    result_.solutionExists = p.solutionExists;
                    result_.isUnsolvable = p.isUnsolvable;
                    result_.requester = requester;

                    //set the plan to be published for execution
                    plan_.plan = p.toString();
                    plan_.solutionExists = p.solutionExists;
                    plan_.isUnsolvable = p.isUnsolvable;
                    plan_.requester = requester;                    

                    currentstateid = FINISHED;
                }

                previousstateid = PLANNING;
                break;
            }

            //Status: Planning Finished
            case FINISHED:
            {
                //Print state
                if (previousstateid != FINISHED){
                    printf("\nStatus: (%i) Planning Finished \n",currentstateid);
                }
                feedback_.status = FINISHED;

                newrequest = false;
                finished = true;
                success = true;
                
                //publish the plan
                plan_pub.publish(plan_);
                

                currentstateid = IDLE;

                previousstateid = FINISHED;
                break;
            }
        }

    }
    
   

};


int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh;

    printf("Starting the planning module!!!\n");
    //sleep(2);

     //Get parameters
    std::string path;
    nh.getParam("path", path);

    //Get Planner's parameters
    string plannersname;
    nh.getParam("planner", plannersname);
    if (plannersname == ""){
        plannersname = "metric-ff";}
    std::string plannerpath;
    nh.getParam("plannerpath", plannerpath);
    std::string optionsPrior;
    nh.getParam("optionsprior", optionsPrior);
    std::string solutionFoundMarker;
    nh.getParam("solutionfoundmarker", solutionFoundMarker);
    std::string problemUnsolvableMarker;
    nh.getParam("problemunsolvablemarker", problemUnsolvableMarker);


    //Creating the selected planner
    Planner selectedplanner(plannersname);
    selectedplanner.path = plannerpath;
    selectedplanner.solutionFoundMarker = solutionFoundMarker;
    selectedplanner.problemUnsolvableMarker = problemUnsolvableMarker;
    selectedplanner.optionsPrior = optionsPrior;


    //Initialize agent
    PlanningAgent agent(ros::this_node::getName(), selectedplanner);
    //printf("Agent ready? %s",(agent.ready)?"true":"false");
    agent.filepath = path;

    ROS_INFO("Planning module is running");
    //ros::spin();

 
    //Rate = 10 Hz
    ros::Rate r(10);
    while (nh.ok()) {
        agent.doUpdate();
        ros::spinOnce();
        r.sleep();
    }

    
    return 0;
}


