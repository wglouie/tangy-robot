
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
#include <boost/bimap.hpp> //maps - key -> value
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

//XML libraries
#include <libxml/parser.h>
#include <libxml/tree.h>


//Action specification file 
#include <execution/mylib.h>
#include <execution/ExecutionAction.h>
#include <execution/execution_plan.h>
#include <execution/action.h>
#include <execution/database.h>
#include <execution/xmltopddl.h>

//Planning client
#include <planning/PlanningAction.h> // Action
#include <planning/planning_client.h> // Client

// Messages
//#include <planning/Plan.h> 
#include <server_msgs/Plan.h>
//#include <server_msgs/PlanRequest.h> // Used for requesting plan through a topic
//#include <server_msgs/CurrentStateRequest.h> //Used for requesting current state through topics
#include <server_msgs/InitialStateRequest.h>

//Robot client - request manager
#include <request_manager/request_manager_client.h>


using namespace std;


enum ExecutionStates { IDLE , REQUESTINGPLAN , WAITINGPLAN , EXECUTING, FINISHED };


typedef boost::bimap< std::string, RequestManagerClient* > robots_bimap;
typedef robots_bimap::value_type map_value;


//TODO:
/**
  * Keep track record of the executed actions. Trace
  *
  */


class ExecutionAgent
{
    
protected:
    ros::NodeHandle nh_;
    
    // create the Execution server
    actionlib::SimpleActionServer<execution::ExecutionAction> as_;
    std::string action_name_;
    
public:
    
    // attributes
    string filepath;
    string plannerspath;   //file location of the planner  
    
    bool ready;
    bool finished;
    bool success;
    bool gotNewPlan;
    //state variables
    int currentstateid;
    int previousstateid;       
    
    ExecutionPlan currentplan;
    string planrequester;
    string currentstaterequester;


    int planindex;
    float timeindex;

    // create messages that are used to publish feedback/result
    execution::ExecutionFeedback feedback_;
    execution::ExecutionResult result_;

    //Clients
    //planning
    PlanningClient planningclient; //In case I want to use a client
    
    //Subscribers and Publishers
    ros::Subscriber plan_sub; // Used to listen/get plan through a topic
    //ros::Publisher plan_request_pub; // Used for requesting plan through a topic
    //ros::Subscriber current_state_request_sub; // Used for receiving current state request through a topic
    //server_msgs::PlanRequest plan_request_; // Used for requesting plan through a topic
    
    //Services
    ros::ServiceServer initial_state_service;
    

    
    //robots
    boost::bimap< std::string, RequestManagerClient* > robots_map; // a map of robots (name -> client)
    
    
    //Database
    Database db;
    
    
    /**
     * Constructor
     **/
    ExecutionAgent(string name):
            as_(nh_ , name , boost::bind(&ExecutionAgent::executeCB , this , _1) , false) ,
            action_name_(name),
            planningclient("planning") //Using client
	{

        printf("Initializing execution agent.\n");
        ready = true;
        currentstateid = IDLE;//0;
        previousstateid = -1;
        
        finished = true;
        success = false;
        
        gotNewPlan = false;

        planindex = 0;
        timeindex = 0.0;

        //starts the server
        as_.start();

        //Publishers and Subscribers
        //subcribe to the plan topic
        //plan_sub = nh_.subscribe<planning::Plan>("plan", 5, boost::bind(&ExecutionAgent::planCallback, this, _1));
        plan_sub = nh_.subscribe<server_msgs::Plan>("plan", 5, boost::bind(&ExecutionAgent::planCallback, this, _1));
        //publish to the plan request topic
        //plan_request_pub = nh_.advertise<server_msgs::PlanRequest>("plan_request", 5);        
        //subcribe to the current_state_request topic to gather information about the robots and save the current state (PDDL) in a file
        //current_state_request_sub = nh_.subscribe<server_msgs::CurrentStateRequest>("current_state_request", 1, boost::bind(&ExecutionAgent::currentStateRequestCallback, this, _1));
        
        //Services
        initial_state_service = nh_.advertiseService<server_msgs::InitialStateRequest::Request,server_msgs::InitialStateRequest::Response>("initial_state_request", boost::bind(&ExecutionAgent::getInitialState, this, _1,_2));
        
    }
    
    /**
     * Destructor
     */
    ~ExecutionAgent(){}
    
     
    
    
    //ACTIONS CALLBACKS
    
    /**
     * This is the main callback. It is called when the server receives
     * a goal, and it is responsible for doing everything related to the 
     * execution application.
     * @param goal
     */
    void executeCB(const execution::ExecutionGoal::ConstPtr &goal)
    {
        printf("\nGOAL RECEIVED! \n");
         
         
        //The action finishes when the a new plan is generated
        // We can specify whatever we want here as an action

        // Rate = 10 Hz
        ros::Rate r(10);
        
        gotNewPlan = false;
        finished = false;
        success = false;    
        
        //TODO: WHAT DOES THE EXECUTION SERVER CAN PROVIDE AS A SERVER
        
        //request for replan
        if (goal->request == "replan"){
        	
        	feedback_.status = REQUESTINGPLAN;
			//publish the feedback
			as_.publishFeedback(feedback_);
			
			currentstateid = REQUESTINGPLAN;


			//Wait for other services
			//ROS_INFO("Waiting for planning server to start.");
			// wait for the action server to start
			//planningclient.ac.waitForServer(); //will wait for infinite time
			//ROS_INFO("Planning server is running.");

			 
			// While the execution process is not finished       
			//while (!finished){
			while (!gotNewPlan){

				// If the bingo game has been preempted
				if (as_.isPreemptRequested() || !ros::ok())
				{                
					ROS_INFO("%s: Preempted", action_name_.c_str());
					as_.setPreempted();
					success = false;
					finished = true;
					currentstateid = IDLE;
					//TODO: stop the execution

					printf("\nStatus: (%i) Waiting New Request \n",currentstateid);
					
					break;
				}            

				//doUpdate2();
				
				//publish the feedback
				as_.publishFeedback(feedback_);

				// Guarantees the 10Hz rate
				r.sleep();
			}
        	
        }
        //THIS BELOW IS CURRENTLY BEEN PROVIDED AS A SERVICE AS WELL
        //Request for generating/getting the current state of the agents.
        else if (goal->request == "current_state"){
        	saveCurrentStateForPlanner();
            
            result_.statusCode = 1;
            success = true;
            //TODO: send feedback? handle error cases here.            
        }                        
        
        
        
        // If the execution is over and everything went as expected
        if(success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }

    }

    
    //MESSAGES CALLBACKS

    /**
      * This function is called when a plan is generated by the planning module.
      */
    //void planCallback(const planning::Plan::ConstPtr& planmsg)
    void planCallback(const server_msgs::Plan::ConstPtr& planmsg)
    {
        printf("\nPlan Received: code [%i]\n", planmsg->code);

        if (planmsg->solutionExists){
            printf("REQUESTER: [%s]\n", planmsg->requester.c_str());
            printf("Plan: \n[%s]", planmsg->plan.c_str());
        }
        else if (planmsg->isUnsolvable){
             printf(" (!) Problem is unsolvable");
        }
        
        //TODO: check the code to see if the plan is to be added, rewrote, etc
        //Create the plan for execution
        currentplan.setActions(planmsg->plan.c_str());
        planrequester = planmsg->requester;


        //finished = false;
        //success = false;

        finished = true;
        success = true;        

        planindex = 0;
        timeindex = 0.0;


        gotNewPlan = true;
    }
    
    
    /**
	  * This function is called when a plan is generated by the planning module.
	  */
    /*
	void currentStateRequestCallback(const server_msgs::CurrentStateRequest::ConstPtr& msg)
	{
		printf("\nCurrent State Request Received: code [%i]\n", msg->code);
		
		currentstaterequester = msg->requester;
		//save the problem instance
		saveCurrentStateForPlanner();
	}
	*/
    
    
	
	//SERVICES CALLBACKS
	
	/**
	 * This callback is called when another node wants the the current problem instance (PDDL) as a string
	 */
	bool getInitialState(server_msgs::InitialStateRequest::Request  &req,
	    		server_msgs::InitialStateRequest::Response &res)
	{
	  ROS_INFO("Service request received from %s", req.requester.c_str());	  
	  res.state = getCurrentStateForPlanner();
	  //ROS_INFO("sending back response: \n%s",res.state.c_str());
	  return true;
	}
	
	
	
	
	
	



    /**
     * This function updated the status of the execution agent
     **/
    void doUpdate(){

        //

        switch (currentstateid) {

            case IDLE: //Status: Idle
            {    //Print state
                if (previousstateid != IDLE){
                    printf("\nStatus: (%i) Idle. Waiting for execution request \n",currentstateid);
                }
                feedback_.status = IDLE;

                //set default viariables
                //finished = true;
                //success = false;

                if(gotNewPlan){

                    currentstateid = EXECUTING;
                    //reset the plan index and time
                    planindex = 0;
                    timeindex = 0.0;

                }

                previousstateid = IDLE;
                break;
            }

            case EXECUTING: //Status: Executing Actions
            {
                //Print state
                if (previousstateid != EXECUTING){
                   printf("\nStatus: (%i) Executing plan \n",currentstateid);
                }
                feedback_.status = EXECUTING;


                printf("\nTIME: %f. \n",timeindex);

                bool continueExecuting = true;
                bool doReplanning = false;
                
                bool hasActionsToExecute = currentplan.hasActionsToExecute();
                
                //1) CHECK NEW ACTIONS TO BE SENT
                //Check if the plan has not finished yet
                if (hasActionsToExecute){
                	

                    if (timeindex == currentplan.getNextActionsStartTime()){
                        vector< Action > nextactions = currentplan.getNextActions();
                        for(int i=0; i<nextactions.size(); ++i){
                            Action a = nextactions.at(i);
                            printf(" Executing action (index %i) at %f. \n",currentplan.index,timeindex);
                            printf(" Command: %s \n",a.rawname.c_str());
                            a.print();
                            planindex ++;
                            currentplan.index ++;


                            //send the action to the robot
                            //RequestManagerClient* rb = robots.at(0);
                            // Get the corresponding robot client from the map
                            //RequestManagerClient* robot_client = robots_map.left.at("r1");
                            RequestManagerClient* robot_client = robots_map.left.at(a.agent);
                            // send a goal to the action
                            if (robot_client->ready && robot_client->ac.isServerConnected()){
								request_manager::RequestManagerGoal request_manager_goal;
								request_manager_goal.newrequest = 1;
								request_manager_goal.command = a.rawname;
								request_manager_goal.info = setCommandInfo(a); //set up the additional info
								robot_client->sendGoal(request_manager_goal);
                            }
                            else{                            	
                            	//The client is not ready for a action new
                            	printf("Robot %s is not ready to receive another command", a.agent.c_str());
                            	
                            	//Following a replanning approach if the robot is not ready or if the robot is not connected. 
                            	continueExecuting = false;
                            	doReplanning = true;
                            	
                            }
                        }
                    }                    
                    else{
                        printf(" Nothing to execute at time %f. \n",timeindex);
                    }
                }
                
                
                //2) CHECK ACTIVITIES THAT ARE RUNNING. The plan finished
                //TODO: what if the robot is running but it no connected anymore. 
                //      For example the robot was running/executing something and it disconnected/shoutdown
                if(!hasActionsToExecute && !isAnyClientRunning()) {                	
					continueExecuting = false;	
					printf(" Plan seems to be over. No more actions be execute and no client running.\n");
				}
                
                
                //3) CHECK if any client has failed to Execute
                if (anyClientFailed()){
                	printf(" A client has failed to execute the command.\n");
                	continueExecuting = false;
                	doReplanning = true;                	
                }
                
                
                
                //Check which state we should go if we should stop executing: finish or replanning? 
                if (!continueExecuting){
                	
                	if (doReplanning){
                		printf("\nReplanning will be called! \n");
                		currentstateid = REQUESTINGPLAN;
                		
                	}else{// we have finish the execution 
                		printf("\nExecution done! \n");
						currentstateid = FINISHED;
                	}                    
                }
                

                sleep(2);
                
                timeindex ++;
                previousstateid = EXECUTING;
                gotNewPlan = false;
                break;
            }


            case REQUESTINGPLAN: //Status: Requesting Plan
            {    //Print state
                if (previousstateid != REQUESTINGPLAN){
                    printf("\nStatus: (%i) Requesting New Plan \n",currentstateid);
                }
                feedback_.status = REQUESTINGPLAN;

                //1) Get current state and the goal (they are saved in the problem.pddl file)                
                saveCurrentStateForPlanner();

                //2) Send a goal to the planning module
                /// USING CLIENT
                planning::PlanningGoal planning_goal;
                planning_goal.newrequest = 1;
                planning_goal.sender = action_name_;
                planningclient.sendGoal(planning_goal);
				
                //USING TOPIC/MESSAGE
                //plan_request_.code = 1;
                //plan_request_.requester = action_name_;
                //plan_request_pub.publish(plan_request_);

                gotNewPlan = false;

                currentstateid = WAITINGPLAN;
                //currentstateid = IDLE; // I can go directly to the idle if necessary
                previousstateid = REQUESTINGPLAN;
                break;
            }


            case WAITINGPLAN: //Status: Waiting for plan
            {
                //Print state
                if (previousstateid != WAITINGPLAN){
                    printf("\nStatus: (%i) Waiting for Plan \n",currentstateid);
                }
                feedback_.status = WAITINGPLAN;


                if(gotNewPlan){

                   // if (currentplan.size() > 0){

                        //Check who requested the plan in order to finish open precesses
                        // if it was the execution node, then wait for the client to finish
                		printf("Requester %s, this %s",planrequester.c_str(), action_name_.c_str());
                        if (planrequester == action_name_){
                        	printf("Plan requested by me arrived");
                        	///* USING CLIENT
                            //When the planning module has responded to the execution request we read
                            // the output
                            if (planningclient.done){ //TODO: we might need to rely only on the new request
                               printf(" >> Planning module has responded to the replanning request! \n");


                               //Get the Plan from the planning module
                               actionlib::SimpleClientGoalState state = planningclient.ac.getState();
                               ROS_INFO("Getting Result\n");
                               planning::PlanningResult result;
                               result = *planningclient.ac.getResult();

                               ROS_INFO("Action finished: %s",state.toString().c_str());
                               //ROS_INFO("Result (Plan): \n%s",result.plan.c_str());

                               //set the output plan as the result of the request
                               //result_.plan = result.plan.c_str();
                               //result_.solutionExists = result.solutionExists;
                              // result_.isUnsolvable = result.isUnsolvable;
                               result_.statusCode = 1;

                               

                               currentstateid = EXECUTING;
                            }
                            //*/
                        }
                        else{

                            printf("\nPlan came from external request!\n");
                            //check if the planning client is still running
                            ///* USING CLIENT
                            if (planningclient.running){
                                planningclient.ac.cancelGoal();
                            }
                            //*/

                            currentstateid = EXECUTING;
                        }

                    //}
                    //if the plan is empty we keep waiting for a valid one
                    //else{
                    //    gotNewPlan = false;
                    //    currentstateid = IDLE;
                    //}

                    //currentstateid = EXECUTING;
                    //reset the plan index and time
                    planindex = 0;
                    timeindex = 0.0;

                }

                previousstateid = WAITINGPLAN;
                break;
            }


            case FINISHED: //Status: Execution Finished
            {
                //Print state
                if (previousstateid != FINISHED){
                    printf("\nStatus: (%i) Execution Finished \n",currentstateid);
                }
                feedback_.status = FINISHED;

                gotNewPlan = false;

                //I want to finish the callback here
                finished = true;
                success = true;

                currentstateid = IDLE;

                previousstateid = FINISHED;
                break;
            }


            default:
              // Code
              break;
        }

    }
    
    
      
    
    
    /**
     * This function sets up important/additional information about the action and each object involded in the action.
     * TODO: the object type must come from somewhere and should not be hardcoded
     */
    vector< string > setCommandInfo(Action act){
    	
    	vector<string> info;
       	
    	if (act.name == "move"){
    		//set up the coordanates for the location
    		
    		//set the origin location info
    		//string infostr = getObjectCommandInfo(act.objects.at(0),&map_of_locations);
    		string infostr = db.getObjectInfo(act.objects.at(0),"location");
    		printf("Origin (%s) coordinates: %s\n",act.objects.at(0).c_str(), infostr.c_str());    		
    		if (infostr != "")
    			info.push_back(act.objects.at(0)+":"+infostr);
    		
    		//set the target location info
    		//infostr = getObjectCommandInfo(act.objects.at(1),&map_of_locations);
    		infostr = db.getObjectInfo(act.objects.at(1),"location");
			printf("Target (%s) coordinates: %s\n",act.objects.at(1).c_str(),infostr.c_str());    		
			if (infostr != "")
				info.push_back(act.objects.at(1)+":"+infostr);

    	}
    	
    	else if (act.name == "dotelepresence"){
    		
    		//set session info
    		string infostr = db.getObjectInfo(act.objects.at(0),"session");
			printf("Session: %s\n",infostr.c_str());    		
			if (infostr != "")
				info.push_back(act.objects.at(0)+":"+infostr);
			
			//set user info
			infostr = db.getObjectInfo(act.objects.at(1),"user");
			printf("User: %s\n",infostr.c_str());    		
			if (infostr != "")
				info.push_back(act.objects.at(1)+":"+infostr);
			
			//set session location info
			infostr = db.getObjectInfo(act.objects.at(2),"location");
			printf("Location: %s\n",infostr.c_str());    		
			if (infostr != "")
				info.push_back(act.objects.at(2)+":"+infostr);
	
    	}
    	    	
    	else if (act.name == "playbingo"){
    		//TODO: get bingo
    		//set game info
    		string infostr = db.getObjectInfo(act.objects.at(0),"game");
			printf("Game: %s\n",infostr.c_str());    		
			if (infostr != "")
				info.push_back(act.objects.at(0)+":"+infostr);
			
			//set game location info
			infostr = db.getObjectInfo(act.objects.at(1),"location");
			printf("Location: %s\n",infostr.c_str());    		
			if (infostr != "")
				info.push_back(act.objects.at(1)+":"+infostr);
    	}
    	    	
    	else if (act.name == "remind"){
    		//TODO: get remind
    		//set user info
    		string infostr = db.getObjectInfo(act.objects.at(0),"user");
			printf("User: %s\n",infostr.c_str());    		
			if (infostr != "")
				info.push_back(act.objects.at(0)+":"+infostr);
			
    		//set game info
			infostr = db.getObjectInfo(act.objects.at(1),"game");
			printf("Game: %s\n",infostr.c_str());    		
			if (infostr != "")
				info.push_back(act.objects.at(1)+":"+infostr);			
    		
    	}
    	
    	
    	return info;
    }
    
    
    
    /**
	 * This function check whether there is any client running a activity
	 */
	bool isAnyClientRunning(){
		
		for (robots_bimap::iterator it = robots_map.begin(); it!=robots_map.end(); ++it){
			//std::cout << "     - Checking " << it->left << std::endl;
			RequestManagerClient* robot_client = it->right;
			//check it is running and connected 
			//TODO: WHAT IF IT IS NOT CONNECTED
			if (robot_client->running && robot_client->ac.isServerConnected()){
				std::cout << "     - " << it->left << " is running " << std::endl;
				return true;
			}
			else if (!robot_client->ac.isServerConnected()){
				std::cout << "     - " << it->left << " lost connection with the server! " << std::endl;
			}
		}    	
		return false;
	}
	
	
	/**
	 * This function check whether there is any client running a activity
	 */
	bool anyClientFailed(){
		bool result = false;
		
		for (robots_bimap::iterator it = robots_map.begin(); it!=robots_map.end(); ++it){
			//std::cout << "     - Checking " << it->left << std::endl;
			RequestManagerClient* robot_client = it->right;
			if (!robot_client->running && robot_client->done && 
					robot_client->succeeded == false && 
					robot_client->currentResult.statusCode == 0){
				std::cout << "     - " << it->left << " finished with a FAILURE!" << std::endl;
				robot_client->reset();
				result = true;
			}    		
		}    	
		return result;
	}
	

	
    /**
	 * This function request an update from every client.
	 */
	void updateClients(){
		
		for (robots_bimap::iterator it = robots_map.begin(); it!=robots_map.end(); ++it){
			std::cout << "     - Updating status from " << it->left << std::endl;
			string agentname = it->left;
			RequestManagerClient* robot_client = it->right;
			//if the client is not running we request an update
			if (!robot_client->running){
				//check if the client is connected with the corresponding server/agent
				if (robot_client->ac.isServerConnected()){
					//send the update goal
					request_manager::RequestManagerGoal request_manager_goal;
					request_manager_goal.newrequest = 1;
					request_manager_goal.command = "update_status";
					robot_client->sendGoal(request_manager_goal);
					
					//Wait for the action to return
					bool finished_before_timeout = robot_client->ac.waitForResult(ros::Duration(10.0));
					if (finished_before_timeout){
						actionlib::SimpleClientGoalState state = robot_client->ac.getState();
						printf("Getting Result from %s.\n",agentname.c_str());
						request_manager::RequestManagerResult result;
						result = *robot_client->ac.getResult();					
						printf("Update finished: %s \n",state.toString().c_str());
						//ROS_INFO("Result (Plan): \n%s",result.plan.c_str());  								  
					}
					else{
						printf("Update from %s did not finish before the time out.\n",agentname.c_str());    
						//Cancelling the goals
						robot_client->ac.cancelAllGoals();
						printf("Update Canceled\n");				
					}
				}
				else{
					printf("Agent %s is connected.\n",agentname.c_str());  
				}
			}
			else{
				printf("Agent %s is running.\n",agentname.c_str());  
			}
				
			
		}    	
	}	
    
    

    /**
     *  This function creates the problem instance file based on
     *  the current state of the robot and the current goal
     */
    string createProblemInstanceContent(){
    	
    	printf("Creating the PDDL file.\n");
    
        string goalfile = plannerspath + "/goal.txt";

        
        //PDDL PROBLEM INSTANCE HEADER
        string content;
        content =  ";;Problem instance created by the Execution Module.\n\n";
        
        content += "(define (problem Problem1-demo)\n";
        //content += "    (:domain TelepresenceDomainv14)\n";
        content += "    (:domain MultiActivityDomainv4)\n";
        
        
        //PDDL OBJECTS
        string pddlobjs;
        pddlobjs =  "    (:objects \n";
        //TODO: get the objects from the db and from the clients
        pddlobjs += db.getCurrentObjectsInPDDL();        
        pddlobjs += "    )\n";
                                   
        

        //PDDL INITIAL/CURRENT STATE
        string pddlinit;
        pddlinit =  "    (:init \n";
             
        //Global/Static info
        pddlinit += "      ;;Global info\n"; 
        pddlinit += db.getCurrentGlobalVariablesInPDDL();
        
        //Locations - Map topology
        pddlinit += "      ;;Locations\n";
        //TODO: get map information from the db, file or a different source. It cannot be hardcoded here.    
        pddlinit += db.getCurrentLocationsInPDDL();
        //distance        

        //Charging stations
        pddlinit += "      ;;Charging stations\n";
		pddlinit += db.getCurrentStationsInPDDL();  //Get it from db/xml                                              
                       
        //Users 		
		pddlinit += "      ;;Users\n"; 
		pddlinit += db.getCurrentUsersInPDDL();  //Get it from db/xml        
        				
		//TODO:: put all the activities in a single xml file (?)
        //Telepresence Sessions        
        pddlinit += "      ;;Telepresence Sessions\n";        
        pddlinit += db.getCurrentSessionsInPDDL(); //Get it from db/xml
        
        //Telepresence Sessions        
        pddlinit += "      ;;Games\n";        
        pddlinit += db.getCurrentGamesInPDDL(); //Get it from db/xml  
        
        
        //Robots
        pddlinit += "      ;;Robots\n";
        for (robots_bimap::iterator it = robots_map.begin(); it!=robots_map.end(); ++it){
			RequestManagerClient* robot_client = it->right;
			string agentname = it->left;
			
			//TODO: Robot information must come and go in XML to make the code general/flexible. That is the agent has to mantain
			//      an updated xml node with information/data from the robot in the form of attributes. Them we can transform 
			//      those xml attributes (with real time data) into pddl here without having to hardcode the properties
			
			//check if the client is connected to the agent/server/robot
			if (robot_client->ac.isServerConnected()){

				printf("::Robot %s.\n",agentname.c_str());
				printf("  - x: %f.\n",robot_client->currentFeedback.x_position);
				printf("  - y: %f.\n",robot_client->currentFeedback.y_position);
				
				pddlinit += "      ;; robot "+agentname+" \n";
				
				//TODO: the agent/robot information must com from the client in the form of a PDDL:
				//robot's location					
				string agentlocation = db.coordinatesToLocation(robot_client->currentFeedback.x_position,robot_client->currentFeedback.y_position); //string agentlocation = "l0";				
				//Getting info from xml string
				string robot_status = "	<robot id=\""+agentname+"\">\n "						
							   "		<attribute name=\"name\" type=\"string\" value=\"robot1\"/> \n"
						       "		<attribute name=\"at\" type=\"Location\" value=\""+agentlocation+"\" pddl=\"true\"/> \n"
						       "		<attribute name=\"ready\" type=\"bool\" value=\"true\" pddl=\"true\"/> \n"	
							   "		<attribute name=\"batterylevel\" type=\"int\" value=\""+boost::str(boost::format("%1$d") % robot_client->currentFeedback.batterylevel)+"\" pddl=\"true\"/> \n"
							   "		<attribute name=\"batterycapacity\" type=\"int\" value=\""+boost::str(boost::format("%1$d") % robot_client->currentFeedback.batterycapacity)+"\" pddl=\"true\"/> \n"							   
							   "		<attribute name=\"velocity\" type=\"int\" value=\"1.00\" pddl=\"true\"/> \n"
							   "		<attribute name=\"consumptionratio\" type=\"int\" value=\"20.00\" pddl=\"true\"/> \n"
						       "		<attribute name=\"consumptionratiosession\" type=\"int\" value=\"20.00\" pddl=\"true\"/> \n"
					       	   "		<attribute name=\"rechargeratio\" type=\"int\" value=\"1.00\" pddl=\"true\"/> \n"							
							   " 	</robot>";
				
				//parse the xml string to facts 
				xmlDocPtr doc=xmlParseDoc((xmlChar *) robot_status.c_str());				
				xmlNode *root = xmlDocGetRootElement(doc);				
				//std::cout<< xmlObjectAttributesToPDDL(root);
				pddlinit += xmlObjectAttributesToPDDL(root);
				
												
				/*
				pddlinit += "      (at "+agentname+" "+agentlocation+") \n";
				//robot availability
				pddlinit += "      (ready "+agentname+") \n";
				//robot's battery
				pddlinit += "      (= (batterylevel "+agentname+") "+boost::str(boost::format("%1$d") % robot_client->currentFeedback.batterylevel)+") \n";//50
				pddlinit += "      (= (batterycapacity "+agentname+") "+boost::str(boost::format("%1$d") % robot_client->currentFeedback.batterycapacity)+") \n"; //50				
				//robot's velocity
				pddlinit += "      (= (velocity "+agentname+") 1.00) \n"; 
				pddlinit += "      (= (consumptionratio "+agentname+") 20.00) \n";  
				pddlinit += "      (= (consumptionratiosession "+agentname+") 20.00) \n";
				//robot's recharge ratio
				pddlinit += "      (= (rechargeratio "+agentname+") 1.00) \n";
				*/				
				
				/*
				pddlinit += "      (at r1 l0) \n";
				pddlinit += "      (ready r1) \n";  
				pddlinit += "      (= (batterylevel r1) 50) \n";  
				pddlinit += "      (= (batterycapacity r1) 50) \n";  
				pddlinit += "      (= (velocity r1) 10.00) \n";  
				pddlinit += "      (= (consumptionratio r1) 10.00) \n";  
				pddlinit += "      (= (rechargeratio r1) 1.00) \n";  
				pddlinit += "      (= (consumptionratiosession r1) 10.00) \n"; 
				*/ 

				
			}
			else{
				printf(" (!) Robot %s is not connected!\n",agentname.c_str());
				pddlinit += "      ;; robot "+agentname+" is not connected! \n";
			}
	        pddlinit += "      \n"; 
		}

                
        //end of init
        pddlinit += "    )\n";
                

        //PDDL GOAL        
        string pddlgoal;
        pddlgoal =  "    (:goal \n";
        //get the goal from the file
        std::ifstream ifs(goalfile.c_str());
        std::string goalstr( (std::istreambuf_iterator<char>(ifs) ),(std::istreambuf_iterator<char>()) );
        pddlgoal += goalstr;        
        pddlgoal += "    )\n";
        
        
        
        //PUUTING ALL TOGETHER
        content += pddlobjs;
        content += pddlinit;
        content += pddlgoal;
        content += ")";

        
        return content;
    }
    
       
    
	/**
	 * This funtion get the lastest information/status from the robots and create the content of the problem instance file 
	 */
	string getCurrentStateForPlanner(){
		ROS_INFO("Getting current state.");
		updateClients();
		ROS_INFO("Get problem instance.");
		string content = createProblemInstanceContent();
		return content;
	}
	
	    
    /*
     * This function reads and save the problem instance (PDDL) in the planner's folder
     */
    void saveCurrentStateForPlanner(){
		string outputproblemfile = plannerspath + "/problem.pddl";
		
    	string content = getCurrentStateForPlanner();
    	//Save pddl problem file
	//If file is already created by another program or manually then just comment the code below.
		ofstream myfile;
		myfile.open(outputproblemfile.c_str());
		myfile << content;
		myfile.close();      
    }
    
    

};


int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "execution");
    ros::NodeHandle nh;

    printf("Starting the execution module!!!\n");
    //sleep(2);
    
    //Get parameters
    std::string path;
    nh.getParam("path", path);   
    //printf("PATH: %s\n",path.c_str());
    
    std::string plannerspath;
    nh.getParam("plannerspath", plannerspath);   

    //Initialize execution agent
    ExecutionAgent agent(ros::this_node::getName());
    //printf("Agent ready? %s",(agent.ready)?"true":"false");
    agent.filepath = path;
    agent.plannerspath = plannerspath;
    
    //Database object
    //agent.getData(); //get map locations
    Database db(path);
    agent.db = db;    

    //RequestManagerClient robotclient("request_manager");
    //agent.robots.push_back(new RequestManagerClient("request_manager"));

    //Creats a map of robots/agents (name -> client) 
    //Definition added to the top of this file
    //typedef boost::bimap< std::string, RequestManagerClient* > robots_bimap;
    //typedef robots_bimap::value_type map_value;
    
    // Add robot r1 (id/name and its corresponding robot manager client)
    agent.robots_map.insert( map_value("r1",new RequestManagerClient("request_manager")));

    
    ROS_INFO("Execution module is running");

    //ros::spin();


    // Rate = 10 Hz
    ros::Rate r(10);
    while (nh.ok()) {
        agent.doUpdate();
        ros::spinOnce();
        r.sleep();
    }

    
    return 0;
}


