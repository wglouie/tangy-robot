
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
#include <time.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/date_time.hpp>

#include <request_manager/mylib.h>
#include <request_manager/RequestManagerAction.h>
#include <request_manager/activity.h>




using namespace std;


enum RequestManagerStates { IDLE , REQUESTING , MOVING, DOINGTELEPRESENCE, PLAYINGBINGO,  REMINDING, EXECUTING, FINISHED };



class RequestManagerAgent
{
    
protected:
    ros::NodeHandle nh_;
    
    // create the Robot's Request Manager server
    actionlib::SimpleActionServer<request_manager::RequestManagerAction> as_;
    std::string action_name_;
    
public:
    
    // attributes
    bool ready;
    bool finished;
    bool success;
    bool newrequest;
    //state variables
    int currentstateid;
    int previousstateid;
    
    string filepath;

    Activity currentactivity;

    int counter;


          
    // create messages that are used to publish feedback/result
    request_manager::RequestManagerFeedback feedback_;
    request_manager::RequestManagerResult result_;


    /**
     * Constructor
     **/
    RequestManagerAgent(string name):
            as_(nh_ , name , boost::bind(&RequestManagerAgent::executeCB , this , _1) , false) ,
            action_name_(name)
	{
        printf("Initializing robot's request manager agent.\n");
        ready = true;
        currentstateid = IDLE;//0;
        previousstateid = -1;
        
        finished = true;
        success = false;
        
        newrequest = false;

        //starts the server
        as_.start();

    }
    
    /**
     * Destructor
     */
    ~RequestManagerAgent(){}
    
    /**
     * This is the main callback. It is called when the server receives
     * a goal, and it is responsible for doing everything related to the 
     * request_manager application.
     * @param goal
     */
    void executeCB(const request_manager::RequestManagerGoal::ConstPtr &goal)
    {
        printf("\nGOAL RECEIVED\n");

        string command = goal->command;

        currentactivity.setActivityInfo(command);
         
        // Rate = 10 Hz
        ros::Rate r(10);
        
        newrequest = true;
        finished = false;
        success = false;

        feedback_.status = 0;
        feedback_.internal_state = REQUESTING;
        //publish the feedback
        as_.publishFeedback(feedback_);
        
        currentstateid = REQUESTING;

        counter = 0;

         
        // While the the execution process of an activity is not finished
        while (!finished){

            // If the bingo game has been preempted
            if (as_.isPreemptRequested() || !ros::ok())
            {                
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                finished = true;
                currentstateid = IDLE;                
                //TODO: stop the execution of the current activity

                printf("\nStatus: (%i) Idle, waiting New Request \n",currentstateid);
                
                break;
            }            

            doUpdate();
            
            //publish the feedback
            as_.publishFeedback(feedback_);

            // Guarantees the 10Hz rate
            r.sleep();
        }
        
        // If the execution of the activity is over and everything went as expected
        if(success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }

    }


    /**
     * This function updated the status of the request_manager agent
     **/
    void doUpdate(){
       

        switch (currentstateid) {

            case REQUESTING: //Status: Requesting activity
            {    //Print state
                if (previousstateid != REQUESTING){
                    printf("\nStatus: (%i) Requesting Activity \n",currentstateid);
                }
                feedback_.status = 0;
                feedback_.internal_state = REQUESTING;
                feedback_.ready = false;
        		feedback_.action = currentactivity.name;
        		feedback_.command = currentactivity.rawname;
        		setCommonFeedback();

                printf("\nActivity: %s\n",currentactivity.name.c_str());

                if (currentactivity.name == "update_status"){
                	currentstateid = FINISHED;
                } 
                
                //Activities
                else if (currentactivity.name == "move"){
                    //TODO: Send the goal to the navigation server

                    currentstateid = MOVING;
                }
                else if (currentactivity.name == "dotelepresence"){
                    //TODO: Send the goal to the telepresence server

                    currentstateid = DOINGTELEPRESENCE;
                }                
                else if (currentactivity.name == "playbingo"){
                    //TODO: Send the goal to the telepresence server

                    currentstateid = PLAYINGBINGO;
                }
                else if (currentactivity.name == "remind"){
                    //TODO: Send the goal to the telepresence server

                    currentstateid = REMINDING;
                }                
                          
                else {
                    printf("\nI don't know how to execute activity '%s' \n",currentactivity.name.c_str());
                    currentstateid = FINISHED;

                }

                previousstateid = REQUESTING;
                break;
            }

            
            //ACTIVITIES
            case MOVING: //Status: Executing Acticity
            {
            	
                //Print state
                if (previousstateid != MOVING){
                   printf("\nStatus: (%i) Executing activity MOVE\n",currentstateid);
                   saveCommandInAFile(currentactivity.rawname);
                }
                feedback_.status = 1;
                feedback_.internal_state = MOVING;
				feedback_.action = currentactivity.name;
				feedback_.command = currentactivity.rawname;
				feedback_.ready = false;
				setCommonFeedback();



                // Check if the acivity is done
                if (counter >= currentactivity.duration  + 100){
                    currentstateid = FINISHED;
                }

                counter++;


                previousstateid = MOVING;
                break;
            }

            case DOINGTELEPRESENCE: //Status: Executing Acticity
            {
                //Print state
                if (previousstateid != DOINGTELEPRESENCE){
                   printf("\nStatus: (%i) Executing activity TELEPRESENCE\n",currentstateid);
                   saveCommandInAFile(currentactivity.rawname);
                }
                feedback_.status = 1;
                feedback_.internal_state = DOINGTELEPRESENCE;
				feedback_.action = currentactivity.name;
				feedback_.command = currentactivity.rawname;
				feedback_.ready = false;
				setCommonFeedback();

                // Check if the acivity is done
                if (counter >= currentactivity.duration + 100){
                    currentstateid = FINISHED;
                }

                counter++;


                previousstateid = DOINGTELEPRESENCE;
                break;
            }

            case PLAYINGBINGO: //Status: Executing Acticity
			{
				//Print state
				if (previousstateid != PLAYINGBINGO){
				   printf("\nStatus: (%i) Executing activity BINGO\n",currentstateid);
				   saveCommandInAFile(currentactivity.rawname);
				}
				feedback_.status = 1;
				feedback_.internal_state = PLAYINGBINGO;
				feedback_.action = currentactivity.name;
				feedback_.command = currentactivity.rawname;
				feedback_.ready = false;
				setCommonFeedback();

				// Check if the acivity is done
				if (counter >= currentactivity.duration + 100){
					currentstateid = FINISHED;
				}

				counter++;


				previousstateid = PLAYINGBINGO;
				break;
			}
			
            case REMINDING: //Status: Executing Acticity
			{
				//Print state
				if (previousstateid != REMINDING){
				   printf("\nStatus: (%i) Executing activity REMIND\n",currentstateid);
				   saveCommandInAFile(currentactivity.rawname);
				}
				feedback_.status = 1;
				feedback_.internal_state = REMINDING;
				feedback_.action = currentactivity.name;
				feedback_.command = currentactivity.rawname;
				feedback_.ready = false;
				setCommonFeedback();

				// Check if the acivity is done
				if (counter >= currentactivity.duration + 20){
					currentstateid = FINISHED;
				}

				counter++;


				previousstateid = REMINDING;
				break;
			}

            case FINISHED: //Status: Activity Finished
            {
                //Print state
                if (previousstateid != FINISHED){
                    printf("\nStatus: (%i) Activity Finished \n",currentstateid);
                }
                feedback_.status = 2;
                feedback_.internal_state = FINISHED;
				feedback_.action = currentactivity.name;
				feedback_.command = currentactivity.rawname;
				feedback_.ready = true;
				setCommonFeedback();
		
				result_.statusCode = 1;

                newrequest = false;
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
     * This function gets sets the x,y positions of the robot and the battery level
     **/
    void setCommonFeedback(){
		//TODO: get battery info
		//feedback_.batterylevel = 17.500;
		//feedback_.batterycapacity = 19.100;
    	feedback_.batterylevel = 1000.00;
    	feedback_.batterycapacity = 1000.00;
		
		//TODO: get position info
		feedback_.x_position = 1.001;
		feedback_.y_position = 1.001;
    }
    
    
    
    void saveCommandInAFile(string cmd){
    	
		
		time_t tim;  //create variable of time_t
		time(&tim); //pass variable tim to time function
		cout << ctime(&tim); // this translates what was returned from time() into a readable format

	 		
    	//write in the file
		std::ofstream outfile;
		string file = filepath + "/experiment.txt";
		printf("File: %s\n",file.c_str());
		//outfile.open("/home/tangerine/ros/tangy/src/navigation/navigation_experiment/experiment/navigation_experiment.txt", std::ios_base::app);
		outfile.open(file.c_str(), std::ios_base::app);
		outfile << cmd + "\n";
		outfile << ctime(&tim) << ": " +cmd + "\n";
		
		outfile.close();
    	
    	
    }
    
       
    


};


int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "request_manager");
    ros::NodeHandle nh;

    printf("Starting the request_manager module!!!\n");
    //sleep(2);

    //Get parameters
    std::string path;
    nh.getParam("path", path);

    //Initialize robot manager agent
    RequestManagerAgent agent(ros::this_node::getName());
    //printf("Agent ready? %s",(agent.ready)?"true":"false");
    agent.filepath = path;
    
    ROS_INFO("Robot's Request Manager module is running");
    ros::spin();
    
    return 0;
}


