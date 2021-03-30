
/**
 * Author: Tiago Vaquero, Rafael Farias
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
#include <boost/algorithm/string.hpp>

#include <boost/date_time.hpp>

#include <boost/bimap.hpp> //maps - key -> value

#include <request_manager/mylib.h>
#include <request_manager/RequestManagerAction.h>
#include <request_manager/activity.h>

//robot_gui client
#include <robot_gui/robot_gui_client.h>

//Telepresence Client
#include <telepresence_activity/telepresence_activity_client.h>

//Navigation Client
#include <tangerine_2dnav/navigation_client.h>

//battery info
#include <drrobot_h20_player/PowerInfo.h>

//odometry info
#include <nav_msgs/Odometry.h>


using namespace std;


enum RequestManagerStates { IDLE , REQUESTING , MOVING , DOINGTELEPRESENCE , UPDATING , EXECUTING , FINISHED };

typedef boost::bimap< std::string, vector< float > > coordinates_bimap;
typedef coordinates_bimap::value_type map_value;

typedef boost::bimap< std::string, std::string > info_bimap;
typedef info_bimap::value_type info_map_value;


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

    bool activity_succeeded;
    //state variables
    int currentstateid;
    int previousstateid;
    
    string filepath;

    Activity currentactivity;


    request_manager::RequestManagerGoal Ngoal;

    //interface client and goal
    RobotGuiClient *robotGuiClient;
    robot_gui::Robot_guiGoal interface_goal;

    //navigation client and goal
    NavigationClient *navigationClient;
    move_base_msgs::MoveBaseGoal navigation_goal;

    //telepresence client and goal
    TelepresenceActivityClient *telepresenceClient;
    telepresence_activity::Telepresence_activity_serverGoal telepresence_goal;
          
    // create messages that are used to publish feedback/result
    request_manager::RequestManagerFeedback feedback_;
    request_manager::RequestManagerResult result_;

    //declaring the map of coordinates
    boost::bimap< std::string, vector<float> > map; // a map of coodinates (name -> coordinates)
    boost::bimap< std::string, std::string> info_map; //

    //battery info
    double bat1_vol_;
    double bat2_vol_;
    //double bat1_temp_;
    //double bat2_temp_;
    //double dcin_vol_;
    //double ref_vol_;
    //unsigned short int power_status_;
    //unsigned short int power_path_;
    //unsigned short int charge_path_;
    drrobot_h20_player::PowerInfo powerInfo_;
    ros::Subscriber powerInfo_sub;

    //current position on the map
    double current_x_position;			// x-position real world
    double current_y_position;			// y-position real world

    ros::Subscriber odom_sub;

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
        activity_succeeded = false;

        //starts the server
        as_.start();

	//Clients

	//initialize the navigation client
	navigationClient = new NavigationClient("navigation_client");

	//initialize the telepresence activity client
	telepresenceClient = new TelepresenceActivityClient("telepresence_activity_client");

	//initialize the gui client
	robotGuiClient = new RobotGuiClient("GUI_client");

	//Topics that it subscribes
	powerInfo_sub = nh_.subscribe("drrobot_powerinfo",100, &RequestManagerAgent::updatePowerStatus, this);
	odom_sub = nh_.subscribe("odom", 100, &RequestManagerAgent::updatePositionStatus, this); 

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

	Ngoal.coordinates = goal->coordinates;
	Ngoal.info = goal->info;


	string command = goal->command;

	currentactivity.setActivityInfo(command);

	create_infoMap();

        // Rate = 10 Hz
        ros::Rate r(10);
        
        newrequest = true;
        finished = false;
        success = false;

	//as_.acceptNewGoal();

        
        feedback_.status = 0;
        feedback_.internal_state = REQUESTING;

        //publish the feedback
        as_.publishFeedback(feedback_);
        
        currentstateid = REQUESTING;

 
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
	
		stopActivities();	

                printf("\nStatus: (%i) Idle, waiting New Request \n",currentstateid);

		printf("\nPrevious goal preempted!!\n\n");sleep(10);
                
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
	    info_map.clear();
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

		feedback_.status = 0; //started
		feedback_.internal_state = REQUESTING;
		feedback_.ready = false;
		feedback_.action = currentactivity.name;
		feedback_.command = currentactivity.rawname;
		setCommonFeedback();

                printf("\nActivity: %s\n",currentactivity.name.c_str());
                activity_succeeded = false;


		//Check which activity we should perform
                if (currentactivity.name == "move"){
                    //Send the goal to the navigation server
                    currentstateid = MOVING;

			setLocationNavigation();

			navigationClient->sendGoal(navigation_goal);

                }
                else if (currentactivity.name == "dotelepresence"){
                    	//Send the goal to the telepresence server

			//ROS_INFO("\nuser name:%s , skype user: %s\n", currentactivity.objects.at(0).c_str(),currentactivity.objects.at(1).c_str());

			std::string info_users = info_map.left.at(currentactivity.objects.at(0));

			std::vector<std::string> aux_info_users = split(info_users, ',');
			//printf("\nuser name: %s, skype user: %s\n", aux_info_users.at(0).c_str(), aux_info_users.at(1).c_str());

			string aux_name = info_map.left.at(currentactivity.objects.at(1));
			printf("name: %s\n", aux_name.c_str());

			telepresence_goal.user = aux_name.c_str();
			telepresence_goal.nameOfCaller = aux_info_users.at(0).c_str();
			telepresence_goal.skypeUser = aux_info_users.at(1).c_str();

			telepresenceClient->sendGoal(telepresence_goal);

			currentstateid = DOINGTELEPRESENCE;
		
                }
		else if (currentactivity.name == "update_status"){
			currentstateid = FINISHED;                    
		}
		else if (currentactivity.name == "remind"){
			interface_goal.activity = "default_tab";
			interface_goal.code = 2;
			interface_goal.text = "My reminder text is set up here";
			interface_goal.speech = "My reminder speech is set up here";

			robotGuiClient->sendGoal(interface_goal);

			currentstateid = FINISHED;

			
		}
                else {
                    printf("\nI don't know how to execute activity '%s' \n",currentactivity.name.c_str());
                    currentstateid = FINISHED;

                }

                previousstateid = REQUESTING;
                break;
            }


            case MOVING: //Status: Executing Acticity MOVE
            {
                //Print state
                if (previousstateid != MOVING){
                   printf("\nStatus: (%i) Executing activity MOVE\n",currentstateid);
                }

		//set up the feedback
		feedback_.status = 1; //in progress
		feedback_.internal_state = MOVING;
		feedback_.action = currentactivity.name;
		feedback_.command = currentactivity.rawname;
		feedback_.ready = false;
		setCommonFeedback();

                // Check if the acivity is done
                if (navigationClient->done){
                    currentstateid = FINISHED;
		    
		    //check if the activity succeeded
		    if (navigationClient->succeeded){
			activity_succeeded = true;
		    }
		    //info_map.clear();
                }

                previousstateid = MOVING;
                break;
            }

            case DOINGTELEPRESENCE: //Status: Executing Acticity DOTELEPRESENCE
            {
                //Print state
                if (previousstateid != DOINGTELEPRESENCE){
                   printf("\nStatus: (%i) Executing activity TELEPRESENCE\n",currentstateid);
                }
			//set up the feedback
			feedback_.status = 1; //in progress
			feedback_.internal_state = DOINGTELEPRESENCE;
			feedback_.action = currentactivity.name;
			feedback_.command = currentactivity.rawname;
			feedback_.ready = false;
			setCommonFeedback();

		        // Check if the acivity is done
		        if (telepresenceClient->done){
		            currentstateid = FINISHED;

 			    //check if the activity succeeded
			    if (telepresenceClient->succeeded){
			    	 activity_succeeded = true;
			    }
		        }

			//info_map.clear();
			previousstateid = DOINGTELEPRESENCE;
			break;
            	}


		case FINISHED: //Status: Activity Finished
		{
			//Print state
			if (previousstateid != FINISHED){
			    printf("\nStatus: (%i) Activity Finished \n",currentstateid);
			}

			if (activity_succeeded){
				feedback_.status = 2; //suceeded
				//set the result
				result_.statusCode = 1;
			}
			else{
				feedback_.status = 3; //failed
				//set the result
				result_.statusCode = 0;
			}
			feedback_.internal_state = FINISHED;
			feedback_.action = currentactivity.name;
			feedback_.command = currentactivity.rawname;
			feedback_.ready = true;
			setCommonFeedback();


			newrequest = false;
			activity_succeeded = false;
			success = true;
			finished = true;

			currentstateid = IDLE;
			info_map.clear();
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
		//get battery info
		feedback_.batterylevel = bat1_vol_; //TODO: set up as an array of battery levels
		//feedback_.batterylevel = bat2_vol_;
		feedback_.batterycapacity = 19.000; //TODO: set up as an array of battery capacities

		//get position info
		feedback_.x_position = current_x_position;
		feedback_.y_position = current_y_position;
	}



	/**
	  * This function gets the battery info
	  */
	void updatePowerStatus(const drrobot_h20_player::PowerInfo::ConstPtr& powerInfo){
		 
		 bat1_vol_ = powerInfo->bat1_vol;
		 bat2_vol_ = powerInfo->bat2_vol;
		 //bat1_temp_ = powerInfo->bat1_temp;
		 //bat2_temp_ = powerInfo->bat2_temp;
		 //dcin_vol_ = powerInfo->dcin_vol;
		 //ref_vol_ = powerInfo->ref_vol;
		
		 //printf("\nGot battery information\n");
	}



	/**
	  * This function gets the odom/location info
	  */	
	void updatePositionStatus(const nav_msgs::Odometry::ConstPtr& odomInfo){
		 
		 current_x_position = odomInfo->pose.pose.position.x;
		 current_y_position = odomInfo->pose.pose.position.y;	
		 //printf("\nGot odom information\n");
	}

	/**
	* This function creates the info map
	**/
	void create_infoMap(){

        	info_map.clear();
		std::vector<std::string> aux[Ngoal.info.size()];

		for(int i =0;i<Ngoal.info.size();i++){
			aux[i] = split(Ngoal.info.at(i), ':');
			//printf("\nidentifier : %s, info: %s\n", aux[i].at(0).c_str(), aux[i].at(1).c_str());
		}
	
		//creating the info map
		for(int i = 0; i<Ngoal.info.size(); i++){
			info_map.insert( info_map_value(aux[i].at(0).c_str(), aux[i].at(1).c_str()));
		}
    }

	/**
	* This function sets the navigation goal
	**/
	void setLocationNavigation(){
		ROS_INFO("location from:%s , location to: %s\n", currentactivity.objects.at(0).c_str(),currentactivity.objects.at(1).c_str());

		std::vector<std::string> info_location;
		for(int i = 0; i < Ngoal.info.size(); i++){
			//info_location[i] = info_map.left.at(currentactivity.objects.at(i));
			info_location.push_back(info_map.left.at(currentactivity.objects.at(i)));
			//printf("size = %d, %s\n", Ngoal.info.size(), info_location[i].c_str());
			//printf("size = %d, %s\n", Ngoal.info.size(), info_location.at(i).c_str());
		}
		//printf("l0: %s, l1: %s\n", info_location.at(0).c_str(), info_location.at(1).c_str());
		sleep(3);
		std::vector<std::string> auxString;
		std::vector<float> auxFloat[Ngoal.info.size()];

		for(int i = 0; i < Ngoal.info.size(); i++){
			//printf("i = %d and %s\n", i, info_location[i].c_str());
			auxString = split(info_location.at(i).c_str(), ',');

			//printf("\n splitting the string of coordinates \n");
			for(int j = 0; j < 7; j++){
				auxFloat[i].push_back((float) atof(auxString.at(j).c_str()));
				//printf("\nconverting %s to %.3f\n", auxString.at(j).c_str(), auxFloat[i].at(j));
				//printf("\nconverting %s\n", auxString.at(j).c_str());
			}

		}

		//vector< float > coordinates = map.left.at(currentactivity.objects.at(1));

		printf("\nPreparing the navigation_goal\n");
	///*
		navigation_goal.target_pose.header.frame_id = "/map";
		navigation_goal.target_pose.header.stamp = ros::Time::now();

		navigation_goal.target_pose.pose.position.x = auxFloat[1].at(0);
		navigation_goal.target_pose.pose.position.y = auxFloat[1].at(1);
		navigation_goal.target_pose.pose.position.z = auxFloat[1].at(2);
		navigation_goal.target_pose.pose.orientation.x = auxFloat[1].at(3);
		navigation_goal.target_pose.pose.orientation.y = auxFloat[1].at(4);
		navigation_goal.target_pose.pose.orientation.z = auxFloat[1].at(5);
		navigation_goal.target_pose.pose.orientation.w = auxFloat[1].at(6);
	//*/
		printf("\nThe navigation_goal is ready\n");
    }

      	/**
	* This function preempts the activities that are being executed
	**/
	void stopActivities(){
		if(currentstateid = MOVING){
			navigationClient->ac.cancelGoal();
			printf("\nMove activity preempted.\n");
		}
		else if(currentstateid = DOINGTELEPRESENCE){
			telepresenceClient->ac.cancelGoal();
			printf("\nTelepresence activity preempted.\n");
		}
		else
			printf("\nActivity preempted.\n");

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


