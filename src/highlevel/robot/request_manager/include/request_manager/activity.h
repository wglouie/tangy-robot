
/**
 * Author: Tiago Vaquero
 * Date: 2013
 * 
 */

#pragma once //avoid redefining the class when included somewhere else

#include <ros/ros.h>
//#include <actionlib/server/simple_action_server.h>

#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <cstring>
#include <fstream>
#include <unistd.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include "mylib.h"


using namespace std;


class Activity
{

public:
    
    // attributes
    
    //Activity related variables
    string rawname;
    string name;
    string agent;
    float starttime;
    float duration;

    //plan related variables
    vector< string > objects;

    
    
    /**
     * Constructors
     **/
    Activity(){
    }
    
    Activity(string activitystr){
        setActivityInfo(activitystr);
    }       

    ~Activity(){
    }
    
    /**
     * This function set the activity info, including start time, duration,
     * operator's name, the agent and the objects involved.
     * This function expect activitys in the following (PDDL) format:
     * <start_time>: (<operator> <agent> <list of objects>) [<duration>]
     * @param planstr
     */
    void setActivityInfo(string activitystr){
        printf("Activity: %s\n",activitystr.c_str());
        reset();

        //Set up raw name of the activity.
        rawname = activitystr;

		//put everuthing to lower case
		boost::algorithm::to_lower(activitystr); 


        //break down the raw activity string
        std::vector<std::string> activityvector = split(activitystr,' ');
        //if the activity has at least three elements: the start time (index), the operator name, and the duration.
        // This is the PDDL format
        if (activityvector.size() >= 3){

            //1) START TIME / index (clean up the ':' if necessary)
            string starttimestr = activityvector.at(0);
            int found = starttimestr.find(":");
            if (found != std::string::npos){
                starttimestr.erase(starttimestr.begin()+found);
            }
            //ignore everything after .0XXXXX
            found = starttimestr.find(".");
            starttimestr.erase(found+2,strlen(starttimestr.c_str()));
            starttime = boost::lexical_cast<float>(starttimestr); //Another option for converting string to float
            //starttime = atof(starttimestr.c_str());


            //2) OPERATOR'S NAME (clean up the '(' if necessary)
            name = activityvector.at(1);
            found = name.find("(");
            if (found != std::string::npos){
                name.erase(name.begin()+found);
            }            


            //3) AGENT, OBJECTS and DURATION
            bool gotAgent = false;
            objects.clear();

            for(int i=2; i<activityvector.size(); ++i){
                string obj = trim(activityvector.at(i));
                //check if the string is not empty
                if (!obj.empty()){
                    //get the duration
                    int foundbegin = obj.find("[");
                    if (foundbegin != std::string::npos){
                        obj.erase(obj.begin()+foundbegin);
                        int foundend = obj.find("]");
                        obj.erase(obj.begin()+foundend);
                        duration = boost::lexical_cast<float>(obj);

                    }
                    //get the agent and objects involved in the activity
                    else{
                        //clean up any ')' if necessary
                        found = obj.find(")");
                        if (found != std::string::npos){
                            obj.erase(obj.begin()+found);
                        }
                        //get agent if we haven't got it yet
                        if(!gotAgent){
                            agent = obj;
                            gotAgent = true;

                        }
                        //get the objects involved in the activity
                        else{
                            objects.push_back(obj);
                        }
                    }
                }
            }           
        }
        //in case we it is just a action label we will just set up the name.
        // An example would be "update_status" or "shoutshown", "stop", etc. 
        // These are lower-level/control/monitoring commands,
        else if (activityvector.size() == 1){        	
        	name = activitystr;        	
        }
               
    }

 

   /**
     * Reset and clear activity variables
     */
    void reset(){
		rawname = "";
		name = "";
    	agent = "";
    	starttime = -1;
    	duration = -1;
    	objects.clear();
    }
            
    
    /**
     * Print the activity
     */
    void print(){

        printf("Activity: %s\n",rawname.c_str());

        printf(" - Start time: %f\n",starttime);
        printf(" - End time: %f\n",starttime+duration);
        printf(" - Duration: %f\n",duration);
        printf(" - Name: %s\n",name.c_str());
        printf(" - Agent: %s\n",agent.c_str());
        printf(" - Objects: ");
        for(int i=0; i<objects.size(); ++i){
            string o = objects.at(i);
            printf("[%s] ",o.c_str());
        }
        printf("\n");
   
    }
    
    
    
    /**
     * Return a string of the action
     */
    string toString(){


        string actionstr = "";

        actionstr += rawname;
        
        return actionstr;
    }

    

};

