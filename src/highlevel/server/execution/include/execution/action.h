
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


using namespace std;


class Action
{

public:
    
    // attributes
    
    //plan related variables
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
    Action(){
    }
    
    Action(string actionstr){
        setActionInfo(actionstr);
    }       

    ~Action(){
    }
    
    /**
     * This function set the action info, including start time, duration,
     * operator's name, the agent and the objects involved.
     * This function expect actions in the following (PDDL) format:
     * <start_time>: (<operator> <agent> <list of objects>) [<duration>]
     * @param planstr
     */
    void setActionInfo(string actionstr){
        //printf("Action: %s\n",actionstr.c_str());

        //Set up raw name of the action.
        rawname = actionstr;

        //break down the raw action  string
        std::vector<std::string> actionvector = split(actionstr,' ');
        //An action has to have at least three elements: the start time (index), the operator name, and the duration.
        if (actionvector.size() >= 3){

            //1) START TIME / index (clean up the ':' if necessary)
            string starttimestr = actionvector.at(0);
            int found = starttimestr.find(":");
            if (found != std::string::npos){
                starttimestr.erase(starttimestr.begin()+found);
            }
            
            found = starttimestr.find(".");
	    //ignore everything after .0XXXXX
            //starttimestr.erase(found+2,strlen(starttimestr.c_str())); //Consider .0 time stamp
	    //igone everyting after .XXXXX (e.g., 120.01 = 120)
	    starttimestr.erase(found,strlen(starttimestr.c_str())); //Do not consider anything after '.' from the time stamp
            starttime = (int) boost::lexical_cast<float>(starttimestr); //Another option for converting string to float
            //starttime = atof(starttimestr.c_str());


            //2) OPERATOR'S NAME (clean up the '(' if necessary)
            name = actionvector.at(1);
            found = name.find("(");
            if (found != std::string::npos){
                name.erase(name.begin()+found);
            }


            //3) AGENT, OBJECTS and DURATION
            bool gotAgent = false;
            objects.clear();

            for(int i=2; i<actionvector.size(); ++i){
                string obj = trim(actionvector.at(i));
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
                    //get the agent and objects involved in the action
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
                        //get the objects involved in the action
                        else{
                            objects.push_back(obj);
                        }
                    }
                }
            }           
        }
               
    }
            
    
    /**
     * Print the action
     */
    void print(){

        printf("Action: %s\n",rawname.c_str());

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

        
	
        /*        
        //Print statistics 
        printf("\n");      
        printf("==========================================\n");        
        printf("Statistics:\n");        
        //Runtime
        //printf("- Runtime:            %s\n",runtime.c_str());
        //Number of actions
        int nact = listofActions.size();
        stringstream naction;//create a stringstream
        naction << nact;//add number to the stream
        printf("- Number of actions:  %s\n",naction.str().c_str());
        
        printf("\nPlan: \n");
        //Print the plan
        for(int i=0; i<listofActions.size(); ++i){
            vector<string> actionvec = listofActions.at(i);
            for(int j=0; j<actionvec.size(); ++j){
                cout<< actionvec.at(j) + ' ';
            }
            cout<< '\n';
        }
        printf("==========================================\n");
	*/ 
    }
    
    
    
    /**
     * Return a string of the action
     */
    string toString(){


        string actionstr = "";

        actionstr += rawname;

        /*
        //Print the plan
        for(int i=0; i<listofActions.size(); ++i){
            vector<string> actionvec = listofActions.at(i);
            for(int j=0; j<actionvec.size(); ++j){
                planstr += actionvec.at(j) + ' ';
            }
            planstr += '\n';
        }
        */
        
        return actionstr;
    }

    

};

