
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
#include <stdio.h>
#include <sstream>
#include <vector>
#include <cstring>
#include <fstream>
//#include <algorithm>
//#include <functional>
//#include <cctype>
//#include <locale>
#include <unistd.h>
#include "action.h"

using namespace std;


class ExecutionPlan
{

public:
    
    // attributes
    
    //plan related variables
    //vector< vector<string> > listofActions;
    vector< Action > actions;


    int index; //this variable stores the index of the next action (or set of actions) to be executed
    
    
    /**
     * Constructors
     **/
    ExecutionPlan(){
    }
    
    ExecutionPlan(string planstr){     
        setActions(planstr);
    }       

    ~ExecutionPlan(){
    }
    
    /**
     * This function sets the list of actions from a plan string
     * @param planstr
     */
    void setActions(string planstr){
        //printf("Plan:\n%s",planstr.c_str());
        
        //listofActions.clear();
        actions.clear();
        index = 0;

        std::istringstream f(planstr);
        std::string line;
        while (std::getline(f, line)) {		
            std::string planline = trim(line);
            if (!planline.empty()){
                //printf("+++ %s\n",planline.c_str());
                //listofActions.push_back(getactionvector(planline));
                actions.push_back(Action(planline));
            }
        }
               
    }

    
    /**
     * Print the plan, including the statistics and the actions
     */
    void print(){
                        
        //Print statistics 
        printf("\n");      
        printf("==========================================\n");        
        printf("Statistics:\n");        
        //Runtime
        //printf("- Runtime:            %s\n",runtime.c_str());
        //Number of actions
        //int nact = listofActions.size();
        int nact = actions.size();
        stringstream naction;//create a stringstream
        naction << nact;//add number to the stream
        printf("- Number of actions:  %s\n",naction.str().c_str());
        
        printf("\nPlan: \n");
        //Print the plan
        for(int i=0; i<actions.size(); ++i){
            Action a = actions.at(i);
            a.print();
            //vector<string> actionvec = listofActions.at(i);
            //for(int j=0; j<actionvec.size(); ++j){
            //    cout<< actionvec.at(j) + ' ';
            //}
            cout<< '\n';
        }
        printf("==========================================\n"); 
    }
    
        
    /**
     * Print the list of actions in the plan
     */
    string toString(){
                
        string planstr = "";
        
        //Print the plan
        for(int i=0; i<actions.size(); ++i){
            Action a = actions.at(i);
            a.print();
            //vector<string> actionvec = listofActions.at(i);
            //for(int j=0; j<actionvec.size(); ++j){
            //    planstr += actionvec.at(j) + ' ';
            //}
            planstr += a.toString() + '\n';
        }
        
        return planstr;
    }
    

    /**
      * Return the size of the plan: the number of actions in the plan
      */
    int size(){
        int s = actions.size();
        return s;
    }


    /**
      * This function gets the start time of the next action(s) to be executed
      */
    float getNextActionsStartTime(){
        float nextstarttime = -1;

        if (actions.size() > index){
            Action a = actions.at(index);
	    //nextstarttime = a.starttime;
	    //force number as int //TODO: should we handle the time stamp differently?
	    int st = (int) a.starttime;
            nextstarttime = st;
        }
        return nextstarttime;
    }


    /**
      * This function gets next action(s) to be executed
      * It uses the index variable to get the next actions.
      */
    vector< Action > getNextActions(){
        vector< Action > nextactions;
        nextactions.clear();

        float nextstarttime = getNextActionsStartTime();

        for(int i=index; i<actions.size(); ++i){
            Action a = actions.at(i);
            if (a.starttime == nextstarttime){
               nextactions.push_back(a);
            }
            else if (a.starttime > nextstarttime){
                break;
            }
        }
        return nextactions;
    }


    /**
      * This function gets all the actions that start at a particular time point
      * @param startpoint
      */
    vector< Action > getActionsStartingAt(float startpoint){

        vector< Action > selectedactions;
        selectedactions.clear();

        for(int i=0; i<actions.size(); ++i){
            Action a = actions.at(i);
            if (a.starttime == startpoint){
               selectedactions.push_back(a);
            }
            else if (a.starttime > startpoint){
                break;
            }
        }
        return selectedactions;
    }


    /**
      * Check whether the plan still has actions to be executed
      */
    bool hasActionsToExecute(){
        bool result = false;
        if (index < actions.size()){
            result = true;
        }

        return result;
    }

    

};

