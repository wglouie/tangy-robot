
/**
 * Author: Tiago Vaquero
 * Date: 2013
 * 
 */

#pragma once //avoid redefining the class when included somewhere else

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

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
#include "planner.h"


using namespace std;


class Plan
{

public:

    //ros::NodeHandle nh_;
    
    // attributes
    //planner
    string plannername;
    //stats
    string runtime;
    
    //plan related variables
    vector< vector<string> > listofActions; 
    
    bool isUnsolvable;
    bool solutionExists;

    
    /**
     * Constructors
     **/
    Plan(){
    }
    
    Plan(string planstr){
        isUnsolvable = false;
        solutionExists = true;        
        setplaninfo(planstr);
    }
    
    
    Plan(string output, Planner p){
        isUnsolvable = false;
        solutionExists = false;
        setPlanInfoFromPlannersOutput(output, p.solutionFoundMarker, p.problemUnsolvableMarker);
    }
    
    Plan(string output, string marker, string markerunsovable){
        isUnsolvable = false;
        solutionExists = false;
        setPlanInfoFromPlannersOutput(output, marker, markerunsovable);
    }

    ~Plan(){
    }
    
    /**
     * This function sets the list of actions and stats of the plan given in the string
     * @param planstr
     */
    void setplaninfo(string planstr){
        //printf("Plan:\n%s",planstr.c_str());
        
        listofActions.clear();

        std::istringstream f(planstr);
        std::string line;
        bool planstarted = false;
        bool planended = false;
        while (std::getline(f, line)) {
            std::string planline = trim(line);
            if (!planstarted){

                //identify stats
                int found = planline.find(";");
                if (!planline.empty() and found != std::string::npos){
                    
                    //Get the runtime
                    if (int foundtime = planline.find("Time") != std::string::npos){
                        planline.erase(0,foundtime+6);
                        //printf("--- Time: %s\n", planline.c_str());
                        runtime = planline;
                    }
                    
                    //TODO: Get other stats
                }
                else{                    
                    planstarted = true;
                    if (!planline.empty()){
                        //printf("+++ %s\n",planline.c_str());
                        listofActions.push_back(getactionvector(planline));
                    }                    
                }
            }
            else{
                if (!planline.empty()){
                    //printf("+++ %s\n",planline.c_str());
                    listofActions.push_back(getactionvector(planline));
                }else{
                    planended = true;
                    break;
                }
            }
        }
               
    }
    
    
   /**
    * This function reads the console output from the planner and gets the 
    * strings concerning the plan information only. If the problem is solvable
    * it will get the planner stats and the actions. If not, the plan/problem 
    * is marked as unsolvable. 
    * The marker identifies where the plan info starts, while the 
    * markerunsovable identifies whether the problem cannot be solved.
    * @param output
    * @param marker
    * @param markerunsovable
    * @return 
    */
   void setPlanInfoFromPlannersOutput(std::string output, std::string marker, std::string markerunsovable){
       std::string planstr = "";     

       //printf("%s\n",output.c_str());

       //Get the plan stats, actions and check if the problems is solvable
       std::istringstream f(output);
       std::string line;
       bool planstarted = false;
       bool planended = false;
       while (std::getline(f, line)) {
           std::string planline = trim(line);
           //printf("%s\n",planline.c_str());
           if (!planstarted){

               if (!planline.empty()){
                   //identify the marker to identify the beginning of the plan info
                   // which will be in the next line.
                   int found = planline.find(marker);
                   if (found != std::string::npos){
                       //printf("Got it: %s\n\n",planline.c_str());
                       planstarted = true;
                       solutionExists = true;
                   }
                   else{
                       int foundunsovable = planline.find(markerunsovable);
                       if (foundunsovable != std::string::npos){
                           //printf("Got it: %s\n\n",planline.c_str());
                           planstr = "";
                           isUnsolvable = true;
                           break;
                       }
                   }
               }

           }
           else{
               if (planline != ""){
                   planstr += planline + "\n";
               }else{
                   planended = true;
                   break;
               }
           }
       }    
       //printf("%s\n",planstr.c_str());
       
       if (solutionExists){
           setplaninfo(planstr);                      
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
        printf("- Runtime:            %s\n",runtime.c_str());
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
    }
    
    
    
    /**
     * Print the plan: the list of actions only
     */
    string toString(){
                
        string planstr = "";
        
        //Print the plan
        for(int i=0; i<listofActions.size(); ++i){
            vector<string> actionvec = listofActions.at(i);
            for(int j=0; j<actionvec.size(); ++j){
                planstr += actionvec.at(j) + ' ';
            }
            planstr += '\n';
        }
        
        return planstr;
    }
    
    
   
    
    
    
    

};

