
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

#include <boost/thread.hpp>
#include <boost/date_time.hpp>


using namespace std;


class Planner
{

public:
   
    // attributes
    //planner
    string name;
    string path;
    
    string optionsPrior;
    string optionsPost;
    string domainArgument;
    string problemArgument;

    string solutionFoundMarker;
    string problemUnsolvableMarker;

    bool plannerResponded;
    string output;
 
        
    /**
     * Constructors
     **/
    Planner(){
    }
    
    Planner(string pname){
        name = pname;
        reset();
    }    
    
    /**
     * Destructor
     */
    ~Planner(){
    }
        
    /**
     * Print the planner's name
     */
    void print(){                   
        printf("Planner: %s\n",name.c_str());
        printf("Path: %s\n",path.c_str());
    }

    
    /**
     * Reset the planner's variables such as the output.
     */
    void reset(){                   
         plannerResponded = false;
         output = "";       
    }


    /**
     * Creates a thread that executes the external planner
     */
    void run(){   	
    	reset();
        cout << " >> Creating a thread for running the planner\n" << endl;                    
        //boost::thread plannerThread(boost::bind( &Planner::execute, this ));
        plannerThread = new boost::thread(boost::bind( &Planner::execute, this ));
       
    }
    
    /**
     * Stops the planner if it is running.
     * It kills the process.
     */
    void stop(){
    	
        //plannerThread->interrupt();

        //In case it does not kill the process (optic-clp for example)
        string checkprocess = "pidof -x "+name+" > /dev/null";
        //if(0 == system("pidof -x colin-clp > /dev/null")) {
        if(0 == system(checkprocess.c_str())) {
              //A process having name PROCESS is running.
            std::cout << "## Planner '"+name+"' is RUNNING" << std::endl;
            kill(getProcIdByName(name),SIGUSR1);
            std::cout << "## Planner '"+name+"' was stopped" << std::endl;

         }
        //else if(1 == system("pidof -x colin-clp > /dev/null")) {
            //	//A process having name PROCESS is NOT running.
        //	std::cout << "## NOT RUNNING '"+name+"'" << std::endl;
        //}
        else{

            std::cout << "## Planner '"+name+"' was not running when I tried to stop it." << std::endl;
        }
    	

    	reset();
    }
    

    /**
     * This function calls the planner
     */
    void execute()
    {
        printf(" >> Calling planner: %s\n", name.c_str());
        printcurrenttime();
        

        //Building up the command line
        string command = setCommandLine();
        
        //print command
        printf("Command line: %s", command.c_str());
        
        //sleep(4);
        
        output = exec((char*)command.c_str());
        //system("/home/tiago/Dropbox/Postdoc/NSERC-Engage/Models/TelepresenceDomain/pddl/colin-clp /home/tiago/Dropbox/Postdoc/NSERC-Engage/Models/TelepresenceDomain/pddl/v14mod2/domainv14mod2.pddl /home/tiago/Dropbox/Postdoc/NSERC-Engage/Models/TelepresenceDomain/pddl/v14mod2/problem14-1.pddl");

        //printf("%s", output.c_str());

        plannerResponded = true;
        printf("\n >> Planning finished\n");
        printcurrenttime();
    }



    /**
     * Building up the command line
     */
    string setCommandLine(){

        string command = path + "/" + name + " ";

        //options prior
        if (optionsPrior != ""){
            command += optionsPrior + " ";
            printf("Got options prior\n");
        }

        //domain file
        if (domainArgument != ""){
            command += domainArgument + " ";
            printf("Got domain arg\n");
        }
        command += path +"/domain.pddl ";

        //problem file
        if (problemArgument != ""){
            command += problemArgument + " ";
            printf("Got problem arg\n");
        }
        //command += path + "/problem14-10.pddl";
        command += path + "/problem.pddl";

        //options post
        if (optionsPost != ""){
            command += " " + optionsPost;
            printf("Got options post\n");
        }


        //Metric-FF Usage: ./metric-ff -p <folder> -o domain.pddl -f problem.pddl
        //std::string plannercommand = plannerpath + "/metric-ff -p " + plannerpath +"/ -o domain.pddl -f problem.pddl";
        //std::string plannercommand = plannerpath + "/" +planner+ " -p " + plannerpath +"/ -o domain.pddl -f problem.pddl";

        //OPTIC Usage: ./optic-clp [OPTIONS] domainfile problemfile [planfile, if -r specified]
        //std::string command = path + "/" +name+ " -N " + planner.path +"/domain.pddl "+ planner.path +"/problem.pddl";
        //std::string command = path + "/optic-clp -N " + path + "/domain.pddl " + path +"/problem14-10.pddl";

        //COLIN Usage: ./colin-clp [OPTIONS] domainfile problemfile [planfile, if -r specified]
        //string command = path + "/" + name+ " " + path +"/domain.pddl "+ path +"/problem14-10.pddl";
        //string command = path + "/optic-clp " + path +"/domain.pddl "+ path +"/problem.pddl";

        //GENERAl
        //string command = path + "/" + name + " " +optionsPrior+ " "+ path +"/domain.pddl "+ path +"/problem14-10.pddl";

        return command;
    }


private:
    boost::thread *plannerThread;

};

