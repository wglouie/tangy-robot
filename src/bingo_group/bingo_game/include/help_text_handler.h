//============================================================================
// Name        : help_text_handler.hpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Handles reading a text file with help-related statements
//               Get a random statement with the functions:
//               get_encouragement(), get_praise(), get_wrongNumberHelp(), get_MissingNumberHelp()
//============================================================================

#ifndef help_text_handler_H_
#define help_text_handler_H_
#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Char.h>
#include <sstream>

class help_text_handler {
  public:
  
  /*    helps are stored in the vector list_of_helps.
    Each help is a vector with the first element being the ques and the second element being the ans.
    Retrieve a help with get_rand_help();
    helps which have been already told in one session is removed from the stored list of helps*/
    
    ros::NodeHandle nh_;
    
    std::vector<std::string> initial_statements;
    std::vector<std::string> encouragement_statements;
    std::vector<std::string> praise_statements;
    std::vector<std::string> wrongNumberHelp_statements;
    std::vector<std::string> missingNumberHelp_statements;
    
    help_text_handler();
    ~help_text_handler();
    
    void delete_progress();
    void read_file(std::string filePath);
    void store_initialHelp(std::string str);
    void store_encouragement(std::string str);
    void store_praise(std::string str);
    void store_wrongNumberHelp(std::string str);
    void store_missingNumberHelp(std::string str);
    std::string get_initialHelp();
    std::string get_encouragement();
    std::string get_praise();
    std::string get_wrongNumberHelp();
    std::string get_missingNumberHelp();
};



#endif