//============================================================================
// Name        : help_text_handler.hpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Handles reading a text file with help-related statements
//               Get a random statement with the functions:
//               get_encouragement(), get_praise()
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
  
    ros::NodeHandle nh_;
    
    std::vector<std::string> encouragement_statements;
    std::vector<std::string> praise_statements;

    help_text_handler();
    ~help_text_handler();
    
    void delete_progress();
    void read_file(std::string filePath);

    void store_encouragement(std::string str);
    void store_praise(std::string str);

    std::string get_encouragement();
    std::string get_praise();

};



#endif