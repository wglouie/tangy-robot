//============================================================================
// Name        : joke_handler.hpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Modular joke handler
//============================================================================

#ifndef joke_handler_H_
#define joke_handler_H_
#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Char.h>
#include <sstream>

class joke_handler {
  
  public:
  
  /*    Jokes are stored in the vector list_of_jokes.
    Each joke is a vector with the first element being the ques and the second element being the ans.
    Retrieve a joke with get_rand_joke();
    Jokes which have been already told in one session is removed from the stored list of jokes*/
    
    ros::NodeHandle nh_;
    
    std::vector< std::vector<std::string> > list_of_jokes;
    
    joke_handler();
    ~joke_handler();
    
    void delete_progress();
    void read_file(std::string filePath);
    void store_joke(std::string ques, std::string ans);
    std::vector<std::string> get_rand_joke();
};



#endif