//============================================================================
// Name        : small_talk_handler.hpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Modular speech handler
//============================================================================

#ifndef small_talk_handler_H_
#define small_talk_handler_H_
#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Char.h>
#include <sstream>

class small_talk_handler {
  public:
  
  /*    Speech is stored in the vector list_of_lines.
    Each vector in list_of_lines has the first element being the speech and the second element
    being the text to be displayed on the screen.
    Retrieve a joke with get_next_line();
    Lines which have been already given in one session is removed from the stored list of lines*/
    
    ros::NodeHandle nh_;
    
    std::vector< std::string > list_of_lines;
    
    small_talk_handler();
    ~small_talk_handler();
    
    void delete_progress();
    void read_file(std::string filePath);
    void store_line(std::string speech);
    std::string get_rand_line();
};



#endif