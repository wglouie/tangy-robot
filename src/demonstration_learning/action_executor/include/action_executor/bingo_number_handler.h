//============================================================================
// Name        : bingo_number_handler.hpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Modular number caller handler
//============================================================================

#ifndef bingo_number_handler_H_
#define bingo_number_handler_H_
#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Char.h>
#include <sstream>


/*    Picks a number from the currently available numbers
        Saves the current progress of the called numbers*/
class bingo_number_handler {
  public:
  
    std::string filePath;
    
    ros::NodeHandle nh_;
    
    std::vector< std::string > all_numbers;
    std::vector< std::string > list_of_numbers_left;
    
    bingo_number_handler();
    ~bingo_number_handler();
    
    void delete_progress(std::string path);
    void read_file(std::string filePath);
    void write_file(std::string curr_num);
    void store_num(std::string num);
    std::string get_rand_num();
    std::vector<int> get_previously_called_numbers();
};



#endif
