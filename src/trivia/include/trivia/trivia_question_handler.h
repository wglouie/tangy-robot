//============================================================================
// Name        : trivia_question_handler.hpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Trivia question handler
//============================================================================

#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <std_msgs/Char.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iostream>
#include <vector>
#include <boost/algorithm/string.hpp>

class trivia_question_handler {
  public:
  
    ros::NodeHandle nh_;
    
    std::vector< std::vector<std::string> > list_of_ques;
    
    trivia_question_handler();
    ~trivia_question_handler();
    
    void delete_progress();
    void read_file(std::string filePath);
    void store_ques(std::string ques, std::string ans1, std::string ans2, std::string ans3, std::string corr_ans, std::string hint);
    std::vector<std::string> get_ques();
    int get_num_ques();
};

