//============================================================================
// Name        : small_talk_handler.cpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : This program reads a file with a list of possible small talk, and then allows you
//               to pull random statements from the list. It also keeps track of all previously used
//               small talk statements in a session.
//               Get a random statement through the function get_rand_line()
//============================================================================

#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Char.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include "small_talk_handler.h"


  
small_talk_handler::small_talk_handler()
{
}

small_talk_handler::~small_talk_handler()
{
}

/*Read the included text file (lines.txt) and store all lines in the variable list_of_lines*/
void small_talk_handler::read_file(std::string filePath)
{

  std::ifstream smallTalkStream(filePath.c_str());


  if (smallTalkStream.is_open())
  {
    std::string l;
    
    while ( getline (smallTalkStream,l) )
    {
        if(!l.empty())
        {
            store_line(l);
        }
    }
    smallTalkStream.close();
  }
}

/*Store line in list*/
void small_talk_handler::store_line(std::string speech)
{
  if(speech.empty())
  {
    ROS_ERROR("line cannot be stored -- improper formatting of smallTalk.txt");
  }else{
    list_of_lines.push_back(speech);
  }
}

/*Retrieve random line which has not been used in the session already
  Once used, a line is removed from the stored list_of_lines
  Returns an empty string if all previous lines have been used.*/
std::string small_talk_handler::get_rand_line()
{
  if(!list_of_lines.empty()){
      ROS_INFO("%d small talk phrases left.", list_of_lines.size());
      int rand_n=rand() % (list_of_lines.size());
      std::string line=list_of_lines.at(rand_n);
      list_of_lines.erase(list_of_lines.begin()+rand_n);
      
      return line;

  }else{
      ROS_WARN("No more lines!");
      std::string empty_line="";

      return empty_line;
  }

}

/*Delete progress*/
void small_talk_handler::delete_progress()
{
  list_of_lines.clear();
}
