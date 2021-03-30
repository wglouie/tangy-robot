//============================================================================
// Name        : help_text_handler.cpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Handles reading a text file with help-related statements
//               Get a random statement with the functions:
//               get_encouragement(), get_praise()
//============================================================================

#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Char.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <trivia/help_text_handler.h>
  
help_text_handler::help_text_handler()
{
}

help_text_handler::~help_text_handler()
{
}

/*Read the included text file (help_texts.txt) and store all help_texts in the variable list_of_help_texts*/
void help_text_handler::read_file(std::string filePath)
{
  std::ifstream helpTextStream(filePath.c_str());
  if (helpTextStream.is_open())
  {
    std::string line;
    
    while ( getline (helpTextStream,line) )
    {
        if(!line.empty())
        {
              if(line.substr(0,4).compare("Enco")==0)
                  store_encouragement(line.substr(14));
                  
              if(line.substr(0,4).compare("Prai")==0)
                  store_praise(line.substr(7));
        }
    }
    helpTextStream.close();
  }
  
  ROS_INFO("Number of praise statements stored: %d", encouragement_statements.size());
  ROS_INFO("Number of encouragement statements stored: %d", praise_statements.size());
}

void help_text_handler::store_encouragement(std::string str)
{
  if(str.empty())
  {
    ROS_ERROR("Statement cannot be stored -- improper formatting of help_text.txt");
  }else{
    encouragement_statements.push_back(str);
  }
}

void help_text_handler::store_praise(std::string str)
{
  if(str.empty())
  {
    ROS_ERROR("Statement cannot be stored -- improper formatting of help_text.txt");
  }else{
    praise_statements.push_back(str);
  }
}

std::string help_text_handler::get_encouragement()
{
  if(!encouragement_statements.empty()){
      int num=encouragement_statements.size();

      if(num!=1){
        
        int rand_n=rand() % (num);
        return encouragement_statements.at(rand_n);
        
      }else if(num==1){
        return encouragement_statements.at(0);
      }
  }
}

std::string help_text_handler::get_praise()
{
  if(!praise_statements.empty()){
      int num=praise_statements.size();

      if(num!=1){
        
        int rand_n=rand() % (num);
        return praise_statements.at(rand_n);
        
      }else if(num==1){
        return praise_statements.at(0);
      }
  }

}

void help_text_handler::delete_progress()
{
    encouragement_statements.clear();
    praise_statements.clear();
}