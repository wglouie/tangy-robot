//============================================================================
// Name        : outro_speech_handler.cpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : This program reads a file with an outro speech script.
//               Get each of the lines in the script through the function get_next_line()
//============================================================================

#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Char.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <trivia/outro_speech_handler.h>


  
outro_speech_handler::outro_speech_handler()
{
}

outro_speech_handler::~outro_speech_handler()
{
}

/*Read the included text file (lines.txt) and store all lines in the variable list_of_lines*/
void outro_speech_handler::read_file(std::string filePath)
{

  std::ifstream outroStream(filePath.c_str());


  if (outroStream.is_open())
  {
    std::string l;
    std::string speech;
    std::string text;
    
    while ( getline (outroStream,l) )
    {
        if(!l.empty())
        {
              if(l.at(0)=='S')
              {
                  speech=l.substr(7);
                  getline (outroStream,l);
                  text=l.substr(5);
                  store_line(speech,text);
              }
        }
    }
    outroStream.close();
  }
}

/*Store line in list*/
void outro_speech_handler::store_line(std::string speech, std::string text)
{
  if(speech.empty()||text.empty())
  {
    ROS_ERROR("line cannot be stored -- improper formatting of outro.txt");
  }else{
    std::vector<std::string> line;
    line.push_back(speech);
    line.push_back(text);
    list_of_lines.push_back(line);
  }
}

/*Retrieve random line which has not been used in the session already
  Once used, a line is removed from the stored list_of_lines*/
std::vector<std::string> outro_speech_handler::get_next_line()
{
  if(!list_of_lines.empty()){
      int num_lines=list_of_lines.size();
      ROS_INFO("%d outro lines left", num_lines);
      
      std::vector<std::string> line=list_of_lines.front();
      list_of_lines.erase(list_of_lines.begin());
      
      return line;

  }else{
        ROS_WARN("No more lines!");
        std::vector<std::string> empty_line;

        return empty_line;
  }

}

void outro_speech_handler::delete_progress()
{
  list_of_lines.clear();
  
}