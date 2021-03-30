//============================================================================
// Name        : joke_handler.cpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : This program reads a file with a list of possible jokes, and then allows you
//               to pull random jokes from the list. It also keeps track of all previously used
//               small talk jokes in a session.
//               Get a random statement through the function get_rand_joke()
//============================================================================

#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Char.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include "joke_handler.h"

joke_handler::joke_handler()
{
}

joke_handler::~joke_handler()
{
}

/*Read the included text file (jokes.txt) and store all jokes in the variable list_of_jokes*/
void joke_handler::read_file(std::string filePath)
{
  std::ifstream jokesStream(filePath.c_str());

  if (jokesStream.is_open())
  {
    std::string line;
    std::string ques;
    std::string ans;
    
    while ( getline (jokesStream,line) )
    {
        if(!line.empty())
        {
              if(line.at(0)=='Q')
              {
                  ques=line.substr(2);
                  getline (jokesStream,line);
                  ans=line.substr(2);
                  store_joke(ques,ans);
              }
        }
    }
    jokesStream.close();
  }
}

/*Store joke in list*/
void joke_handler::store_joke(std::string ques, std::string ans)
{
  if(ques.empty()||ans.empty())
  {
    ROS_ERROR("Joke cannot be stored -- improper formatting of jokes.txt");
  }else{
    std::vector<std::string> joke;
    joke.push_back(ques);
    joke.push_back(ans);
    list_of_jokes.push_back(joke);
  }
}

/*Retrieve random joke which has not been used in the session already
  Once used, a joke is removed from the stored list_of_jokes*/
std::vector<std::string> joke_handler::get_rand_joke()
{
  if(!list_of_jokes.empty()){
      int num_jokes=list_of_jokes.size();
      ROS_INFO("%d jokes left", num_jokes);
      if(num_jokes!=1){
        
        int rand_n=rand() % (num_jokes);
        std::vector<std::string> joke=list_of_jokes.at(rand_n);
        list_of_jokes.erase(list_of_jokes.begin()+rand_n);
        
        return joke;
        
      }else if(num_jokes==1){
        std::vector<std::string> joke=list_of_jokes.at(0);
        list_of_jokes.erase(list_of_jokes.begin());
        return joke;
        
      }
  }else{
        ROS_WARN("No more jokes!");
        std::vector<std::string> empty_joke;
        empty_joke.push_back(" ");
        empty_joke.push_back(" ");
        
        return empty_joke;
  }

}

void joke_handler::delete_progress()
{
  list_of_jokes.clear();
}
