//============================================================================
// Name        : bingo_number_handler.cpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : This program generates all possible bingo numbers, and then allows you to pull random
//               bingo numbers. It also keeps track of all previously used numbers in a session by
//               writing them to the file 'progress.txt'.
//               Get a random number through the function get_rand_num()
//============================================================================

#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Char.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <action_executor/bingo_number_handler.h>



using namespace std;
  
bingo_number_handler::bingo_number_handler()
{
    for(int i =1; i<=15; i++){
      stringstream ss;
      ss<<"B-"<<i;
      all_numbers.push_back(ss.str());
    }
    for(int i=16; i<=30; i++){
      stringstream ss;
      ss<<"I-"<<i;
      all_numbers.push_back(ss.str());
    }
    for(int i=31; i<=45; i++){
      stringstream ss;
      ss<<"N-"<<i;
      all_numbers.push_back(ss.str());
    }
    for(int i=46; i<=60; i++){
      stringstream ss;
      ss<<"G-"<<i;
      all_numbers.push_back(ss.str());
    }
    for(int i=61; i<=75; i++){
      stringstream ss;
      ss<<"O-"<<i;
      all_numbers.push_back(ss.str());
    }
    
}

bingo_number_handler::~bingo_number_handler()
{
}

/*Read the included text file (lines.txt) and store all lines in the variable list_of_numbers*/
void bingo_number_handler::read_file(string path)
{
  list_of_numbers_left=all_numbers;
  filePath=path;
  
  ifstream readNumberStream(path.c_str());


  if (readNumberStream.is_open())
  {
    string l;

    while ( getline (readNumberStream,l) )
    {
        if(!l.empty())
        {
            for (int i=0; i<list_of_numbers_left.size(); i++){
                if(l.compare(list_of_numbers_left.at(i))==0){
                  list_of_numbers_left.erase(list_of_numbers_left.begin()+i);
                }
            }
        }
    }
    readNumberStream.close();
  }
}

/*Write a number to the text file of called numbers*/
void bingo_number_handler::write_file(string curr_num)
{
    ofstream writeNumberStream;
    writeNumberStream.open(filePath.c_str(), ios::in | ios::app);
    writeNumberStream<<"\n"<<curr_num;
    writeNumberStream.close();
}

/*Store line in list*/
void bingo_number_handler::store_num(string num)
{
    if(num.empty())
    {
      ROS_ERROR("line cannot be stored -- improper formatting of progress.txt");
    }else{
      for(int i=0; i<list_of_numbers_left.size(); i++){
          if(num.compare(list_of_numbers_left.at(i))==0){
            list_of_numbers_left.erase(list_of_numbers_left.begin()+i);
            write_file(num);
          }
      }
    }
}

/*Retrieve random number which has not been used in the session already
  Once used, a number is removed from the stored list_of_numbers.
  If all numbers have been used, returns an empty string*/
string bingo_number_handler::get_rand_num()
{
    if(!list_of_numbers_left.empty()){
        ROS_INFO("%d numbers left.", list_of_numbers_left.size());
        string num;
        if(list_of_numbers_left.size()!=1){
          int rand_n=rand() % (list_of_numbers_left.size()-1);
          num=list_of_numbers_left.at(rand_n);
          store_num(num);
        }else if(list_of_numbers_left.size()==1){
          num=list_of_numbers_left.at(0);
          store_num(num);
        }
        
        return num;
  
    }else{
        ROS_WARN("No more numbers!");
        string empty_line="";
  
        return empty_line;
    }

}

vector<int> bingo_number_handler::get_previously_called_numbers()
{
  vector<int> result;
  if(filePath.size()!=0){
    ifstream readNumberStream(filePath.c_str());
  
    if (readNumberStream.is_open())
    {
      string l;
  
      while ( getline (readNumberStream,l) )
      {
          if(l.size()!=0){
              int num=atoi(l.substr(2).c_str());
              result.push_back(num);
              
          }

      }
      readNumberStream.close();
    }
  }
  return result;
  

}

void bingo_number_handler::delete_progress(std::string path)
{
    //Delete file contents
    std::ofstream ofs;
    ofs.open(path.c_str(), std::ofstream::out | std::ofstream::trunc);
    ofs.close();
    list_of_numbers_left.clear();
}
