#include "robot_gui/robot_gui_client.h"
#include <vector>
#include <string>
#include <ros/ros.h>
#include <string>
#include <stdio.h>
#include <std_msgs/Char.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iostream>
#include <tr1/functional>
#include <boost/algorithm/string.hpp>

vector<string> question0;
vector<string> question1;
// ros::NodeHandle nh_;
std::vector< std::vector<std::string> > list_of_ques;
std::vector< std::string> list_of_lines;

/*Store ques in list*/
void store_ques(std::string ques, std::string ans1, std::string ans2, std::string ans3, std::string corr_ans, std::string hint)
{
  if(ques.empty()||ans1.empty()||ans2.empty()||ans3.empty()||corr_ans.empty())
  {
    ROS_ERROR("Ques cannot be stored -- improper formatting of CATEGORY_NAME.txt");
  }else{
    std::vector<std::string> question;
    question.push_back(ques);
    question.push_back(ans1);
    question.push_back(ans2);
    question.push_back(ans3);
    question.push_back(corr_ans);
    question.push_back(hint);
    list_of_ques.push_back(question);
  }
}
void read_file(std::string filePath)
{
  std::ifstream quesStream(filePath.c_str());
  ROS_INFO("File path is: [%s]",filePath.c_str());

  if (quesStream.is_open())
  {
    std::string line;
    std::string ques;
    std::string ans1;
    std::string ans2;
    std::string ans3;
    std::string corr_ans;
    std::string hint;
    
    while ( getline (quesStream,line) )
    {
      // std::cout<<line<<std::endl;
      // std::cin.get();
        if(!line.empty())
        {
          list_of_lines.push_back(line);
/*
              if(line.at(0)=='Q')
              {
                  ques=line.substr(2);
                  getline (quesStream,line);
                  ans1=line.substr(3);
                  getline (quesStream,line);
                  ans2=line.substr(3);
                  getline (quesStream,line);
                  ans3=line.substr(3);
                  getline (quesStream,line);
                  corr_ans=line.substr(3);
                  getline (quesStream, line);
                  hint=line.substr(2);
                  store_ques(ques,ans1,ans2,ans3,corr_ans, hint);
              }*/
        }
    }
    quesStream.close();
  }
}
/*Retrieve random ques which has not been used in the session already
  Once used, a ques is removed from the stored list_of_ques*/
std::vector<std::string> get_ques()
{
  if(!list_of_ques.empty()){
      int num_ques=list_of_ques.size();
      ROS_INFO("%d ques left", num_ques);
      if(num_ques!=1){
        std::vector<std::string> ques=list_of_ques.at(0);
        list_of_ques.erase(list_of_ques.begin());
        
        return ques;
        
      }else if(num_ques==1){
        std::vector<std::string> ques=list_of_ques.at(0);
        list_of_ques.erase(list_of_ques.begin());
        return ques;
        
      }
  }else{
        ROS_WARN("No more ques!");
        std::vector<std::string> empty_ques;
        empty_ques.push_back(" ");
        empty_ques.push_back(" ");
        
        return empty_ques;
  }

}


void init(){
    // robotGuiClient("test_gui");
    //Store fake questions
    
    read_file("/home/tangy/tangy-robot/src/trivia/database/misc_text_new.txt");
    ROS_INFO("No. lines. = %d", int (list_of_lines.size()));
    ROS_INFO("No. ques. = %d", int (list_of_ques.size()));
}

/*Say and display on the screen the same string*/
robot_gui::Robot_guiGoal say_and_display(std::string activity, std::string str0, std::string str1, int subtab){
    robot_gui::Robot_guiGoal goal;
    goal.activity = activity;
    goal.code=2;
    goal.speech=str0;
    goal.text=str1;
    goal.subtab=subtab;
    return goal;
}

int main(int argc, char **argv) {
  
    ros::init(argc, argv, "test_gui");
    ros::NodeHandle nh;
    RobotGuiClient robotGuiClient("test gui");
    init();
    while(1){
          robot_gui::Robot_guiGoal go0=say_and_display("trivia","Which of the following animals sleep standing up?","Test\nA:Test \nB:Test \nC:Test ", 2 );
          robotGuiClient.sendGoalAndWait(go0);
          robot_gui::Robot_guiGoal go1=say_and_display("trivia","A: ","Test\nA:Test \nB:Test \nC:Test ", 2 );
          robotGuiClient.sendGoalAndWait(go1);   
          robot_gui::Robot_guiGoal go2=say_and_display("trivia","Elephants ","Test\nA:Test \nB:Test \nC:Test ", 2 );
          robotGuiClient.sendGoalAndWait(go2);   
          robot_gui::Robot_guiGoal go3=say_and_display("trivia","B: ","Test\nA:Test \nB:Test \nC:Test ", 2 );
          robotGuiClient.sendGoalAndWait(go3);
          robot_gui::Robot_guiGoal go4=say_and_display("trivia","Flamingos","Test\nA:Test \nB:Test \nC:Test ", 2 );
          robotGuiClient.sendGoalAndWait(go4);   
          robot_gui::Robot_guiGoal go5=say_and_display("trivia","C: ","Test\nA:Test \nB:Test \nC:Test ", 2 );
          robotGuiClient.sendGoalAndWait(go5);   
          robot_gui::Robot_guiGoal go6=say_and_display("trivia","Camels","Test\nA:Test \nB:Test \nC:Test ", 2 );
          robotGuiClient.sendGoalAndWait(go6);   
    }

    //Build hash mapping for questions to be used in transferring audio files from Brian computer to Tangy computer.
    std::ofstream ofs;
    std::tr1::hash<std::string> hash_fn;
    ofs.open("/home/tangy/tangy-robot/src/trivia/database/Hash_misc_text_new.txt");
    // int sz=list_of_ques.size();
    int sz=list_of_lines.size();
    int cnt=1;
    for(int i=0; i<sz; i++){
      ROS_INFO("i counter = %d",i);
/*     std::vector< std::string > qs = get_ques();
       std::string strFrag0=qs.at(0);
      std::string strFrag1=qs.at(1);
      std::string strFrag2=qs.at(2);
      std::string strFrag3=qs.at(3);
      std::string strFrag4=qs.at(5);*/
      std::string line=list_of_lines.at(i);
      boost::trim(line);
      size_t hsh=hash_fn(line);
/*      boost::trim(strFrag0);
      size_t hash0=hash_fn(strFrag0);
      boost::trim(strFrag1);
      size_t hash1=hash_fn(strFrag1);
      boost::trim(strFrag2);
      size_t hash2=hash_fn(strFrag2);
      boost::trim(strFrag3);
      size_t hash3=hash_fn(strFrag3);
      boost::trim(strFrag4);
      size_t hash4=hash_fn(strFrag4);
      
      
      std::stringstream ss;
      
      
      std::cout<<cnt << " [" <<strFrag0 << "] = " << hash0 <<  "\n" ;
      std::cout<<cnt+1 << " [" <<strFrag1 << "] = " << hash1 <<  "\n" ;
      std::cout<<cnt+2 << " [" <<strFrag2 << "] = " << hash2 <<  "\n" ;
      std::cout<<cnt+3 << " [" <<strFrag3 << "] = " << hash3 <<  "\n" ;
      std::cout<<cnt+4 << " [" <<strFrag4 << "] = " << hash4 <<  "\n";*/
      // ss << hsh << "\n";
      // ROS_INFO(ss.str().c_str());
      // ofs << hash0 << "\n" << hash1 <<"\n" << hash2 <<"\n" << hash3 << "\n" << hash4 << "\n";
      ofs<< hsh << "\n";
      
      cnt=cnt+5;

    }
    ROS_INFO("DIDN'T CRASH");
    ofs.close();
    return 0;
}