#include <trivia/trivia.h>

//Using the system() command liberally, which is dangerous
//Ideally should use the X library, which the xinput utility is
//based on. Leaving as a TODO for the future

void triviaGameServer::calibrate_keyboards(){
  //TODO: COMPLETE THIS FUNCTION
  //sets device ids to each team in order of the calibration method
  //disable devices
  
  teams.push_back(0);
  teams.push_back(1);
  
  vector<float> location0;
  vector<float> location1;
/*  location0.push_back(0);
  location0.push_back(0);
  location1.push_back(0);
  location1.push_back(0);*/
  
  location0.push_back(1.2);
  location0.push_back(-0.4);
  
  location1.push_back(1.2);
  location1.push_back(0.4);
  
  locations.push_back(location0);
  locations.push_back(location1);
  
  for(int i=0; i<locations.size(); i++){
    scores.push_back(i);
  }
  
  LEFT=atan(double (location0.at(1))/(double (location0.at(0)))) * 180 / 3.141;
  RIGHT=atan(double (location1.at(1))/(double (location1.at(0)))) * 180 / 3.141;
}

//Returns the team number, or -1 if no teams respond
int triviaGameServer::wait_for_response(){
  int response=-1;
  //Open Devices
  if ((dev0 = open (evID0.c_str(), O_RDONLY|O_NONBLOCK)) == -1){
          printf ("not a valid device.\n");
          return -1;
  }
  if ((dev1 = open (evID1.c_str(), O_RDONLY|O_NONBLOCK)) == -1){
          printf ("not a valid device.\n");
          return -1;
  }
  
  double start =ros::Time::now().toSec();
  double curr_time=ros::Time::now().toSec();
  ROS_INFO("Waiting for keyboard input.");
  while ((curr_time-start)<30 && response==-1){
  
          memset((void*)&ev0, 0, sizeof(ev0));
          memset((void*)&ev1, 0, sizeof(ev1));
  
          rd0 = read (dev0, (void*)&ev0, sizeof(ev0));
          rd1 = read (dev1, (void*)&ev1, sizeof(ev1));
  
          if(rd0>0 && ev0.value==1){
                 if(ev0.code==KEY_CODE_RESPONSE){
                   response=0;
                   tangy.beep();
                   tangy.moveNeck(RIGHT,-2);
                    tangy.nod();
                 }else if(ev0.code==KEY_CODE_HELP){
                   response=2;
                   tangy.beep();
                   tangy.moveNeck(RIGHT,-2);
                   tangy.nod();
                 }
                 
                 ROS_INFO("RESPONSE GIVEN BY USER IS [%d]/KEYBOARD CODE is [%d]", response, ev0.code);
          }
          if(rd1>0 && ev1.value==1){
                 if(ev1.code==KEY_CODE_RESPONSE){
                   response=1;
                   tangy.beep();
                   tangy.moveNeck(LEFT,-2);
                    tangy.nod();
                 }else if(ev1.code==KEY_CODE_HELP){
                   response=2;
                   tangy.beep();
                   tangy.moveNeck(LEFT,-2);
                    tangy.nod();
                 }
                 ROS_INFO("RESPONSE GIVEN BY USER IS [%d]/KEYBOARD CODE is [%d]", response, ev1.code);
                 
          }
          
          curr_time=ros::Time::now().toSec();
  }
  ROS_INFO("Input was %d", response);
  
  close(dev0);
  close(dev1);
  return response;
}

std::vector<std::string> triviaGameServer::wait_for_answer(){
  string ans="";
  string team="";
  vector<string> answers;
  //Open Devices
  if ((dev0 = open (evID0.c_str(), O_RDONLY|O_NONBLOCK)) == -1){
          ROS_ERROR("NOT A VALID DEVICE - KEYBOARD 0 INPUT WON'T WORK");
	  vector<string> invalid_ans;
	  invalid_ans.push_back(ans);
	  invalid_ans.push_back(team);
          return invalid_ans;
  }
  if ((dev1 = open (evID1.c_str(), O_RDONLY|O_NONBLOCK)) == -1){
          ROS_ERROR("NOT A VALID DEVICE - KEYBOARD 1 INPUT WON'T WORK");
	  vector<string> invalid_ans;
	  invalid_ans.push_back(ans);
	  invalid_ans.push_back(team);
          return invalid_ans;
  }
  
  double start =ros::Time::now().toSec();
  double curr_time=ros::Time::now().toSec();
  
  while ((curr_time-start)<30 && ans.size()==0){
  
          memset((void*)&ev0, 0, sizeof(ev0));
          memset((void*)&ev1, 0, sizeof(ev1));
  
          rd0 = read (dev0, (void*)&ev0, sizeof(ev0));
          rd1 = read (dev1, (void*)&ev1, sizeof(ev1));
  
          if(rd0>0 && ev0.value==1){
                 if(ev0.code==KEY_CODE_A){
                   ans="A1";
                   team="0";
                   tangy.beep();
                 }else if(ev0.code==KEY_CODE_B){
                   ans="A2";
                   team="0";
                   tangy.beep();
                 }else if(ev0.code==KEY_CODE_C){
                   ans="A3";
                   team="0";
                   tangy.beep();
                 }else if(ev0.code==KEY_CODE_HELP){
                   ans="help";
                   team="2";
                   tangy.beep();
                 }
                 ROS_INFO("RESPONSE GIVEN BY USER [%s] IS [%s]/KEYBOARD CODE is [%d]", team.c_str(), ans.c_str(), ev0.code);
          }
          if(rd1>0 && ev1.value==1){
                 if(ev1.code==KEY_CODE_A){
                   ans="A1";
                   team="1";
                   tangy.beep();
                 }else if(ev1.code==KEY_CODE_B){
                   ans="A2";
                   team="1";
                   tangy.beep();
                 }else if(ev1.code==KEY_CODE_C){
                   ans="A3";
                   team="1";
                   tangy.beep();
                 }else if(ev1.code==KEY_CODE_HELP){
                   ans="help";
                   team="2";
                   tangy.beep();
                 }
                 ROS_INFO("RESPONSE GIVEN BY USER [%s] IS [%s]/KEYBOARD CODE is [%d]",team.c_str(), ans.c_str(), ev1.code);
          }
          
          curr_time=ros::Time::now().toSec();
  }
  
  answers.push_back(ans);
  answers.push_back(team);
  close(dev0);
  close(dev1);
  return answers;
}

std::vector<std::string> triviaGameServer::wait_for_category_votes(){
  
  std::vector<std::string> votes;
  //Open Devices
  if ((dev0 = open (evID0.c_str(), O_RDONLY|O_NONBLOCK)) == -1){
          ROS_ERROR("NOT A VALID DEVICE - KEYBOARD 0 INPUT WON'T WORK");
          return votes;
  }
  if ((dev1 = open (evID1.c_str(), O_RDONLY|O_NONBLOCK)) == -1){
          ROS_ERROR("NOT A VALID DEVICE - KEYBOARD 1 INPUT WON'T WORK");
          return votes;
  }
  
  double start =ros::Time::now().toSec();
  double curr_time=ros::Time::now().toSec();
  
  bool keyboard0_answered=0;
  bool keyboard1_answered=0;
  while ((curr_time-start)<20 && votes.size()!=2){
  
          memset((void*)&ev0, 0, sizeof(ev0));
          memset((void*)&ev1, 0, sizeof(ev1));
  
          rd0 = read (dev0, (void*)&ev0, sizeof(ev0));
          rd1 = read (dev1, (void*)&ev1, sizeof(ev1));
  
          if(rd0>0 && ev0.value==1 && keyboard0_answered==0){
                  string ans="";
                  
                 if(ev0.code==KEY_CODE_A){
                   ans="A1";
                   tangy.beep();
                   votes.push_back(ans);
                    keyboard0_answered=1;
                 }else if(ev0.code==KEY_CODE_B){
                   ans="A2";
                   tangy.beep();
                   votes.push_back(ans);
                    keyboard0_answered=1;
                 }else if(ev0.code==KEY_CODE_C){
                   ans="A3";
                   tangy.beep();
                   votes.push_back(ans);
                  keyboard0_answered=1;
                 }
                 if(keyboard0_answered){
                   tangy.moveNeck(RIGHT,-2);
                    usleep(100000);
                    tangy.nod();
                    sleep(1);
                 }
          }
          if(rd1>0 && ev1.value==1 && keyboard1_answered==0){
                  string ans="";
                 if(ev1.code==KEY_CODE_A){
                   ans="A1";
                   tangy.beep();
                   votes.push_back(ans);
                    keyboard1_answered=1;
                 }else if(ev1.code==KEY_CODE_B){
                   ans="A2";
                   tangy.beep();
                   votes.push_back(ans);
                    keyboard1_answered=1;
                 }else if(ev1.code==KEY_CODE_C){
                   ans="A3";
                   tangy.beep();
                   votes.push_back(ans);
                    keyboard1_answered=1;
                 }
                 if(keyboard1_answered){
                   tangy.moveNeck(LEFT,-2);
                   usleep(100000);
                    tangy.nod();
                    sleep(1);
                 }
          }
          
          curr_time=ros::Time::now().toSec();
  }
  
  close(dev0);
  close(dev1);
  return votes;
}
