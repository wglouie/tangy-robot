#include <tangyRobot.h>

void tangyRobot::gui_handler(robot_gui::Robot_guiGoal interface_goal, int delay) {

	robotGuiClient.sendGoalAndWait(interface_goal);
	sleep(delay);

}

   /*****************************************************Speech and Text********************************************************/
    
    
    
    /*Have the robot say, using the text to speech, the string argument provided
    The argument delay gives increasing delays between sentences as it is incremented from 0 to 4
    (0 indicates no delay, 4 indicates long delay)*/
    void tangyRobot::say(std::string activity, std::string str, int subtab){

      robot_gui::Robot_guiGoal goal;
      goal.activity = activity;
      goal.code=2;
      goal.speech=str;
      goal.subtab=subtab;
      gui_handler(goal, get_speech_delay(str));
      
    }
    /*Have the robot display on the screen the string argument provided*/
    void tangyRobot::display_text(std::string activity, std::string str, int subtab){
      
      robot_gui::Robot_guiGoal goal;
      goal.activity = activity;
      goal.code=0;
      goal.text=str;
      goal.subtab=subtab;
      gui_handler(goal, 1);
      
    }
    /*Say and display on the screen the same string*/
    void tangyRobot::say_and_display(std::string activity, std::string str, int subtab){
      robot_gui::Robot_guiGoal goal;
      goal.activity = activity;
      goal.code=2;
      goal.text=str;
      goal.speech=str;
      goal.subtab=subtab;
      gui_handler(goal, get_speech_delay(str));
    }
    void tangyRobot::say_and_display(std::string activity, std::string str0,std::string str1, int subtab){
      robot_gui::Robot_guiGoal goal;
      goal.activity = activity;
      goal.code=2;
      goal.text=str1;
      goal.speech=str0;
      goal.subtab=subtab;
      gui_handler(goal, get_speech_delay(str0));
    }
    void tangyRobot::say_and_display(std::string activity, std::string str0, std::string str1, std::string str2, std::string str3, std::string str4, int subtab){

      robot_gui::Robot_guiGoal goal;
      goal.activity = activity;
      goal.code=2;
      goal.speech=str0;
      goal.subtab=subtab;
      std::string formattedstr;
      if(str2.size()!=0 && str3.size()!=0 && str4.size()!=0){
        formattedstr=str1+"\n\n"+"A: "+ str2+"\n"+"B: "+str3+"\n"+"C: "+str4;
        ROS_INFO("Passing %s", formattedstr.c_str());
        goal.text=formattedstr;
      }else {
        goal.text=str1;
      }

      gui_handler(goal, get_speech_delay(str1));

    }
    
    /*Determine a proper delay time based on the size of the input string*/
    int tangyRobot::get_speech_delay(std::string str){
      int delay;
      if(str.size()<40){
        delay=0;
      }else{
        delay=0;
      }
      return delay;
    }
    
    /*Say and display str1 and str2, respectively*/

