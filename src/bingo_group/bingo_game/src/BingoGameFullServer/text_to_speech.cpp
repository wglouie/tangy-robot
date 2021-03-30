#include <BingoGameFullServer/BingoGameFullServer.h>

   /*****************************************************Speech and Text********************************************************/
    
    
    
    /*Have the robot say, using the text to speech, the string argument provided
    The argument delay gives increasing delays between sentences as it is incremented from 0 to 4
    (0 indicates no delay, 4 indicates long delay)*/
    void BingoGameFullServer::say(std::string str){
      
      robot_gui::Robot_guiGoal goal;
      goal.activity = "bingo";
      goal.code=2;
      goal.speech=str;

      gui_handler(goal, get_speech_delay(str));
      
    }
    /*Have the robot display on the screen the string argument provided*/
    void BingoGameFullServer::display_text(std::string str){
      
      robot_gui::Robot_guiGoal goal;
      goal.activity = "bingo";
      goal.code=0;
      goal.text=str;
      
      gui_handler(goal, 1);
      
    }
    /*Say and display on the screen the same string*/
    void BingoGameFullServer::say_and_display(std::string str){
      robot_gui::Robot_guiGoal goal;
      goal.activity = "bingo";
      goal.code=2;
      goal.text=str;
      goal.speech=str;
    
      gui_handler(goal, get_speech_delay(str));
    }
    void BingoGameFullServer::say_and_display(std::string str1, std::string str2){
      #ifdef TTS
      robot_gui::Robot_guiGoal goal;
      goal.activity = "bingo";
      goal.code=2;
      goal.speech=str1;
      goal.text=str2;
      

      gui_handler(goal, get_speech_delay(str1));
      #endif
    }
    
    /*Determine a proper delay time based on the size of the input string*/
    int BingoGameFullServer::get_speech_delay(std::string str){
      int delay;
      if(str.size()<40){
        delay=0;
      }else{
        delay=0;
      }
      return delay;
    }
    
    /*Say and display str1 and str2, respectively*/

