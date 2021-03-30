#include <trivia/trivia.h>

void triviaGameServer::give_question(int cat_num){
  bool hint_given=false;
  bool answered=false;
  ROS_INFO("give_question()");
  vector<int> teams_answered;
  vector<string> question=quesHandlers.at(cat_num).get_ques();

  say_question(question.at(0),question.at(1), question.at(2), question.at(3));
  
  //While there are teams that haven't answered the question correctly yet
  while(teams_answered.size()<teams.size() && !answered){
    while(pause_game){
      //Pause the game
    }
    
    int team=-1;
    std::vector< std::string > response;
    response=wait_for_answer();
    if(response.at(1)=="0"){
	team=0;
    }else if (response.at(1)=="1"){
	team=1;
    }else if (response.at(1)=="2"){
	team=2;
    }else if (response.at(1)=="3"){
	team=3;
    }
    //team=atoi(response.at(1).c_str());   Don't want an accidental team==0 situation (since atoi default returns 0 for no-length strings
    bool prev_team=false;
    int j=0;
    ROS_INFO("Team = [%d], ans = [%s]", team, response.at(0).c_str());
    //Check if the team that answered didn't already just provide an answer
    while(j<teams_answered.size() && prev_team==false){
      if(team==teams_answered.at(j)){
        prev_team=true;
        team=3;
      }
      j++;
    }
    

    //If nobody responds
    if(team==-1 && hint_given){
      int ans_num;
      std::string answer;
      std::string answer_for_screen;
      if(question.at(4)=="A1"){
        ans_num=1;
        answer="The answer was  A";
        answer_for_screen="A: "+question.at(ans_num);
      }else if(question.at(4)=="A2"){
        ans_num=2;
        answer="The answer was  B";
        answer_for_screen="B: "+question.at(ans_num);
      }else{
        ans_num=3;
        answer="The answer was  C";
        answer_for_screen="C: "+question.at(ans_num);
      }
			if((rand()%100)<75){
			  tangy.convo_gesture();
			}
      say_and_display("Oh my, this was definitely a hard question. Well, no problem.",question.at(0), question.at(1), question.at(2), question.at(3));
      say_and_display(answer, answer_for_screen);
      say_and_display(question.at(ans_num), answer_for_screen);
      answered=true;
      return;
      
    //If a team wants to get help, turn and ask whether they want to hear the question again, or whether they want a hint
    }else if(team==-1 && !hint_given){
      tangy.convo_gesture();
      say_and_display("How about a hint?",question.at(0), question.at(1), question.at(2), question.at(3));
      give_hint(question);
      tangy.moveNeck(0,-2);
      hint_given=true;
      
    }else if(team==2){
      ROS_INFO("Help requested!");
      tangy.look_at_face();
			if((rand()%100)<75){
			  tangy.convo_gesture();
			}
      say_and_display("Would you like me to repeat the question? Or would you like a hint?", "A: Repeat", "B: Hint");
      
      string help_resp="";
      while(help_resp!="A1" && help_resp!="A2"){
        help_resp = wait_for_answer().at(0);
      }
      if(help_resp=="A1"){
        ROS_INFO("Repeating question...");
        say_question(question.at(0),question.at(1), question.at(2), question.at(3), true);
        tangy.moveNeck(0,-2);
      }else if(help_resp=="A2"){
        ROS_INFO("Giving hint...");
			  if((rand()%100)<75){
  			  tangy.convo_gesture();
  			}
        give_hint(question);
        
      }else{
        ROS_INFO("Bug in help code - var help_resp!= A1||A2");
      }
      tangy.stop_look_at_face();
      tangy.moveNeck(0,-2);
    }else if(team==3){
      ROS_INFO("Help requested!");
	tangy.convo_gesture();
      say_and_display("Would another team like to give the question a try?", question.at(0), question.at(1), question.at(2), question.at(3));

    }else{
    //If a team responds, find their location, face them, and then give them an opportunity to answer the question
      vector<float> location;
      location=loc_teams(team);
      tangy.reset_head_pos();
      double right_rad=RIGHT*3.141/180;
      double left_rad=LEFT*3.141/180;
      if(team==0){
	     tangy.rotate_manual(left_rad);
      }else{
	     tangy.rotate_manual(right_rad);
      }
      tangy.look_at_face();

      teams_answered.push_back(team);
      
      std::string answer=response.at(0);

      ROS_INFO("Checking answer to see if answers match (received ans=[%s], correct ans=[%s])", answer.c_str(), question.at(4).c_str());
      if(answer==question.at(4)){
        tangy.stop_look_at_face();
        tangy.clap();
        tangy.nod();
        ROS_INFO("Correct answer!");
        //If the team got the question correct
        string praise=helpHandler.get_praise();
        say_and_display(praise, "Great Job!");
        std::string answer_for_screen;
        
        if(question.at(4)=="A1"){
          answer_for_screen="A: "+question.at(1);
  			  if((rand()%100)<75){
    			  tangy.convo_gesture();
    			}
          say_and_display("The answer was  A", answer_for_screen);
          say_and_display(question.at(1), answer_for_screen);
        }else if(question.at(4)=="A2"){
          answer_for_screen="B: "+question.at(2);
    			if((rand()%100)<75){
        	  tangy.convo_gesture();
        	}
          say_and_display("The answer was  B", answer_for_screen);
          say_and_display(question.at(2), answer_for_screen);
        }else{
          answer_for_screen="C: "+question.at(3);
    			if((rand()%100)<75){
    			  tangy.convo_gesture();
    			}
          say_and_display("The answer was  C", answer_for_screen);
          say_and_display(question.at(3), answer_for_screen);
        }
        //say_and_display("The answer was "+ answer_for_screen, answer_for_screen);
        
      
        scores.at(team)=scores.at(team)+1;
        answered=true;
      }else if(answer=="help"){
        
        give_hint(question);
        tangy.moveNeck(0,-2);
      }else{
        //If they got it wrong
        tangy.stop_look_at_face();
        tangy.shake();
        ROS_INFO("Incorrect answer!");
        string encouragement=helpHandler.get_encouragement();
        say_and_display(encouragement,question.at(0), question.at(1), question.at(2), question.at(3));
      }
      tangy.stop_look_at_face();
      tangy.rotate("map",0);
      
      while(pause_game){
  	    //Pause the game
  	  }
  	  
      // tangy.face(0,0);
    
      if(!answered){
        if(teams_answered.size()<teams.size()){
    			if((rand()%100)<75){
    			  tangy.convo_gesture();
    			}
          say_and_display("Would another team like to give the question a try?", question.at(0), question.at(1), question.at(2), question.at(3));
    
        }else{
          int ans_num;
          std::string answer;
          std::string answer_for_screen;
          if(question.at(4)=="A1"){
            ans_num=1;
            answer="The answer was  A";
            answer_for_screen="A: "+question.at(ans_num);
          }else if(question.at(4)=="A2"){
            ans_num=2;
            answer="The answer was  B";
            answer_for_screen="B: "+question.at(ans_num);
          }else{
            ans_num=3;
            answer="The answer was  C";
            answer_for_screen="C: "+question.at(ans_num);
          }
          if((rand()%100)<75){
            tangy.convo_gesture();
          }
          say_and_display("Oh my, this was definitely a hard question. Well, no problem.",question.at(0), question.at(1), question.at(2), question.at(3));
          say_and_display(answer, answer_for_screen);
    			if((rand()%100)<75){
    			  tangy.convo_gesture();
    			}
          say_and_display(question.at(ans_num), answer_for_screen);
          answered=true;
        }
      }
        
    }
  }
    
}

void triviaGameServer::give_hint(vector<string> question){
  
  if(question.size()==6){
		if((rand()%100)<75){
		  tangy.convo_gesture();
		}
    say_and_display("Here's the hint: ", question.at(0), question.at(1), question.at(2), question.at(3));
    say_and_display(question.at(5), question.at(5));
    sleep(2);
    say_and_display("I hope that helped.",question.at(0), question.at(1), question.at(2), question.at(3));
    
  }else{
		if((rand()%100)<75){
		  tangy.convo_gesture();
		}
    say_and_display("Oops! Sorry, there's no hint for this question!", question.at(0), question.at(1), question.at(2), question.at(3));
  }
  
}
