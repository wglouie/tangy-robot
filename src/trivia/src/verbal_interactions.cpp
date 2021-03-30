#include <trivia/trivia.h>


void triviaGameServer::say(std::string speech){
  tangy.say(activity_code, speech);
}

//Subtabs: subtab 0 - one text label; subtab 1 - two text labels on one page; subtab 2 - four text labels on one page
void triviaGameServer::say_and_display(std::string s0, std::string s1, std::string s2, std::string s3, std::string s4){
  int s0l=s0.size();
  int s1l=s1.size();
  int s2l=s2.size();
  int s3l=s3.size();
  int s4l=s4.size();
  // ROS_INFO("s0.length=%d, s1.length=%d, s2.length=%d, s3.length=%d, s4.length=%d", s0l, s1l, s2l, s3l, s4l);
  if(s1.size()==0){
    tangy.say_and_display(activity_code, s0, s0,0);
  }else if(s2.size()==0){
    tangy.say_and_display(activity_code, s0, s1,0);
  }else if(s3.size()==0){
    //Two options case
    string formatted_string=s1+"\n"+s2;
    tangy.say_and_display(activity_code, s0, formatted_string, 1);
  }else if(s4.size()==0){
    //Three options case -- NOT USED YET, CAN CHANGE
    string formatted_string="A: "+s1+"\nB: "+s2+"\nC: "+s3;
    tangy.say_and_display(activity_code, s0, formatted_string);
  }else{
    //Question and three multiple choices case
    string formatted_string=s1+"\nA: "+s2+"\nB: "+s3+"\nC: "+s4;
    tangy.say_and_display(activity_code, s0, formatted_string, 2);
  }
}

void triviaGameServer::say_question(std::string q, std::string a1, std::string a2, std::string a3, bool repeat){
  if(!repeat){
    if(ques_count==0){
        say_and_display("Time for the first question!", q);
    }else{
        int m=rand()%100;
        if(m<33){
          say_and_display( "Alright. Let's move on to the next question.", q);
          			if((rand()%100)<75){
          			  tangy.convo_gesture();
          			}
        }else if(m<66){
          say_and_display( "Time for the next question.", q);
        }else{
          say_and_display( "How about another question?", q);
        }
    }
  }
  tangy.moveNeck(RIGHT,-2);
  tangy.point_at_screen();
  //Say the question and then all of the answers
	if((rand()%100)<50){
	  tangy.convo_gesture();
	}
  say_and_display( q, q);
  say_and_display( a1, q, a1, a2, a3);
  tangy.moveNeck(LEFT,-5);
  tangy.point_at_screen();
  say_and_display( a2, q, a1, a2, a3);
  tangy.moveNeck(RIGHT,-5);
  say_and_display( a3, q, a1, a2, a3);
  tangy.moveNeck(0,0);
  say_and_display("What is your answer?", q,a1,a2,a3);
}

void triviaGameServer::give_intro_speech(){
	tangy.wave();
	ROS_INFO("give_intro_speech()");
	vector<string> curr_line=introHandler.get_next_line();
	string last_displayed_line="";
	int line_count=0;
	while(!curr_line.empty()) {

		line_count++;

		if(last_displayed_line.compare( curr_line.at(1) )!=0) {
			say_and_display(curr_line.at(0), curr_line.at(1));
			last_displayed_line=curr_line.at(1);
			if(line_count%3==0){
			  tangy.convo_gesture();
			}
		}else {
			say_and_display(curr_line.at(0),last_displayed_line);
		}
		curr_line=introHandler.get_next_line();

	}
}


void triviaGameServer::give_outro_speech(){
ROS_INFO("give_outro_speech()");
	vector<string> curr_line=outroHandler.get_next_line();
	string last_displayed_line="";
	int line_count=3;
	while(!curr_line.empty()) {
		line_count--;

		if(line_count==0) {
			tangy.wave();
		}

		if(last_displayed_line.compare( curr_line.at(1) )!=0) {
			if((rand()%100)<50){
			  tangy.convo_gesture();
			}
			say_and_display(curr_line.at(0), curr_line.at(1));
			last_displayed_line=curr_line.at(1);

		} else{
			if((rand()%100)<50){
			  tangy.convo_gesture();
			}
			say_and_display(curr_line.at(0),last_displayed_line);

		}
		curr_line=outroHandler.get_next_line();
	}
}

void triviaGameServer::tell_joke(){
	ROS_INFO("tell_joke()");
	while(pause_game) {
	}
	if(end_game==false) {
		vector<string> joke= jokeHandler.get_rand_joke();

		if(joke.at(0).size()!=1){
      tangy.set_music_volume(tangyRobot::QUIET);
			// convo_gesture();
			say_and_display("How about a joke?");
			// convo_gesture();
			say_and_display(joke.at(0));
			// convo_gesture();
			
			sleep(2);
      int y=rand()%100;
      if(y<50){
        tangy.laugh_gesture();
      }
			
			say_and_display(joke.at(1));
			say_and_display("Hee! hee! hee!",joke.at(1));
      		tangy.set_music_volume(tangyRobot::LOUD);
		}
	}
  
}
