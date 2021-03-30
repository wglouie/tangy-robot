/*A modified version of trivia.cpp WHICH DOESN'T REQUIRE THE ROBOT TO BE RUN
Created in order to test the FSM, gui and the audio ONLY during the game
*/

#include <trivia/trivia_limited.h>
using namespace std;

//Create the trivia action server and read all the database files
triviaGameServer_limited::triviaGameServer_limited(std::string name):
  as_(nh_,name, boost::bind(&triviaGameServer_limited::executeCB, this, _1), false),
  action_name_(name),robotGuiClient("gui"),
  helpHandler(), introHandler(), outroHandler(), jokeHandler(){
    ROS_INFO("triviaGameServer_limited");
    as_.start();
    
    //Get verbal interactions
    nh_.getParam("/introFile", introFileLocation);
    nh_.getParam("/outroFile", outroFileLocation);
    nh_.getParam("/helpTextFile", helpTextFileLocation);
    nh_.getParam("/jokeFile", jokeFileLocation);
    nh_.getParam("/keylog0", keyLog0);
    nh_.getParam("/keylog1", keyLog1);
    
    nh_.getParam("/evID0", evID0);
    nh_.getParam("/evID1", evID1);
    
    //Read index file to get categories
    nh_.getParam("/index", indexFilePath);
    std::ifstream ifs(indexFilePath.c_str());
    if (ifs.is_open()){
      std::string line;
      
      while ( getline (ifs,line) )
      {

        if(!line.empty())
        {
          categories.push_back(line);
          ROS_INFO("Stored category: %s", line.c_str());
          for(int len=0; len<line.length(); len++){
        		if(line.at(len)==' '){
        			line.at(len)='_';
        		}
        	}
        	categories_for_paths.push_back(line);
        }

      }
      ifs.close();
    }
    if(categories.size()<3){
      ROS_ERROR("**********NOT ENOUGH CATEGORIES IN THE DATABASE!!************");
    }
    //Store Categories and create question handlers for each category
    for(int i=0; i<categories_for_paths.size(); i++){
      
      std::string path;
      nh_.getParam("/"+categories_for_paths.at(i), path);
      filePaths.push_back(path);
      trivia_question_handler handler;
      handler.read_file(path);
      quesHandlers.push_back(handler);
    }
    activity_code="trivia";
    
    
	  init();
}

triviaGameServer_limited::~triviaGameServer_limited(){
}

bool triviaGameServer_limited::init(){
	ROS_INFO("init()");
	end_game=false;
	ques_count=0;
	pause_game=false;

	jokeHandler.read_file(jokeFileLocation);
	introHandler.read_file(introFileLocation);
	outroHandler.read_file(outroFileLocation);
	helpHandler.read_file(helpTextFileLocation);
	
	calibrate_keyboards();
}

void triviaGameServer_limited::play_game(){
	ROS_INFO("play_game()");
 	// give_intro_speech();
	int cat_num=0;

	//Seed the first category choices
	std::string curr_cat=categories.at(cat_num);
  std::string cat2=categories.at(rand()%categories.size());
  std::string cat3=categories.at(rand()%categories.size());

	//Loop through the questions in the category, and query the category after every five questions
  double start =ros::Time::now().toSec();
  double end=ros::Time::now().toSec();
  //End the game either when the operator decides to end it, or after 1 hr
	while(end_game==false && (end-start)<3600){
    while(pause_game){
	    //Pause the game
	  }
	  
	  //Query categories every five numbers
		if(ques_count % 5==0){
  	  while(cat2==curr_cat){
  	    cat2=categories.at(rand()%categories.size());
  	  }
  	  while(cat3==curr_cat||cat3==cat2){
  	    cat3=categories.at(rand()%categories.size());
  	  }
			cat_num=query_category(curr_cat, cat2, cat3);
			curr_cat=categories.at(cat_num);

    	ROS_INFO("The current category is %s", curr_cat.c_str());
		}
	  
	  //Questions
		give_question(cat_num);
		ques_count++;
		ROS_INFO("Number of questions given so far in the game=%d", ques_count);
    while(pause_game){
	    //Pause the game
	  }
	  
	  //Other interactions
		if(ques_count % 7 ==0 && ques_count!=0){
		  tell_joke();
		}
		
    end=ros::Time::now().toSec();
	}
  while(pause_game){
	    //Pause the game
  }
  
  //End game
  say_and_display("So that will be the last question.", "Congratulations!");
  if(scores.at(0)==scores.at(1)){

    say_and_display("Congratulations to both teams! You both answered an equal number of questions correctly!", "Congratulations!");
  }else{
    std::string t_name;
    if(scores.at(0)>scores.at(1)){
      t_name="1";
    }else{
      t_name="2";
    }

    say_and_display("Congratulations to Team "+t_name+"! You're the winner today!", "Congratulations!");
  }
  
	give_outro_speech();
	while(1){
	  //END GAME LOOP
	}
}


int triviaGameServer_limited::query_category(std::string curr_cat, std::string cat2, std::string cat3){
  
  //Present the categories verbally and on the screen
  ROS_INFO("query_category()");
  if(ques_count==0){
    say_and_display( "Let's choose our first category!", "Categories");
  }else{
    int m=rand()%100;
    if(m>50){
      say_and_display( "Do you want to switch categories?", "Categories");
    }else{
      say_and_display( "We can stay on this category, or change to another one.", "Categories");
    }
  }
  say_and_display( "The categories are:", "Categories:", curr_cat, cat2, cat3);

  say_and_display( curr_cat, "Categories:", curr_cat, cat2, cat3);
  say_and_display( cat2, "Categories:", curr_cat, cat2, cat3);
  say_and_display( cat3, "Categories:", curr_cat, cat2, cat3);
  
  //Query teams to choose a category
  vector<string> cat_choices;
    say_and_display("Which category would you like?","Categories:", curr_cat, cat2, cat3);
  cat_choices=wait_for_category_votes();

  int vote0=0;
  int vote1=0;
  int vote2=0;
  for(int i=0; i<cat_choices.size(); i++){
    if(cat_choices.at(i)=="A1"){
      vote0++;
    }else if(cat_choices.at(i)=="A2"){
      vote1++;
    }else if(cat_choices.at(i)=="A3"){
      vote2++;
    }
  }

  ROS_INFO("The votes are: %d, %d, %d", vote0, vote1, vote2);
  std::string next_cat;
  
  if(vote0>vote1 && vote0>vote2){
    // say_and_display("So the chosen category is: "+curr_cat, curr_cat);     ORIG FORMAT
     say_and_display("So the chosen category is: ", curr_cat);
    say_and_display(curr_cat, curr_cat);
    next_cat=curr_cat;
  }else if(vote1>vote0 && vote1>vote2){
    say_and_display("So the chosen category is: ", cat2);
    say_and_display(cat2, cat2);
    next_cat=cat2;
  }else if(vote2>vote0 && vote2>vote1){
    say_and_display("So the chosen category is: ", cat3);
    say_and_display(cat3, cat3);
    next_cat=cat3;
  }else if(vote0==vote1){
    say_and_display("There was a tie between ","A: "+curr_cat,"B: "+cat2);
    say_and_display(curr_cat,"A: "+curr_cat,"B: "+cat2);
    say_and_display(" and ","A: "+curr_cat,"B: "+cat2);
    say_and_display(cat2,"A: "+curr_cat,"B: "+cat2);
    say_and_display("I will choose randomly between the two categories.", "A: "+curr_cat,"B: "+cat2);
    int r=rand()%100;
    if(r<50){
      say_and_display("The next category will be: ", curr_cat);
      say_and_display(curr_cat, curr_cat);
      next_cat=curr_cat;
    }else{
      say_and_display("The next category will be: ", cat2);
      say_and_display(cat2, cat2);
      next_cat=cat2;
    }
  }else if(vote1==vote2){
    std::string tied_cats="B: "+cat2+"\nC: "+cat3;
    say_and_display("There was a tie between ",tied_cats);
    say_and_display(cat2,tied_cats);
    say_and_display(" and ",tied_cats);
    say_and_display(cat3,tied_cats);
    say_and_display("I will choose randomly between the two categories.", tied_cats);
    int r=rand()%100;
    if(r<50){
      say_and_display("The next category will be: ", cat3);
      say_and_display(cat3, cat3);
      next_cat=cat3;
    }else{
      say_and_display("The next category will be: ", cat2);
      say_and_display(cat2, cat2);
      next_cat=cat2;
    }
  }else{
    std::string tied_cats="A: "+curr_cat+"\nC: "+cat3;
    say_and_display("There was a tie between ",tied_cats);
    say_and_display(curr_cat,tied_cats);
    say_and_display(" and ",tied_cats);
    say_and_display(cat3,tied_cats);
    say_and_display("I will choose randomly between the two categories.",tied_cats);
    int r=rand()%100;
    if(r<50){
      say_and_display("The next category will be: ", curr_cat);
      say_and_display(curr_cat, curr_cat);
      next_cat=curr_cat;
    }else{
      say_and_display("The next category will be: ", cat3);
      say_and_display(cat3, cat3);
      next_cat=cat3;
    }
  }
  
  for(int i=0; i<categories.size(); i++){
    if(next_cat==categories.at(i)){
      ROS_INFO("The next category has id=%d, and is %s", i, next_cat.c_str());
      return i;
    }
  }
  ROS_ERROR("COULDN'T PICK A CATEGORY FOR SOME REASON");
  return 0;
}

vector<float> triviaGameServer_limited::loc_teams(int team){
  ROS_INFO("loc_teams()");
  vector<float> location;
  if(team==-1){
    location.push_back(0);
    location.push_back(0);
  }else{
    if(team<teams.size()){
      location=locations.at(team);
    }else{
      location.push_back(0);
      location.push_back(0);
    }
    
  }
  return location;
}

void triviaGameServer_limited::endGameCallback(const std_msgs::String::ConstPtr& str) {
	ROS_INFO("Ending Game!");
	if(str->data.compare("y")==0) {
		end_game=true;
	} else if(str->data.compare("n")==0) {
		end_game=false;
	}
}

void triviaGameServer_limited::pauseCallback(const std_msgs::String::ConstPtr& str) {
	ROS_INFO("Pausing!");
	if(str->data.compare("pause")==0) {
		pause_game=true;
	} else if(str->data.compare("unpause")==0) {
		pause_game=false;
	}
}

//Eventually, maybe add the same options as the bingo game -- continue game,
//do demo, etc.
void triviaGameServer_limited::executeCB(const trivia::TriviaGameGoal::ConstPtr &goal){
    ROS_INFO("Goal Received!!");
    if(goal->start==0){
        ROS_INFO("Starting a new game!");
        end_game=false;

        //delete_progress();    *********** Not useful yet
        //init();
        play_game();
        
    }else{
      end_game=true;
    }
}

bool triviaGameServer_limited::delete_progress(){
    jokeHandler.delete_progress();
    introHandler.delete_progress();
    outroHandler.delete_progress();
    helpHandler.delete_progress();
    ROS_INFO("Progress deleted!");
}

void triviaGameServer_limited::give_question(int cat_num){
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
    team=wait_for_response();
    bool prev_team=false;
    int j=0;
    
    //Check if the team that answered didn't already just provide an answer
    while(j<teams_answered.size() && prev_team==false){
      if(team==teams_answered.at(j)){
        prev_team=true;
        team=-1;
      }
      j++;
    }
    

    //If nobody responds
    if(team==-1){
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
			      say_and_display("Oh my, this was definitely a hard question. Well, no problem.",question.at(0), question.at(1), question.at(2), question.at(3));
      say_and_display(answer, answer_for_screen);
      say_and_display(question.at(ans_num), answer_for_screen);
      answered=true;
      return;
      
    //If a team wants to get help, turn and ask whether they want to hear the question again, or whether they want a hint
    }else if(team==2){
      ROS_INFO("Help requested!");
      
			      say_and_display("Would you like me to repeat the question? Or would you like a hint?", "Repeat", "Hint");
      
      string help_resp="";
      while(help_resp!="A1" && help_resp!="A2"){
        help_resp = wait_for_answer(team);
      }
      if(help_resp=="A1"){
        ROS_INFO("Repeating question...");
        say_question(question.at(0),question.at(1), question.at(2), question.at(3), true);
      }else if(help_resp=="A2"){
        ROS_INFO("Giving hint...");
			          give_hint(question);
      }else{
        ROS_INFO("Bug in help code - var help_resp!= A1||A2");
      }
      //Rotate back to face front
      
    }else{
    //If a team responds, find their location, face them, and then give them an opportunity to answer the question
      vector<float> location;
      location=loc_teams(team);
      
      
      
      
      std::stringstream ss;
      ss << (team+1);
      std::string str_num = ss.str();
      say_and_display("Team "+str_num+".", question.at(0), question.at(1), question.at(2), question.at(3));
      teams_answered.push_back(team);
      
      std::string answer;
      answer=wait_for_answer(team);
      ROS_INFO("Checking answer to see if answers match (received ans=[%s], correct ans=[%s])", answer.c_str(), question.at(4).c_str());
      if(answer==question.at(4)){
        ROS_INFO("Correct answer!");
        //If the team got the question correct
        string praise=helpHandler.get_praise();
        say_and_display(praise, "Great Job!");
        std::string answer_for_screen;
        
        if(question.at(4)=="A1"){
          answer_for_screen="A: "+question.at(1);
  			            say_and_display("The answer was A", answer_for_screen);
          say_and_display(question.at(1), answer_for_screen);
        }else if(question.at(4)=="A2"){
          answer_for_screen="B: "+question.at(2);
    			          say_and_display("The answer was B", answer_for_screen);
          say_and_display(question.at(2), answer_for_screen);
        }else{
          answer_for_screen="C: "+question.at(3);
    			          say_and_display("The answer was C", answer_for_screen);
          say_and_display(question.at(3), answer_for_screen);
        }
        //say_and_display("The answer was "+ answer_for_screen, answer_for_screen);
        
      
        scores.at(team)=scores.at(team)+1;
        answered=true;
      }else if(answer=="help"){
        
        give_hint(question);
        
      }else{
        //If they got it wrong
        ROS_INFO("Incorrect answer!");
        string encouragement=helpHandler.get_encouragement();
        say_and_display(encouragement,question.at(0), question.at(1), question.at(2), question.at(3));
      }
      
      
      while(pause_game){
  	    //Pause the game
  	  }
  	  
      //
    
      if(!answered){
        if(teams_answered.size()<teams.size()){
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
                    say_and_display("Oh my, this was definitely a hard question. Well, no problem.",question.at(0), question.at(1), question.at(2), question.at(3));
          say_and_display(answer, answer_for_screen);
    			          say_and_display(question.at(ans_num), answer_for_screen);
          answered=true;
        }
      }
        
    }
  }
    
}

void triviaGameServer_limited::give_hint(vector<string> question){
  
  if(question.size()==6){
		    say_and_display("Here's the hint: ", question.at(0), question.at(1), question.at(2), question.at(3));
    say_and_display(question.at(5), question.at(5));
    sleep(2);
    say_and_display("I hope that helped.",question.at(0), question.at(1), question.at(2), question.at(3));
    
  }else{
		    say_and_display("Oops! Sorry, there's no hint for this question!", question.at(0), question.at(1), question.at(2), question.at(3));
  }
  
}

void triviaGameServer_limited::say(std::string speech){
  
}

//Subtabs: subtab 0 - one text label; subtab 1 - two text labels on one page; subtab 2 - four text labels on one page
void triviaGameServer_limited::say_and_display(std::string s0, std::string s1, std::string s2, std::string s3, std::string s4){
  int s0l=s0.size();
  int s1l=s1.size();
  int s2l=s2.size();
  int s3l=s3.size();
  int s4l=s4.size();
  // ROS_INFO("s0.length=%d, s1.length=%d, s2.length=%d, s3.length=%d, s4.length=%d", s0l, s1l, s2l, s3l, s4l);
  if(s1.size()==0){
    say_and_display(activity_code, s0, s0,0);
  }else if(s2.size()==0){
    say_and_display(activity_code, s0, s1,0);
  }else if(s3.size()==0){
    //Two options case
    string formatted_string=s1+"\n"+s2;
    say_and_display(activity_code, s0, formatted_string, 1);
  }else if(s4.size()==0){
    //Three options case -- NOT USED YET, CAN CHANGE
    string formatted_string="A: "+s1+"\nB: "+s2+"\nC: "+s3;
    say_and_display(activity_code, s0, formatted_string);
  }else{
    //Question and three multiple choices case
    string formatted_string=s1+"\nA: "+s2+"\nB: "+s3+"\nC: "+s4;
    say_and_display(activity_code, s0, formatted_string, 2);
  }
}

void triviaGameServer_limited::say_question(std::string q, std::string a1, std::string a2, std::string a3, bool repeat){
  if(!repeat){
    if(ques_count==0){
        say("Time for the first question!");
    }else{
        int m=rand()%100;
        if(m<33){
          say( "Alright. Let's move on to the next question.");
        }else if(m<66){
          say( "Time for the next question.");
        }else{
          say( "How about another question?");
        }
    }
  }
  
  //Say the question and then all of the answers
	 say_and_display( q, q);
  say_and_display( a1, q, a1, a2, a3);
  say_and_display( a2, q, a1, a2, a3);
  say_and_display("or",q,a1,a2,a3);
  say_and_display( a3, q, a1, a2, a3);
  
}

void triviaGameServer_limited::give_intro_speech(){
	
	ROS_INFO("give_intro_speech()");
	vector<string> curr_line=introHandler.get_next_line();
	string last_displayed_line="";
	int line_count=0;
	while(!curr_line.empty()) {

		line_count++;

		if(last_displayed_line.compare( curr_line.at(1) )!=0) {
			say_and_display(curr_line.at(0), curr_line.at(1));
			last_displayed_line=curr_line.at(1);
					}else {
			say_and_display(curr_line.at(0),last_displayed_line);
		}
		curr_line=introHandler.get_next_line();

	}
}


void triviaGameServer_limited::give_outro_speech(){
ROS_INFO("give_outro_speech()");
	vector<string> curr_line=outroHandler.get_next_line();
	string last_displayed_line="";
	int line_count=3;
	while(!curr_line.empty()) {
		line_count--;

		if(line_count==0) {
			
		}

		if(last_displayed_line.compare( curr_line.at(1) )!=0) {
						say_and_display(curr_line.at(0), curr_line.at(1));
			last_displayed_line=curr_line.at(1);

		} else{
						say_and_display(curr_line.at(0),last_displayed_line);

		}
		curr_line=outroHandler.get_next_line();
	}
}

void triviaGameServer_limited::tell_joke(){
	ROS_INFO("tell_joke()");
	while(pause_game) {
	}
	if(end_game==false) {
		vector<string> joke= jokeHandler.get_rand_joke();

		if(joke.at(0).size()!=1){
      
			// convo_gesture();
			say_and_display("How about a joke?");
			// convo_gesture();
			say_and_display(joke.at(0));
			// convo_gesture();
			
			sleep(2);
      int y=rand()%100;
      if(y<50){
        
      }
			
			say_and_display(joke.at(1));
			say_and_display("Hee! hee! hee!",joke.at(1));
      
		}
	}
  
}

//Using the system() command liberally, which is dangerous
//Ideally should use the X library, which the xinput utility is
//based on. Leaving as a TODO for the future

void triviaGameServer_limited::calibrate_keyboards(){
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
  
  location0.push_back(1.33);
  location0.push_back(-0.65);
  
  location1.push_back(1.33);
  location1.push_back(0.65);
  
  locations.push_back(location0);
  locations.push_back(location1);
  
  for(int i=0; i<locations.size(); i++){
    scores.push_back(i);
  }
}

//Returns the team number, or -1 if no teams respond
int triviaGameServer_limited::wait_for_response(){
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
  while ((curr_time-start)<20 && response==-1){
  
          memset((void*)&ev0, 0, sizeof(ev0));
          memset((void*)&ev1, 0, sizeof(ev1));
  
          rd0 = read (dev0, (void*)&ev0, sizeof(ev0));
          rd1 = read (dev1, (void*)&ev1, sizeof(ev1));
  
          if(rd0>0 && ev0.value==1){
                 if(ev0.code==KEY_CODE_RESPONSE){
                   response=0;
                   
                 }else if(ev0.code==KEY_CODE_HELP){
                   response=2;
                   
                 }
                 ROS_INFO("RESPONSE GIVEN BY USER IS [%d]/KEYBOARD CODE is [%d]", response, ev0.code);
          }
          if(rd1>0 && ev1.value==1){
                 if(ev1.code==KEY_CODE_RESPONSE){
                   response=1;
                   
                 }else if(ev1.code==KEY_CODE_HELP){
                   response=2;
                   
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

std::string triviaGameServer_limited::wait_for_answer(int team){
  string ans="";
  //Open Devices
  if ((dev0 = open (evID0.c_str(), O_RDONLY|O_NONBLOCK)) == -1){
          ROS_ERROR("NOT A VALID DEVICE - KEYBOARD 0 INPUT WON'T WORK");
          return "";
  }
  if ((dev1 = open (evID1.c_str(), O_RDONLY|O_NONBLOCK)) == -1){
          ROS_ERROR("NOT A VALID DEVICE - KEYBOARD 1 INPUT WON'T WORK");
          return "";
  }
  
  double start =ros::Time::now().toSec();
  double curr_time=ros::Time::now().toSec();
  
  while ((curr_time-start)<25 && ans.size()==0){
  
          memset((void*)&ev0, 0, sizeof(ev0));
          memset((void*)&ev1, 0, sizeof(ev1));
  
          rd0 = read (dev0, (void*)&ev0, sizeof(ev0));
          rd1 = read (dev1, (void*)&ev1, sizeof(ev1));
  
          if(rd0>0 && ev0.value==1){
                 if(ev0.code==KEY_CODE_A){
                   ans="A1";
                   
                 }else if(ev0.code==KEY_CODE_B){
                   ans="A2";
                   
                 }else if(ev0.code==KEY_CODE_C){
                   ans="A3";
                   
                 }else if(ev0.code==KEY_CODE_HELP){
                   ans="help";
                   
                 }
                 ROS_INFO("RESPONSE GIVEN BY USER IS [%s]/KEYBOARD CODE is [%d]", ans.c_str(), ev0.code);
          }
          if(rd1>0 && ev1.value==1){
                 if(ev1.code==KEY_CODE_A){
                   ans="A1";
                   
                 }else if(ev1.code==KEY_CODE_B){
                   ans="A2";
                   
                 }else if(ev1.code==KEY_CODE_C){
                   ans="A3";
                   
                 }else if(ev0.code==KEY_CODE_HELP){
                   ans="help";
                   
                 }
                 ROS_INFO("RESPONSE GIVEN BY USER IS [%s]/KEYBOARD CODE is [%d]", ans.c_str(), ev1.code);
          }
          
          curr_time=ros::Time::now().toSec();
  }
  
  close(dev0);
  close(dev1);
  return ans;
}

std::vector<std::string> triviaGameServer_limited::wait_for_category_votes(){
  
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
  while ((curr_time-start)<30 && votes.size()!=2){
  
          memset((void*)&ev0, 0, sizeof(ev0));
          memset((void*)&ev1, 0, sizeof(ev1));
  
          rd0 = read (dev0, (void*)&ev0, sizeof(ev0));
          rd1 = read (dev1, (void*)&ev1, sizeof(ev1));
  
          if(rd0>0 && ev0.value==1 && keyboard0_answered==0){
                  string ans="";
                 if(ev0.code==KEY_CODE_A){
                   ans="A1";
                   votes.push_back(ans);
                keyboard0_answered=1;
                 }else if(ev0.code==KEY_CODE_B){
                   ans="A2";
                   votes.push_back(ans);
                keyboard0_answered=1;
                 }else if(ev0.code==KEY_CODE_C){
                   ans="A3";
                   votes.push_back(ans);
                  keyboard0_answered=1;
                 }
          }
          if(rd1>0 && ev1.value==1 && keyboard1_answered==0){
                  string ans="";
                 if(ev1.code==KEY_CODE_A){
                   ans="A1";
                   votes.push_back(ans);
                    keyboard1_answered=1;
                 }else if(ev1.code==KEY_CODE_B){
                   ans="A2";
                   votes.push_back(ans);
                    keyboard1_answered=1;
                 }else if(ev1.code==KEY_CODE_C){
                   ans="A3";
                   votes.push_back(ans);
                    keyboard1_answered=1;
                 }
                 
          }
          
          curr_time=ros::Time::now().toSec();
  }
}

void triviaGameServer_limited::say_and_display(std::string activity, std::string str0, std::string str1, int subtab){
    robot_gui::Robot_guiGoal goal;
    goal.activity = activity;
    goal.code=2;
    goal.speech=str0;
    goal.text=str1;
    goal.subtab=subtab;
    robotGuiClient.sendGoalAndWait(goal);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trivia_game");
    ros::NodeHandle n;
    srand(time (NULL));
    triviaGameServer_limited game("trivia_game");
    ros::Subscriber pause_sub=n.subscribe<std_msgs::String>("control_command",10, &triviaGameServer_limited::pauseCallback, &game);
    ros::Subscriber cancel_sub=n.subscribe<std_msgs::String>("end_game",10, &triviaGameServer_limited::endGameCallback, &game);
    ROS_INFO("[%s] Trivia Game server is running" , ros::this_node::getName().c_str());


    ros::spin();
    return 0;
}

