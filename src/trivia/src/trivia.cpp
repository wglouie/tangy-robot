#include <trivia/trivia.h>
using namespace std;

//Create the trivia action server and read all the database files
triviaGameServer::triviaGameServer(std::string name):
  as_(nh_,name, boost::bind(&triviaGameServer::executeCB, this, _1), false),
  action_name_(name),
  tangy(),
  helpHandler(), introHandler(), outroHandler(), jokeHandler(){
    ROS_INFO("triviaGameServer():");
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

triviaGameServer::~triviaGameServer(){
}

bool triviaGameServer::init(){
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

void triviaGameServer::play_game(){
	ROS_INFO("play_game()");
 	give_intro_speech();
	int cat_num=0;
    tangy.moveNeck(LEFT,-5);
    tangy.moveNeck(RIGHT,-5);
	tangy.nod();
	sleep(4);
	//Seed the first category choices
	int cc0=cat_num;
	int cc1=rand()%categories.size();
	int cc2=rand()%categories.size();
	std::string curr_cat=categories.at(cat_num);
  std::string cat2=categories.at(cc1);
  std::string cat3=categories.at(cc2);

	//Loop through the questions in the category, and query the category after every five questions
  double start =ros::Time::now().toSec();
  double end=ros::Time::now().toSec();
  //End the game either when the operator decides to end it, or after 1 hr
	while(end_game==false && (end-start)<3600){
    while(pause_game){
	    //Pause the game
	  }
	  
	  //Other interactions
		if(ques_count % 5 ==0 && ques_count!=0){
		  tell_joke();
		}
		
	  //Query categories every five numbers
		if(ques_count % 5==0){
			if(ques_count!=0){
				tangy.move("map", 0, 0);
				tangy.rotate("map",0);
			}
			//Recount number of questions in each category
			vector<int> num_ques_in_cats;
			for(int c=0;c<categories.size();c++){
				int num=quesHandlers.at(c).get_num_ques();
				num_ques_in_cats.push_back(num);
			}
			if(num_ques_in_cats.at(cat_num)<5){
				cc0=rand()%categories.size();
				curr_cat=categories.at(cc0);
			}
			while(cat2==curr_cat && num_ques_in_cats.at(cc1)>4){
				cc1=rand()%categories.size();
				cat2=categories.at(cc1);
			}
			while(cat3==curr_cat||cat3==cat2 && num_ques_in_cats.at(cc2)>4){
				cc2=rand()%categories.size();
				cat3=categories.at(cc2);
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
		
    end=ros::Time::now().toSec();
	}
  while(pause_game){
	    //Pause the game
  }
  
  //End game
  say_and_display("So that will be the last question.", "Congratulations!");
  if(scores.at(0)==scores.at(1)){
    tangy.play_music("celebrate");
    tangy.celebrate();
    say_and_display("Congratulations to both teams! You both answered an equal number of questions correctly!", "Congratulations!");
  }else{
    std::string t_name;
    if(scores.at(0)>scores.at(1)){
      t_name="1";
    }else{
      t_name="2";
    }
    tangy.play_music("celebrate");
    tangy.celebrate();
    say_and_display("Congratulations to Team "+t_name+"! You're the winner today!", "Congratulations!");
  }
  sleep(15);
  tangy.stop_music();
	give_outro_speech();
	while(1){
	  //END GAME LOOP
	}
}


int triviaGameServer::query_category(std::string curr_cat, std::string cat2, std::string cat3){
  
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
  tangy.moveNeck(LEFT,-5);
  tangy.point_at_screen();
  say_and_display( curr_cat, "Categories:", curr_cat, cat2, cat3);
  say_and_display( cat2, "Categories:", curr_cat, cat2, cat3);
  tangy.moveNeck(RIGHT,-5);
  say_and_display( cat3, "Categories:", curr_cat, cat2, cat3);
  tangy.moveNeck(0,-2);
  
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
  
  if((rand()%100)<50){
    tangy.convo_gesture();
  }
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
    say_and_display("There was a tie between ","B: "+cat2,"C: "+cat3);
    say_and_display(cat2,"B: "+cat2,"C: "+cat3);
    say_and_display(" and ","B: "+cat2,"C: "+cat3);
    say_and_display(cat3,"B: "+cat2,"C: "+cat3);
    say_and_display("I will choose randomly between the two categories.","B: "+cat2,"C: "+cat3);
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
    say_and_display("There was a tie between ","A: "+curr_cat,"C: "+cat3 );
    say_and_display(curr_cat,"A: "+curr_cat,"C: "+cat3 );
    say_and_display(" and ","A: "+curr_cat,"C: "+cat3 );
    say_and_display(cat3,"A: "+curr_cat,"C: "+cat3 );
    say_and_display("I will choose randomly between the two categories.","A: "+curr_cat,"C: "+cat3 );
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

vector<float> triviaGameServer::loc_teams(int team){
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

void triviaGameServer::endGameCallback(const std_msgs::String::ConstPtr& str) {
	ROS_INFO("Ending Game!");
	if(str->data.compare("y")==0) {
		end_game=true;
	} else if(str->data.compare("n")==0) {
		end_game=false;
	}
}

void triviaGameServer::pauseCallback(const std_msgs::String::ConstPtr& str) {
	ROS_INFO("Pausing!");
	if(str->data.compare("pause")==0) {
		pause_game=true;
	} else if(str->data.compare("unpause")==0) {
		pause_game=false;
	}
}

//Eventually, maybe add the same options as the bingo game -- continue game,
//do demo, etc.
void triviaGameServer::executeCB(const trivia::TriviaGameGoal::ConstPtr &goal){
  tangy.init();
    tangy.reset_head_pos();
    ROS_INFO("Goal Received!!");
    if(goal->start==0){
        ROS_INFO("Starting a new game!");
        end_game=false;
        tangy.reset_head_pos();

        //delete_progress();    *********** Not useful yet
        //init();
        play_game();
        
    }else{
      end_game=true;
    }
}

bool triviaGameServer::delete_progress(){
    jokeHandler.delete_progress();
    introHandler.delete_progress();
    outroHandler.delete_progress();
    helpHandler.delete_progress();
    ROS_INFO("Progress deleted!");
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "trivia_game");
    ros::NodeHandle n;
    srand(time (NULL));
    triviaGameServer game("trivia_game");
    ros::Subscriber pause_sub=n.subscribe<std_msgs::String>("control_command",10, &triviaGameServer::pauseCallback, &game);
    ros::Subscriber cancel_sub=n.subscribe<std_msgs::String>("end_game",10, &triviaGameServer::endGameCallback, &game);
    ROS_INFO("[%s] Trivia Game server is running" , ros::this_node::getName().c_str());


    ros::spin();
    return 0;
}

