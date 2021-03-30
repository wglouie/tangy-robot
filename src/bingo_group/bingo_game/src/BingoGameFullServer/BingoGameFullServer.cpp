#include <BingoGameFullServer/BingoGameFullServer.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

BingoGameFullServer::BingoGameFullServer(std::string name):
    as_(nh_,name, boost::bind(&BingoGameFullServer::executeCB, this, _1), false),
    action_name_(name),
    robotGuiClient("RobotGuiClient"), bingoDetectionClient("BingoDetection"),armMovementClient("ArmMovementClient"), moveBaseClient("MoveBaseClient"), faceDetectionClient("FaceDetectionClient"),
    jokeHandler(), introHandler(),introDemoHandler(), demoHelloHandler(), outroHandler(), smallTalkHandler(), numberHandler(), demoNumberHandler(), helpHandler(), navClient(), musicPlayer(), neckHandler("bingo_game")
{
    as_.start();
    nh_.getParam("/resultFile", resultFileLocation);
    nh_.getParam("/bingoNumbers", bingoGameNumbersFileLocation);
    nh_.getParam("/jokeFile", jokesFileLocation);
    nh_.getParam("/smallTalkFile", smallTalkFileLocation);
    nh_.getParam("/introFile", introFileLocation);
    nh_.getParam("/outroFile", outroFileLocation);
    nh_.getParam("/progressFile", progressFileLocation);
    nh_.getParam("/helpTextFile", helpTextFileLocation);
    nh_.getParam("/demoprogressFile",demoprogressFileLocation);
    nh_.getParam("/demoIntroFile", demoIntroLocation);
    nh_.getParam("/demoHelloFile", deomHelloLocation);

    init();
    navClient.initialize(nh_);
    game_began=false;
    wave_plan_num=0;
    point_at_screen_plan_num=0;
    celebrate_plan_num=0;
    convo_gesture_num=0;
    num_times_helped=0;
    num_times_failed=0;
    pause_game=false;
    end_game=false;
    stop_msg.data = "stop";
    go_msg.data = "go";

    client = nh_.serviceClient<help_indicators::get_help_indicators>("get_help_indicators");
    go_pub = nh_.advertise<std_msgs::String>("help_indicators_go", 1000);
    clear_pub = nh_.advertise<std_msgs::String>("clear_help_indicator", 1000);
    debug_pub = nh_.advertise<std_msgs::String>("debug_move", 1000);
    game_state_pub=nh_.advertise<std_msgs::String>("game_state",1000);
    nod_pub=nh_.advertise<std_msgs::Bool>("head_ready",1000);
    start_track_face_pub=nh_.advertise<std_msgs::String>("start_track_face",1000);
    stop_track_face_pub=nh_.advertise<std_msgs::String>("stop_track_face",1000);
              
    //choosePOI = nh_.advertise<std_msgs::String>("pick_center_person", 10);
/////////////////////////////////////////////////////////////
    stop_neck_movement = nh_.advertise<std_msgs::String>("stop_neck_movement",1);
    pick_a_person_pub = nh_.advertise<std_msgs::String>("pick_a_person",1);
////////////////////////////////////////////////////////////

    ROS_INFO("All Initialized");
    //debug publisher
    plan_wave();
    plan_point_at_screen();
    plan_celebrate();
    plan_laugh_gesture();
	//laugh_gesture();


}

BingoGameFullServer::~BingoGameFullServer(void) {
    //destructor
}

bool BingoGameFullServer::init()
{
    demoNumberHandler.reset_pool();
    jokeHandler.read_file(jokesFileLocation);
    introHandler.read_file(introFileLocation);
    outroHandler.read_file(outroFileLocation);
    smallTalkHandler.read_file(smallTalkFileLocation);
    introDemoHandler.read_file(demoIntroLocation);


	//demo
    demoNumberHandler.read_file(demoprogressFileLocation);
#ifdef DEMO
    numberHandler.read_file(demoprogressFileLocation);
#else
    numberHandler.read_file(progressFileLocation);
#endif
    helpHandler.read_file(helpTextFileLocation);

}
bool BingoGameFullServer::delete_progress(){
 std::ofstream ofs;
 ofs.open(progressFileLocation.c_str(), std::ofstream::out | std::ofstream::trunc);
 ofs.close();

    jokeHandler.delete_progress();
    introHandler.delete_progress();
    outroHandler.delete_progress();
    smallTalkHandler.delete_progress();
    introDemoHandler.delete_progress();
    demoHelloHandler.delete_progress();
    demoNumberHandler.delete_progress();
    numberHandler.delete_progress();
    helpHandler.delete_progress();
    ROS_INFO("Progress deleted!");
}

void BingoGameFullServer::executeCB(const bingo_game::BingoGameGoal::ConstPtr &goal) {
    reset_head_pos();

    if(goal->start==0){
        ROS_INFO("Starting a new game!");
sleep(15);
        end_game=false;
        reset_head_pos();
        // actionlib::SimpleActionClient<bingo_game::BingoGameAction> ac("bingo_game", true);


        delete_progress();
        init();
        if(!game_began){
            give_intro_speech();
            game_began=true;
        }else{
            say_and_display("Let's play another game of Bingo!");
	    say_and_display("Can you please clear your Bingo cards?");
	    sleep(20);
        }

        play_Bingo();
    }else if(goal->start ==1){
        ROS_INFO("Continuing Bingo game!");
        end_game=false;
        reset_head_pos();
        play_Bingo();
    }else if(goal->start==2){
        ROS_INFO("Stopping Bingo games!");
        end_bingo_game();
    }else if(goal->start == 3)
    {
//	look_at_face();


	ROS_INFO("Starting Demo Game");
        reset_head_pos();
        end_game=false;
	delete_progress();
        init();
	//look_at_face();

	
      //give_demo_intro_speech();
           
       play_Bingo_Demo();
	
    }else if(goal->start == 4)
    {
	    demoHelloHandler.read_file(deomHelloLocation);
    debug_print("Giving Demo Hello speech");
    gs_msg.data="Giving Demo Hello speech";
    game_state_pub.publish(gs_msg);
    wave();

    vector<string> curr_line=demoHelloHandler.get_next_line();
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
        curr_line=demoHelloHandler.get_next_line();

    }
}else if(goal->start==5){
plan_point_at_screen();
sleep(20);
    ROS_INFO("Running some script");
    std::ifstream infile;
    infile.open("/home/tangy/Results.txt");
    std::string line;
/*
give_demo_intro_speech();
sleep(25);


give_demo_intro_speech();
sleep(25);

say_and_display("joke joke joke ", "Did you hear the joke about the butter?");
sleep(5);
say_and_display("joke joke joke ", "Did you hear the joke about the butter?");
sleep(5);
say_and_display("joke joke joke ", "Did you hear the joke about the butter?");
sleep(5);

point_at_screen();
say_and_display("joke joke joke ", "B-11");
sleep(5);
point_at_screen();
say_and_display("joke joke joke ", "B-11");
sleep(5);
point_at_screen();
say_and_display("joke joke joke ", "B-11");
sleep(5);

plan_wave();
wave();
say_and_display("bye bye bye", "Goodbye!");
sleep(15);
wave();
say_and_display("bye bye bye", "Goodbye!");
sleep(15);
wave();
say_and_display("bye bye bye", "Goodbye!");
sleep(15);


plan_point_at_screen();

point_at_screen();
say_and_display("In the bee column, can you remove: ", "Please mark the following number: B-11");
sleep(10);
point_at_screen();
say_and_display("In the bee column, can you remove: ", "Please mark the following number: B-11");
sleep(10);
point_at_screen();
say_and_display("In the bee column, can you remove: ", "Please mark the following number: B-11");
sleep(10);

*/
sleep(3);
  look_at_card_3();

say_and_display("Good Job!", "Good Job!");
sleep(20);
/*
plan_celebrate();
say_and_display("Yo Yo Yo Yo Yo Yo Yo Yo Yo Jacob jacob jacob jacob jacob bacon bacon bacon bacon", "BINGO!!!");
celebrate();
sleep(10);

plan_point_at_screen();
point_at_screen();
say_and_display("In the bee column, can you remove: ", "Please remove the following number: N-40");
sleep(10);
point_at_screen();
say_and_display("In the bee column, can you remove: ", "Please remove the following number: N-40");
sleep(10);
point_at_screen();
say_and_display("In the bee column, can you remove: ", "Please remove the following number: N-40");
sleep(10);



say_and_display("Yo Yo Yo Yo Yo Yo Yo Yo Yo Jacob jacob jacob jacob jacob bacon bacon bacon bacon", "Great Job!");
sleep(10);

say_and_display("Yo Yo Yo Yo Yo Yo Yo Yo Yo Jacob jacob jacob jacob jacob bacon bacon bacon bacon", "Please move your card closer");
sleep(10);

say_and_display("Yo Yo Yo Yo Yo Yo Yo Yo Yo Jacob jacob jacob jacob jacob bacon bacon bacon bacon", "BINGO!!!");
celebrate();
sleep(10);
	
    while(std::getline(infile, line))
    {
        std::istringstream inputline(line);
if(strncmp(line.c_str(),"GREETING",8)==0)
		{
			cout<<"greets";
			give_intro_speech();
			 
		}
		else if(strncmp(line.c_str(),"CALLING", 7)==0)
		{
			cout<<"calls";
			string num=numberHandler.get_rand_num();
			string num_text=num.substr(0,1)+"."+num.substr(2);
			say_and_display(num_text,num);
			 
		}
		else if(strncmp(line.c_str(),"JOKE",4)==0)
		{
			cout<<"so funny";
			tell_joke();
			 
		}
		else if(strncmp(line.c_str(),"NAVIGATING_TO_HELP",18)==0)
		{
			cout<<"get out the way";
			move_straight_no_rotate(0.1);
			 
		}
		else if(strncmp(line.c_str(),"REMOVEMARKER",12)==0)
		{
			cout<<"remover";
			string wrong_str=numberHandler.get_rand_num();
			say_and_display("In the bee column, can you remove: ", wrong_str);
		}
		else if(strncmp(line.c_str(),"MARKNUMBER",10)==0)
		{
			cout<<"marknombre";
			string num=numberHandler.get_rand_num();
			say_and_display("In the bee column, can you remove: ", num);
			 
		}
		else if(strncmp(line.c_str(),"ENCOURAGE",9)==0)
		{
			cout<<"encourager";
			string praise=helpHandler.get_praise();
  			say_and_display("You are doing a great job! keep up the great work", "GREAT JOB!");
  			string encouragement=helpHandler.get_encouragement();

		}
		else if(strncmp(line.c_str(),"NAVIGATING_TO_FRONT",19)==0)
		{
			cout<<"to da front";
			move_straight_no_rotate(-0.1);
			 
		}
		else if(strncmp(line.c_str(),"CELEBRATE",9)==0)
		{
			cout<<"woo";
			celebrate();
		}
		else if(strncmp(line.c_str(),"VALEDICTION",11)==0)
		{
			cout<<"valedicter";
			say_and_display("goodbye");
		}
		cout<<endl;

		sleep(2);
    }
*/
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bingo_game");
    ros::NodeHandle n;
    srand (time (NULL));
    BingoGameFullServer bingo(ros::this_node::getName());
    ros::Subscriber pause_sub=n.subscribe<std_msgs::String>("control_command",10, &BingoGameFullServer::pauseCallback, &bingo);
    ROS_INFO("[%s] Bingo Game server is running" , ros::this_node::getName().c_str());

    //bingo.check_bingo_card();
    /*********For Pictures*******/
    //bingo.reset_head_pos();
    /*
    while(1){
        int action = -1;
        std::cin >> action;
        if(action == 0)
            bingo.give_intro_speech();
        if(action == 1)
            bingo.tell_joke();
        if(action == 2){
            bingo. display_text("B-5");
            bingo.point_at_screen();
        }
        if(action==3){
			bingo. display_text("Please remove the marker on O-72 ");
        }
        if(action==4){
            bingo. display_text("Please move your card closer");
        }
        if(action==5){
            bingo. display_text("Please mark N-36");
            bingo.point_at_screen();
        }
        if(action==6){
            bingo. display_text("Bingo");
        }if(action==7){
            bingo. display_text("Bingo!!!");
            bingo.celebrate();
        }
    }
 	*/   
    ros::spin();
    return 0;
}
