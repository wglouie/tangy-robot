#include <BingoGameFullServer/BingoGameFullServer.h>

/*Intro speech interaction. Each line in the speech is separated into aural and display text
in the format of {speech, text}*/
void BingoGameFullServer::give_intro_speech() {
	debug_print("Giving intro speech");
	state_pub("Interaction");
	wave();

	vector<string> curr_line=introHandler.get_next_line();
	string last_displayed_line = "";
	int line_count = 0;
	while(!curr_line.empty()) {
		line_count++;

		if(last_displayed_line.compare(curr_line.at(1)) != 0) {
			say_and_display(curr_line.at(0), curr_line.at(1));
			last_displayed_line = curr_line.at(1);
		}else {
			say_and_display(curr_line.at(0), last_displayed_line);
		}
		curr_line = introHandler.get_next_line();

	}
}

void BingoGameFullServer::give_demo_intro_speech() {
    debug_print("Giving Demo intro speech");
    gs_msg.data = "Giving Demo intro speech";
    game_state_pub.publish(gs_msg);
    wave();

    vector<string> curr_line = introDemoHandler.get_next_line();
    string last_displayed_line = "";
    int line_count = 0;
    while(!curr_line.empty()) {

        line_count++;

        if(last_displayed_line.compare(curr_line.at(1)) != 0) {
            say_and_display(curr_line.at(0), curr_line.at(1));
            last_displayed_line=curr_line.at(1);
        }else {
            say_and_display(curr_line.at(0), last_displayed_line);
        }
        curr_line = introDemoHandler.get_next_line();

    }
}

/*Outro speech interaction. Each line in the speech is separated into aural and display text
in the format of {speech, text}*/
void BingoGameFullServer::give_outro_speech() {
	debug_print("Giving outro speech");
	state_pub("Interaction");
	vector<string> curr_line = outroHandler.get_next_line();
	string last_displayed_line = "";
	int line_count = 3;
	while(!curr_line.empty()) {
		line_count--;

		if(line_count == 0) {
			wave();
		}

		if(last_displayed_line.compare(curr_line.at(1)) != 0) {

			say_and_display(curr_line.at(0), curr_line.at(1));
			last_displayed_line = curr_line.at(1);
		} else{
			say_and_display(curr_line.at(0), last_displayed_line);
		}
		curr_line=outroHandler.get_next_line();
	}
}

/*Play the Bingo game -- call out numbers, check for whether somebody needs help
and inject various jokes, gestures and small talk.
While numberHandler isn't returning an empty string, loop through the while loop
and call random bingo numbers. On every fifth and ninth number, do a joke, smallTalk
or a gesture.*/
void BingoGameFullServer::play_Bingo(){

	ready_to_nod();
 	begin_looking_for_help();
	int count = 1;
		
	//Initialize music
	set_music_volume(LOUD);
	play_music("normal");
	
	//Call First number
	string num = numberHandler.get_rand_num();
	std::string num_text = num.substr(0,1) + "." + num.substr(2);
	state_pub("Calling number");
	say_and_display("The first number is:");
	say_and_display(num_text,num);
	say_and_display(num_text,num);
	count++;
	num = numberHandler.get_rand_num();

	//Looping game actions
	while(num.size() != 0 && end_game == false){ //TODO see if we need to put update behaviours for each speech event
	
		//state_pub("Currently playing Bingo");
		num_text = num.substr(0, 1) + "." + num.substr(2);
	  
		if(pause_game){
			pause_everything();
		}
		rand_head_pos();
		int rand_n = rand()%100;
		state_pub("Calling number"); //TODO be aware the robot does not always say the number -> for voice monitor
		if(rand_n < 33){
			say_and_display("The next number is:");
  			if(count % 2 == 0 && !determine_help()){ //TODO this is repeated 3 times, can it be pulled out of the ifs?
  				point_at_screen(); //TODO behaviour moving arm?
  			}
			say_and_display(num_text, num);
			rand_head_pos(); //TODO behaviour moving neck?
		} else if(rand_n < 60){
			display_text("The next number is:");
  			if(count % 2 == 0 && !determine_help()){
  				point_at_screen();
  			}
			say_and_display(num_text, num);
			say_and_display("One more time.", num);
			rand_head_pos();
		} else{
			display_text("The next number is:");
  			if(count % 2 == 0 && !determine_help()){
  				point_at_screen();
  			}
			say_and_display(num_text, num);
			rand_head_pos();
		}
		say_and_display(num_text, num);

		if(count % 7 == 0){
			tell_joke();
		}

		if(count % 9 == 0){ //TODO see what is going to happen here
			plan_point_at_screen();
			plan_wave();
			plan_celebrate();
			plan_laugh_gesture();
		}
		not_ready_to_nod();
		do_help(); //TODO Why three times?
		do_help();
		do_help();
		ready_to_nod();
		count++;
		num = numberHandler.get_rand_num();
	} //End while loop

	bool helped = false;
	while (!helped && !end_game){
		//TODO some sort of delaying end game state?
		display_text("All the numbers have been called!");
		do_help();
	}
	stop_music();
	stop_looking_for_help();
	as_.setSucceeded();
	
}

//Bingo Game Demo -> Tangy is stationary and calls out numbers and after 6 numbers checks the users card

void BingoGameFullServer::play_Bingo_Demo() {

    int count=1;

    //Initialize music
    set_music_volume(LOUD);
    play_music("normal");
    gs_msg.data="Beginning playing Demo Bingo";
    game_state_pub.publish(gs_msg);

    //Call First number
    string num=demoNumberHandler.get_rand_num();
    std::string num_text=num.substr(0,1)+"."+num.substr(2);

    say_and_display("The first number is:");
    say_and_display(num_text,num);
    say_and_display(num_text,num);
    count++;
    num=demoNumberHandler.get_rand_num();

    //Looping game actions
    while(num.size()!=0 && end_game==false){

        gs_msg.data="Currently playing Demo Bingo";
        game_state_pub.publish(gs_msg);
            num_text=num.substr(0,1)+"."+num.substr(2);

        if(pause_game){
            pause_everything();
        }
        int rand_n=rand()%100;
        if(rand_n<33){
            say_and_display("The next number is:");
        if(count % 2==0 && !determine_help()){
            point_at_screen();
        }
            say_and_display(num_text,num);
        } else if(rand_n<60){
            display_text("The next number is:");
        if(count % 2==0 && !determine_help()){
            point_at_screen();
        }
            say_and_display(num_text,num);
            say_and_display("One more time.", num);
        } else{
            display_text("The next number is:");
        if(count % 2==0 && !determine_help()){
            point_at_screen();
        }
            say_and_display(num_text,num);
        }
        say_and_display(num_text,num);

        if(count % 4==0){
            //if(rand()%100<66){
                tell_joke();
            /*}else{
                make_small_talk();
                // tell_joke();
            }*/
        }

        if(count % 9==0){
            plan_point_at_screen();
            plan_wave();
            plan_celebrate();
        }

        if(count % 6==0){
          	std::string name = "";
            //name=detect_face();
            demo_help(name,0,0);
		
            end_game=true;
        }
    
    	  count++;
    	  num=demoNumberHandler.get_rand_num();
    }
    bool helped=false;
    while (!helped&&!end_game){
      display_text("All the numbers have been called!");
      //do_help();
    }

    say_and_display("Well that was fun. I hope you had a good time playing with me.", "BINGO");
    stop_music();
    as_.setSucceeded();
}
    
void BingoGameFullServer::end_bingo_game() {
	debug_print("Ending Bingo Game");
	state_pub("Ending game");
	reset_head_pos();
	end_game=true;
	give_outro_speech();
	as_.setSucceeded();
}

void BingoGameFullServer::state_pub(std::string state){
	gs_msg.data = state;
	game_state_pub.publish(gs_msg);
}
