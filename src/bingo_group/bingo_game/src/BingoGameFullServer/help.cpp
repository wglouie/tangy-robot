#include <BingoGameFullServer/BingoGameFullServer.h>

void BingoGameFullServer::begin_looking_for_help(){
    ROS_INFO("Scanning for help requests...");
    go_pub.publish(go_msg);
}

void BingoGameFullServer::stop_looking_for_help(){
    ROS_INFO("Stopping scanning for help requests.");
    go_pub.publish(stop_msg);
}

std::vector<help_indicators::triangle> BingoGameFullServer::get_help_requests(){
    // check if theres any help_indicators
    help_indicators::get_help_indicators srv;
    std::vector<help_indicators::triangle> help_indicators;
    srv.request.str = "Anything";
    try {
        client.call(srv);
        help_indicators = srv.response.help_indicators;
    } catch (std::string error) {
        ROS_ERROR("Failed to call service: get_help_indicators, %s!", error.c_str());
    }
    return help_indicators;
}

void BingoGameFullServer::clear_request(std::string name){
    if(!name.empty()){
        ROS_INFO("Finished helping [%s]. Sending request to remove from queue...", name.c_str());
        std_msgs::String clear_msg;
        clear_msg.data = name;
        clear_pub.publish(clear_msg);
    }else{
        ROS_WARN("Cannot clear an empty name!");
    }
}

/*Help interaction--including all parts of the help interaction, including the movement
    to and from the person in need of help.*/


bool BingoGameFullServer::determine_help(){
    //Check if help required
    state_pub("Determining help");
    std::vector<help_indicators::triangle> indicators = get_help_requests();
    ROS_INFO("There are [%d] people I have to help!", (int) indicators.size());
    if(!indicators.empty()){
        return true;
    }
    return false;
}

void BingoGameFullServer::do_help(){
    state_pub("Determining help");
    std::vector<help_indicators::triangle> indicators = get_help_requests();
    ROS_INFO("There are [%d] people I have to help!", (int) indicators.size());
    if(!indicators.empty()){
        while((int) indicators.size()>0){
            say_and_display("Please give me a second to help someone.", "Coming to Help...");
            string name = "";

            state_pub("Moving");
            look_at_face();
            // face(indicators[0].x, indicators[0].y);
            move_to_help(indicators[0].x, indicators[0].y);
            set_music_volume(QUIET);
            // rotate_to_face();
            // look_down();
            reset_head_pos();
            //name=detect_face();
            stop_look_at_face();
            help(name, indicators[0].x, indicators[0].y);
            reset_num_times_helped();
            reset_num_times_failed();
            clear_request(indicators[0].name);
            //sleep(1);
            indicators = get_help_requests();
            ROS_INFO("There are [%d] people I still need to help!", (int) indicators.size());

            if(end_game==false){
                say_and_display("If you need any more help press your button.", "If you need any more help press your button");
            }
            //stop_look_at_face();
            if(end_game){
                move_straight_no_rotate(-0.35);
            }else{
                move_straight_no_rotate(-0.5);
            }

            set_music_volume(LOUD);
        }
        debug_print("Moving back to origin");
        state_pub("Moving");
        move_to_orig_pos();
    }
}

void BingoGameFullServer::help(std::string name, float p_x, float p_y){

    int good_attempt_lim=3;
    int bad_attempt_lim=3;

    //Check to see if game is paused
    if(pause_game){
        pause_everything();
    }
    //ros::spinOnce();
    //look_down();
    look_at_face();
    //First contact with a person
    if((num_times_helped + num_times_failed) == 0){
        if(!name.empty()){
            say_and_display("Hi " + name, "Hi!");
        }
        say_and_display(helpHandler.get_initialHelp(), "Checking Bingo Card");
    } else if (num_times_failed < (bad_attempt_lim - 1)){
        say_and_display("Let me check your card again, "+name, "Checking Bingo Card");
    } else{
        say_and_display(name+", I'm going to give this one last try.", "Checking Bingo Card");
    }
    //ros::spinOnce();
    //rotate_to_face();
    //display_text("Checking Bingo Card");
    //stop_look_at_face();
    state_pub("Checking card");
    //Look at the bingo card, and determine what state it's in
    stop_look_at_face();
    check_bingo_card();
    //sleep(5);
    //Check to see if game is paused
    look_at_face();
    if(pause_game){
        pause_everything();
    }
    //look_down();
    //ros::spinOnce();
    //Begin giving help for detected bingo card
    state_pub("Giving help");
    string gameState = bingoDetectionClient.get_GameState();

    if(gameState.compare("error")==0){
        num_times_failed++;
        ROS_INFO("Bingo game: error state.");
        //look_at_face();
        say_and_display("I'm sorry. I cannot reed your card "+name+ ". Please move it slightly and make sure nothing is blocking it", "Please Move Card");
        //look_down();
        stop_look_at_face();
        if(num_times_failed<bad_attempt_lim && num_times_helped<good_attempt_lim){
            help(name, p_x, p_y);
        }

    } else if(gameState.compare("bingo")==0){
        num_times_failed=0;
        num_times_helped++;
        //look_at_face();
        ROS_INFO("Bingo game: winning state.");
        stop_music();
        sleep(1);
        play_music("celebrate");
        move_straight_no_rotate(-0.35);
        // face(p_x+1, p_y);
        rotate("map", 0);
        celebrate();
        say_and_display("Congratulations "+name+ "! You have won Bingo!", "BINGO!!!!!!!!");
        end_game=true;
        sleep(20);
        stop_music();
        stop_look_at_face();

    } else if(gameState.compare("good card")==0){
        num_times_failed=0;
        num_times_helped++;
        //look_at_face();
        ROS_INFO("Bingo game: good card state.");
        string praise=helpHandler.get_praise();
        string encouragement=helpHandler.get_encouragement();

        say_and_display(name+" "+praise+" "+encouragement, "GREAT JOB!");
        stop_look_at_face();

    }else if(gameState.compare("bad card")==0){
        num_times_failed=0;
        num_times_helped++;
        //look_at_face();
        ROS_INFO("Bingo game: bad card state.");
        vector<string> wrongNumbers=bingoDetectionClient.get_WrongNumbers();
        ROS_INFO("There are %d wrong numbers.", (int) wrongNumbers.size());
        vector<string> missingNumbers=bingoDetectionClient.get_MissingNumbers();
        ROS_INFO("There are %d missing numbers.", (int) missingNumbers.size());
        state_pub("Request changes");

        if(!wrongNumbers.empty()){
            move_straight_no_rotate(-0.175);
            rotate("map", 0);
            //face(p_x+1, p_y);
            say_and_display(helpHandler.get_wrongNumberHelp()+" "+name, "OOPS! Misplaced Markers!");
            point_at_screen();
            string wrong_str="Remove markers from: ";

            if(!wrongNumbers.empty()){
                wrong_str=wrong_str+wrongNumbers.at(0);
                for(int i=1; i<(int) wrongNumbers.size(); i++){
                    wrong_str=wrong_str+", "+wrongNumbers.at(i);
                }
            }

            bool b=false;
            bool i=false;
            bool n=false;
            bool g=false;
            bool o=false;
            for(int j=0; j<(int) wrongNumbers.size(); j++){
                if((wrongNumbers.at(j)).at(0)=='B'&& b==false){
                    say_and_display("In the bee column, can you remove: ", wrong_str);
                    b=true;
                }else if((wrongNumbers.at(j)).at(0)=='I'&& i==false){
                    i=true;
                    say_and_display("In the I column, can you remove: ", wrong_str);
                }else if((wrongNumbers.at(j)).at(0)=='N'&& n==false){
                    n=true;
                    say_and_display("In the en column, can you remove: ", wrong_str);
                }else if((wrongNumbers.at(j)).at(0)=='G'&& g==false){
                    g=true;
                    say_and_display("In the gee column, can you remove: ", wrong_str);
                }else if((wrongNumbers.at(j)).at(0)=='O'&& o==false){
                    o=true;
                    say_and_display("In the oh column, can you remove: ", wrong_str);
                }
                say_and_display(wrongNumbers.at(j), wrong_str);
            }
            if(missingNumbers.empty()){
                move_to_help(p_x,p_y);
            }

        }

        if(!missingNumbers.empty()){
            if(wrongNumbers.empty()){
                move_straight_no_rotate(-0.175);
                face(p_x+1,p_y);
            }

            say_and_display(name+" "+helpHandler.get_missingNumberHelp(), "OOPS! Missing Markers!");
            point_at_screen();
            string missing_str="Place markers on: ";
            if(!missingNumbers.empty()){
                missing_str=missing_str+missingNumbers.at(0);
                for(int i=1; i<(int) missingNumbers.size(); i++){
                    missing_str=missing_str+", "+missingNumbers.at(i);
                }
            }
            bool b=false;
            bool i=false;
            bool n=false;
            bool g=false;
            bool o=false;
            for(int j=0; j<(int) missingNumbers.size(); j++){
                if((missingNumbers.at(j)).at(0)=='B'&& b==false){
                    say_and_display("In the bee column, can you put: ", missing_str);
                    b=true;
                }else if((missingNumbers.at(j)).at(0)=='I'&& i==false){
                    i=true;
                    say_and_display("In the I column, can you put: ", missing_str);
                }else if((missingNumbers.at(j)).at(0)=='N'&& n==false){
                    n=true;
                    say_and_display("In the en column, can you put: ", missing_str);
                }else if((missingNumbers.at(j)).at(0)=='G'&& g==false){
                    g=true;
                    say_and_display("In the gee column, can you put: ", missing_str);
                }else if((missingNumbers.at(j)).at(0)=='O'&& o==false){
                    o=true;
                    say_and_display("In the oh column, can you put: ", missing_str);
                }
                say_and_display(missingNumbers.at(j), missing_str);
            }

            move_to_help(p_x, p_y);
        }
        stop_look_at_face();
        if(num_times_failed<bad_attempt_lim && num_times_helped<good_attempt_lim){
            help(name, p_x, p_y);
        }

    } else{
        num_times_failed++;
        ROS_ERROR("Game State unknown--bingo search client did not execute!");
        //look_at_face();
        say_and_display("Oh No. I cannot reed your card. Please move it slightly and make sure nothing is covering it", "Please Move Card");
        // look_down();
        stop_look_at_face();
        if(num_times_failed<bad_attempt_lim && num_times_helped<good_attempt_lim){
            help(name, p_x, p_y);
        }
    }
    //End help interaction
    //sleep(2);
}

void BingoGameFullServer::demo_help(std::string name, float p_x, float p_y){

//Infite helping for demo
    int good_attempt_lim=300;
    int bad_attempt_lim=300;

    //Check to see if game is paused
    if(pause_game){
        pause_everything();
    }
    //ros::spinOnce();
    // look_down();
    look_at_face();
    //First contact with a person
    if((num_times_helped+num_times_failed)==0){
        if(!name.empty()){
            say_and_display("Hi "+name, "Hi!");
        }
        say_and_display(helpHandler.get_initialHelp(),"Checking Bingo Card");
    } else if (num_times_failed<(bad_attempt_lim-1)){
        say_and_display("Let me check your card again, "+name, "Checking Bingo Card");
    } else{
        say_and_display(name+", I'm going to give this one last try.", "Checking Bingo Card");
    }
    //ros::spinOnce();
    //rotate_to_face();

    // 	display_text("Checking Bingo Card");

    gs_msg.data="Checking bingo card";
    game_state_pub.publish(gs_msg);
    stop_look_at_face();
    //Look at the bingo card, and determine what state it's in
    check_bingo_card();
    //sleep(5);
    look_at_face();
    gs_msg.data="Finished checking bingo card. About to give help.";
    game_state_pub.publish(gs_msg);
    //Check to see if game is paused
    if(pause_game){
        pause_everything();
    }
    // 	look_down();
    //ros::spinOnce();
    //Begin giving help for detected bingo card
    string gameState=bingoDetectionClient.get_GameState();

    if(gameState.compare("error")==0){
        num_times_failed++;
        ROS_INFO("Bingo game: error state.");
        //look_at_face();
        say_and_display("I'm sorry. I cannot reed your card "+name+ ". Please move it slightly and make sure nothing is blocking it", "Please Move Card");
        // look_down();
        stop_look_at_face();
        if(num_times_failed<bad_attempt_lim && num_times_helped<good_attempt_lim){
            demo_help(name, p_x, p_y);
        }

    } else if(gameState.compare("bingo")==0){
        num_times_failed=0;
        num_times_helped=0;
        //look_at_face();
        ROS_INFO("Bingo game: winning state.");
        stop_music();
        sleep(1);
        play_music("celebrate");
        //move_straight_no_rotate(-0.35);
        // face(p_x+1, p_y);
        rotate("map", 0);
        celebrate();
        say_and_display("Congratulations "+name+ "! You have won Bingo!", "BINGO!!!!!!!!");
        end_game=true;
        sleep(20);
        stop_music();
        stop_look_at_face();

    } else if(gameState.compare("good card")==0){
        num_times_failed=0;
        num_times_helped=0;
        //look_at_face();
        ROS_INFO("Bingo game: good card state.");
        string praise=helpHandler.get_praise();
        string encouragement=helpHandler.get_encouragement();

        say_and_display(name+" "+praise+" "+encouragement, "GREAT JOB!");
        stop_look_at_face();

    }else if(gameState.compare("bad card")==0){
        num_times_failed=0;
        num_times_helped=0;
        //look_at_face();
        ROS_INFO("Bingo game: bad card state.");
        vector<string> wrongNumbers=bingoDetectionClient.get_WrongNumbers();
        ROS_INFO("There are %d wrong numbers.", (int) wrongNumbers.size());
        vector<string> missingNumbers=bingoDetectionClient.get_MissingNumbers();
        ROS_INFO("There are %d missing numbers.", (int) missingNumbers.size());

        if(!wrongNumbers.empty()){
           //move_straight_no_rotate(-0.175);
            rotate("map", 0);
            //face(p_x+1, p_y);
            say_and_display(helpHandler.get_wrongNumberHelp()+" "+name, "OOPS! Misplaced Markers!");
            point_at_screen();
            string wrong_str="Remove markers from: ";

            if(!wrongNumbers.empty()){
                wrong_str=wrong_str+wrongNumbers.at(0);
                for(int i=1; i<(int) wrongNumbers.size(); i++){
                    wrong_str=wrong_str+", "+wrongNumbers.at(i);
                }
            }

            bool b=false;
            bool i=false;
            bool n=false;
            bool g=false;
            bool o=false;
            for(int j=0; j<(int) wrongNumbers.size(); j++){
                if((wrongNumbers.at(j)).at(0)=='B'&& b==false){
                    say_and_display("In the bee column, can you remove: ", wrong_str);
                    b=true;
                }else if((wrongNumbers.at(j)).at(0)=='I'&& i==false){
                    i=true;
                    say_and_display("In the I column, can you remove: ", wrong_str);
                }else if((wrongNumbers.at(j)).at(0)=='N'&& n==false){
                    n=true;
                    say_and_display("In the en column, can you remove: ", wrong_str);
                }else if((wrongNumbers.at(j)).at(0)=='G'&& g==false){
                    g=true;
                    say_and_display("In the gee column, can you remove: ", wrong_str);
                }else if((wrongNumbers.at(j)).at(0)=='O'&& o==false){
                    o=true;
                    say_and_display("In the oh column, can you remove: ", wrong_str);
                }
                say_and_display(wrongNumbers.at(j), wrong_str);
            }
            if(missingNumbers.empty()){
                //move_to_help(p_x,p_y);
            }

        }

        if(!missingNumbers.empty()){
            if(wrongNumbers.empty()){
                //move_straight_no_rotate(-0.175);
                //face(p_x+1,p_y);
            }

            say_and_display(name+" "+helpHandler.get_missingNumberHelp(), "OOPS! Missing Markers!");
            point_at_screen();
            string missing_str="Place markers on: ";
            if(!missingNumbers.empty()){
                missing_str=missing_str+missingNumbers.at(0);
                for(int i=1; i<(int) missingNumbers.size(); i++){
                    missing_str=missing_str+", "+missingNumbers.at(i);
                }
            }
            bool b=false;
            bool i=false;
            bool n=false;
            bool g=false;
            bool o=false;
            for(int j=0; j<(int) missingNumbers.size(); j++){
                if((missingNumbers.at(j)).at(0)=='B'&& b==false){
                    say_and_display("In the bee column, can you put: ", missing_str);
                    b=true;
                }else if((missingNumbers.at(j)).at(0)=='I'&& i==false){
                    i=true;
                    say_and_display("In the I column, can you put: ", missing_str);
                }else if((missingNumbers.at(j)).at(0)=='N'&& n==false){
                    n=true;
                    say_and_display("In the en column, can you put: ", missing_str);
                }else if((missingNumbers.at(j)).at(0)=='G'&& g==false){
                    g=true;
                    say_and_display("In the gee column, can you put: ", missing_str);
                }else if((missingNumbers.at(j)).at(0)=='O'&& o==false){
                    o=true;
                    say_and_display("In the oh column, can you put: ", missing_str);
                }
                say_and_display(missingNumbers.at(j), missing_str);
            }

            //move_to_help(p_x, p_y);
        }
        stop_look_at_face();
        if(num_times_failed<bad_attempt_lim && num_times_helped<good_attempt_lim){
            demo_help(name, p_x, p_y);
        }

    } else{
        num_times_failed++;
        ROS_ERROR("Game State unknown--bingo search client did not execute!");
        //look_at_face();
        say_and_display("Oh No. I cannot reed your card. Please move it slightly and make sure nothing is covering it", "Please Move Card");
        // look_down();
        stop_look_at_face();
        if(num_times_failed<bad_attempt_lim && num_times_helped<good_attempt_lim){
            demo_help(name, p_x, p_y);
        }
    }

    //End help interaction
    //sleep(2);
}

void BingoGameFullServer::reset_num_times_helped(){
    num_times_helped=0;
    num_times_failed=0;
}
void BingoGameFullServer::reset_num_times_failed(){
    num_times_failed=0;
}
