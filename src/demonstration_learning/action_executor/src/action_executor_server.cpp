#include <action_executor/action_executor_server.h>
#include <boost/lexical_cast.hpp>

// Initialize our action executor server
ActionExecutorServer::ActionExecutorServer(std::string name, float wait_time):
    as_(nh_,name, boost::bind(&ActionExecutorServer::goalCB, this, _1), false),
    tangy(wait_time),
    number_handler(),
    performing_gesture(false),
    trajectory_executor_client(name, wait_time),
    text_to_speech_client(name, wait_time){

    as_.start();
    ROS_INFO("[%s] Action Executor server is running" , ros::this_node::getName().c_str());

    nh_.getParam("/progress_file", progress_file_location);
    nh_.getParam("/default_gesture_mode", default_gesture_mode);
    nh_.getParam("/nav_goal_tolerence", nav_goal_tolerence);
    nh_.getParam("/default_action_file", default_action_database);
    populate_action_list(default_action_database);

    //Subscribers and services
    //exec_traj_pub = nh_.advertise<std_msgs::String>("robot_ik/execute_file",1000);
    called_numbers_pub = nh_.advertise<std_msgs::Int32MultiArray>("action_executor/called_numbers", 1000);
    notify_help_start_pub = nh_.advertise<std_msgs::String>("action_executor/notify_help_start", 1000);
    notify_help_start_sub = nh_.subscribe("action_executor/notify_help_start", 1000,&ActionExecutorServer::help_start_callback,this);
    notify_help_stop_sub = nh_.subscribe("rqt_task_learning/notify_help_stop", 1000, &ActionExecutorServer::help_stop,this);
    missing_numbers_sub = nh_.subscribe("world_state_identifier/missing_numbers", 1000, &ActionExecutorServer::missing_number_callback,this);
    incorrect_numbers_sub = nh_.subscribe("world_state_identifier/incorrect_numbers", 1000, &ActionExecutorServer::incorrect_number_callback, this);
    learning_state_sub = nh_.subscribe("rqt_task_learning/learning_state", 1000, &ActionExecutorServer::learning_state_callback, this);
    move_straight_sub = nh_.subscribe("action_executor/move_straight_cmd", 1000, &ActionExecutorServer::move_straight_cmd_cb,this);
    action_list_srv = nh_.advertiseService("action_executor/get_action_list", &ActionExecutorServer::get_action_list_srv, this);
    last_action_srv = nh_.advertiseService("action_executor/get_last_executed_action", &ActionExecutorServer::get_last_executed_action_srv, this);
    exe_spch_and_gest_srv = nh_.advertiseService("action_executor/exe_spch_and_gest", &ActionExecutorServer::exe_spch_and_gest_srv_cb, this);
    preview_customization_srv = nh_.advertiseService("action_executor/preview_customization", &ActionExecutorServer::preview_customization_cb, this);
    init_database_srv = nh_.advertiseService("action_executor/init_database", &ActionExecutorServer::init_database_cb, this);
    customize_action_sub = nh_.subscribe("action_executor/customize_action", 1000, &ActionExecutorServer::customize_action_cb, this);
    save_custom_database_sub = nh_.subscribe("action_executor/save_database", 1000, &ActionExecutorServer::save_action_database_cb, this);
    get_world_state_client = nh_.serviceClient<demonstration_learning_msgs::get_current_world_state>("world_state_identifier/get_current_world_state");
    exec_gesture_file_client = nh_.serviceClient<robot_inverse_kinematics::execute_gesture_file>("robot_ik/execute_file");
}

// Destructor
ActionExecutorServer::~ActionExecutorServer(){

}
void ActionExecutorServer::learning_state_callback(const std_msgs::String::ConstPtr& msg){
    std::string learning_state = msg->data;
    if(learning_state.compare("new game") == 0){
        populate_action_list(default_action_database);
        number_handler.delete_progress(progress_file_location);
    } else if (learning_state.compare("continue game") == 0){
        std::string temp_path = ros::package::getPath("action_executor") + "/database/temp_database_actions.xml";
        populate_action_list(temp_path);
    } else if (learning_state.compare("run policy") == 0){
        populate_action_list(default_action_database);
    }
    number_handler.read_file(progress_file_location);
    publish_called_numbers();
    last_executed_action = "None";
    helping_player = false;
}

void ActionExecutorServer::missing_number_callback(const demonstration_learning_msgs::bingo_numbers_string::ConstPtr& msg){
    last_missing_numbers = msg->bingo_numbers;
    ROS_INFO("Received Missing Numbers:");
    for(int i=0; i<last_missing_numbers.size(); i++){
        ROS_INFO_STREAM(last_missing_numbers[i]);
    }
}
void ActionExecutorServer::incorrect_number_callback(const demonstration_learning_msgs::bingo_numbers_string::ConstPtr& msg){
    last_incorrect_numbers = msg->bingo_numbers;
    ROS_INFO("Received Incorrect Numbers:");
    for(int i=0; i<last_incorrect_numbers.size(); i++){
        ROS_INFO_STREAM(last_incorrect_numbers[i]);
    }
}

void ActionExecutorServer::help_start_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Action executor received a help start message");
    std::string expected_string = "start help";
     if(expected_string.compare(msg->data) == 0){
        helping_player = true;
     } else {
         ROS_ERROR("Unexpected message published on notify_help_start topic.");
     }
}

void ActionExecutorServer::help_stop(const std_msgs::String::ConstPtr& msg){
    std::string expected_string = "stop help";
     if(expected_string.compare(msg->data) == 0){
        helping_player = false;
     } else {
         ROS_ERROR("Unexpected message published on notify_help_stop topic.");
     }
}

// Execute the action provided by the client
void ActionExecutorServer::goalCB(const action_executor::action_executorGoal::ConstPtr &goal){
    bool thread_created = false;
    std::string requested_action = goal->action_name;
    std::map<std::string,Action>::const_iterator iter = actions.find(requested_action);
    action_executor::action_executorActionResult result_;
    demonstration_learning_msgs::world_state world_state;
    world_state = get_world_state();

    if ( iter == actions.end() ) {
        ROS_ERROR("Do nothing: Action does not exist in the database.");
        result_.result.done = -1;
        as_.setAborted(result_.result,"Action does not exist in the databse");
        last_executed_action = "Failed to execute";
    } else {
        Action action_to_execute = iter->second;

        //Add number to end of call number action
        if(action_to_execute.name.compare("CALL NUMBER") == 0){
            action_to_execute.speech.append(" " + number_handler.get_rand_num());
            publish_called_numbers();
        } else if (action_to_execute.name.compare("REMOVEMARKER") == 0){
            try{
              if(last_incorrect_numbers.size() == 0){
                  throw 1;
              } else {
                  for(int i=0; i<last_incorrect_numbers.size(); i++){
                      action_to_execute.speech.append(" " + last_incorrect_numbers[i]);
                  }
              }
            } catch(int error){
                switch(error){
                    case 1:
                       ROS_ERROR("There are no incorrectly marked Bingo numbers on the players card or we have not detected a players card yet");
                       as_.setAborted(result_.result,"There are no incorrectly marked Bingo numbers on the players card or we have not detected a players card yet");
                       break;
                }
                return;
            }
        } else if (action_to_execute.name.compare("MARKNUMBER") == 0){
            try{
              if(last_missing_numbers.size() == 0){
                  throw 1;
              } else {
                  for(int i=0; i<last_missing_numbers.size(); i++){
                      action_to_execute.speech.append(" " + last_missing_numbers[i]);
                  }
              }
            } catch(int error){
                switch(error){
                    case 1:
                       ROS_ERROR("There are no missing Bingo numbers on the players card or we have not detected a players card yet");
                       as_.setAborted(result_.result,"There are no missing Bingo numbers on the players card or we have not detected a players card yet");
                       break;
                }
                return;
            }
        } else if(action_to_execute.name.compare("MOVE_TO_HELP") == 0){
            try{
              if(world_state.activity_state.compare("NULL") == 0){
                  throw 1;
              } else if(world_state.user_assistance_requests.size() == 0){
                  throw 2;
              } else if(helping_player == true){
                  throw 3;
              } else {
                  if(world_state.robot_state.compare("At front of room") != 0){
                    tangy.move_straight_no_rotate(-0.5);
     				        tangy.rotate("map",0);
                  }
                  ROS_INFO("Start helping user Player");
                  action_to_execute.location.x = world_state.user_assistance_requests[0].x - 0.30;
                  action_to_execute.location.y = world_state.user_assistance_requests[0].y;
                  std_msgs::String notify_help_msg;
                  notify_help_msg.data = "start help";
                  notify_help_start_pub.publish(notify_help_msg);
                  helping_player = true;
              }

            } catch(int error){
                switch(error){
                    case 1:
                       ROS_ERROR("Cannot help without a world state due to no connection to service");
                       as_.setAborted(result_.result,"Cannot help without a world state due to no connection to service");
                       break;
                    case 2:
                       ROS_ERROR("There is nobody to help");
                       as_.setAborted(result_.result,"There is nobody to help");
                       break;
                    case 3:
                       ROS_ERROR("We are still helping a user. Cannot move to next user yet.");
                       as_.setAborted(result_.result,"We are still helping a user. Cannot move to next user yet.");
                       break;
                }
                return;
            }
        } else if((action_to_execute.name.compare("MOVE_TO_FRONT") == 0) && world_state.robot_state.compare("At front of room") != 0){
            tangy.move_straight_no_rotate(-0.5);
            tangy.rotate("map", 3.1415);
        }

        // Navigate
        try{
            if(is_nav_goal_close(world_state,action_to_execute))
                throw 1;
            else if(isnan(action_to_execute.location.x) && isnan(action_to_execute.location.y)){
                throw 2;
            } else {
                tangy.move("map",action_to_execute.location.x,action_to_execute.location.y);
                tangy.rotate("map",0);
            }
        } catch(int error){
            switch(error){
                case 1:
                    ROS_WARN("Not going to navigate. The navigation goal is too close");
                    break;
                case 2:
                    ROS_WARN_STREAM(action_to_execute.name + " does not have any base movements");
                    break;
            }

        }

        //Avoid hitting the table
        if(world_state.robot_state.compare("At front of room") != 0 && goal->run_gestures && action_to_execute.name.compare("MOVE_TO_FRONT") != 0 && action_to_execute.name.compare("MOVE_TO_HELP") != 0){
            tangy.move_straight_no_rotate(-0.35);
            tangy.rotate("map",0);
        }

        // Perform arm gestures
        if(goal->run_gestures){
            if(boost::filesystem::exists(action_to_execute.gesture_trajectory_file + "_left_arm.gesture")){
                exec_gesture_file(action_to_execute.gesture_trajectory_file);
                ROS_INFO("Executing custom gestures");
            } else {
                ROS_INFO("Attempting to run default gesture");
                run_default_gesture(action_to_execute.gesture_trajectory_file);
            }
        } else {
            ROS_WARN("Gestures were turned off for this action goal");
        }

        // Talk and Display
        if(goal->run_speech){
            std::string activity_window = "bingo";
            ROS_INFO(action_to_execute.speech.c_str());
            text_to_speech_client.send_action_goal(action_to_execute.speech);
            tangy.display_text(activity_window,action_to_execute.speech,0);
        } else {
            ROS_WARN("Speech was turned off for this action goal");
        }

        while(trajectory_executor_client.is_goal_active() || text_to_speech_client.is_goal_active());

        if(world_state.robot_state.compare("At front of room") != 0 && goal->run_gestures && action_to_execute.name.compare("MOVE_TO_FRONT") != 0){
            tangy.move_straight_no_rotate(0.35);
            tangy.rotate("map",0);
        }

        // Complete Action
        result_.result.done = 1;
        as_.setSucceeded(result_.result,"Success");
        last_executed_action = requested_action;
    }    
}
void ActionExecutorServer::run_default_gesture(std::string gesture_name){
    if(!gesture_name.compare("wave")){
        ROS_INFO("Sending default gesture goal: Wave");
        tangy.wave();
    } else if(!gesture_name.compare("point_at_screen")){
        ROS_INFO("Sending default gesture goal: Point at Screen");
        tangy.point_at_screen();
    } else if(!gesture_name.compare("celebrate")){
        ROS_INFO("Sending default gesture goal: Celebrate");
        tangy.celebrate();
    } else if(!gesture_name.compare("laugh_gesture")){
        ROS_INFO("Sending default gesture goal: Laugh Gesture");
        tangy.laugh_gesture();
    }
}

// Populate actions from XML database file
void ActionExecutorServer::populate_action_list(std::string action_file){
		actions.clear();
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(action_file.c_str());
    if(result.status != pugi::status_ok) {
        std::cout << std::endl << "action file " << action_file << " not found!" << std::endl;
        return;

    }

    pugi::xpath_node actions_node;
    actions_node = doc.select_single_node("actions");

    std::cout << "----------------------------------" << std::endl;
    // Add Actions
    for(pugi::xml_node action_node = actions_node.node().child("action"); action_node; action_node = action_node.next_sibling("action")) {
        std::string name =  action_node.attribute("name").value();
        std::string speech = action_node.attribute("speech").value();
        std::string gesture = action_node.attribute("gesture").value();
        std::string locationX = action_node.attribute("locationX").value();
        std::string gui_text = action_node.attribute("gui_text").value();
        Point point;
        if(locationX.compare("NONE") == 0){
            point.x = NAN;
            point.y = NAN;
        } else {
            point.x = boost::lexical_cast<float>(action_node.attribute("locationX").value());
            point.y = boost::lexical_cast<float>(action_node.attribute("locationY").value());
        }

        Action new_action = {name, gui_text, speech, gesture, point};
        actions.insert(std::pair<std::string,Action>(name,new_action));
    }
}

void ActionExecutorServer::save_action_database_cb(const std_msgs::String::ConstPtr& msg){
    save_current_database(msg->data.c_str());
    //doc.save_file(msg->data.c_str());
}

bool ActionExecutorServer::get_action_list_srv(demonstration_learning_msgs::get_action_list::Request &req,
                                                demonstration_learning_msgs::get_action_list::Response &res)
{
    std::vector<std::string> action_list;
    std::vector<std::string> gui_text_list;
    for (auto const& element : actions) {
        action_list.push_back(element.first);
        gui_text_list.push_back(element.second.gui_text);
    }
    res.action_list = action_list;
    res.gui_text_list = gui_text_list;
    ROS_INFO("Sending Action List :");
    for(int i=0; i<action_list.size(); i++){
        ROS_INFO_STREAM(action_list[i]);
    }
    return true;
}

bool ActionExecutorServer::exe_spch_and_gest_srv_cb(demonstration_learning_msgs::exe_spch_and_gest::Request &req,
                                      demonstration_learning_msgs::exe_spch_and_gest::Response &res){
    bool thread_created = false;
    // Run gesture file path

    std::string gesture_file_path = ros::package::getPath(req.gesture_containing_package)
                                    + req.gesture_relative_file_path;
		
    //std::string gesture_file_path = req.gesture_file_path;
    if (gesture_file_path.find_first_not_of(' ') != std::string::npos) {
        if(boost::filesystem::exists(gesture_file_path + "_left_arm.gesture")){
            trajectory_executor_client.send_action_goal(gesture_file_path);
            //thread_ = boost::thread(&ActionExecutorServer::exec_gesture_file, this, gesture_file_path);
            //			thread_created = true;
            ROS_INFO("Executing custom gestures");
        } else {
            ROS_ERROR("File provided does not exist. Not running gesture. File: [%s]", gesture_file_path.c_str());
        }
    } else {
        ROS_WARN("No gesture file path provided. Not running gesture");
    }

    // Run speech
    std::string speech = req.speech;
    if (speech.find_first_not_of(' ') != std::string::npos) {
        //Use custom speech
        std::string activity_window = "bingo";
        if(req.turn_off_display){
            text_to_speech_client.send_action_goal(speech);
        } else {
            text_to_speech_client.send_action_goal(speech);
            tangy.display_text(activity_window,speech,0);
        }

        
    } else {
       ROS_WARN("No speech provided. Not running speech");
    }
		
    while(trajectory_executor_client.is_goal_active() || text_to_speech_client.is_goal_active());

    res.success = true;
    return true;
}

bool ActionExecutorServer::preview_customization_cb(demonstration_learning_msgs::preview_customization::Request &req,
                              demonstration_learning_msgs::preview_customization::Response &res){
		bool thread_created = false;    
		
		std::map<std::string,Action>::iterator iter = actions.find(req.action_name);

    if ( iter == actions.end() ) {
        ROS_ERROR("Cannot preview customization because action does not exist in the database.");
        return false;
    } else {
        Action original_action = iter->second;

        // Set speech for preview
        std::string speech = req.speech;
        if (speech.find_first_not_of(' ') != std::string::npos) {
            //Use custom speech
        } else {
           speech  = original_action.speech;
            ROS_ERROR("Given speech is only whitespaces. Using default speech for the preview");
        }

        // Set gesture file path for preview
        std::string gesture_file_path = req.gesture_file_path;
        if (gesture_file_path.find_first_not_of(' ') != std::string::npos) {
            //Use custom gesture file path
        } else {
            gesture_file_path  = original_action.gesture_trajectory_file;
            ROS_ERROR("Trajectory file is only whitespaces. Using default trajectory file for the preview");
        }


        if(boost::filesystem::exists(gesture_file_path + "_left_arm.gesture")){
            trajectory_executor_client.send_action_goal(gesture_file_path);
            //thread_ = boost::thread(&ActionExecutorServer::exec_gesture_file, this, gesture_file_path);
            //thread_created = true;
            ROS_INFO("Executing custom gestures");
        } else {
            ROS_ERROR("Preview using default gestures created in MoveIt!");
            run_default_gesture(gesture_file_path);
        }

        std::string activity_window = "bingo";
        text_to_speech_client.send_action_goal(speech);
        tangy.display_text(activity_window,speech,0);

        while(trajectory_executor_client.is_goal_active() || text_to_speech_client.is_goal_active());

        res.success = true;

    }

    return true;
}

void ActionExecutorServer::publish_called_numbers(){
    std_msgs::Int32MultiArray called_numbers_msg;
    called_numbers_msg.data = number_handler.get_previously_called_numbers();
    called_numbers_pub.publish(called_numbers_msg);
    ROS_INFO("Published Called Numbers :");
    for(int i=0; i<called_numbers_msg.data.size(); i++){
        ROS_INFO_STREAM(called_numbers_msg.data[i]);
    }
}
bool ActionExecutorServer::get_last_executed_action_srv(demonstration_learning_msgs::get_last_executed_action::Request &req,
                                                    demonstration_learning_msgs::get_last_executed_action::Response &res){
    res.last_action = last_executed_action;
    ROS_INFO_STREAM("Sending last executed action :" << res.last_action);
    return true;
}

demonstration_learning_msgs::world_state ActionExecutorServer::get_world_state(){
    demonstration_learning_msgs::get_current_world_state srv;
    demonstration_learning_msgs::world_state current_world_state;
    srv.request.get_world_state = "true";
    if(get_world_state_client.call(srv)){
        ROS_INFO("Yay we got the current world state");
        current_world_state = srv.response.current_world_state;
        return current_world_state;
    } else {
        ROS_ERROR("Bad service call for get_current_world_state service");
        current_world_state.activity_state = "NULL";
        current_world_state.name = "NULL";
        return current_world_state;
    }
}

bool ActionExecutorServer::is_nav_goal_close(demonstration_learning_msgs::world_state current_world_state, Action action_to_execute){
    if(isnan(action_to_execute.location.x) && isnan(action_to_execute.location.y)){
        return false;
    }

    float distXGoal = current_world_state.robot_location.pose.position.x - action_to_execute.location.x;
    float distYGoal = current_world_state.robot_location.pose.position.y - action_to_execute.location.y;
    float distFromGoal = sqrt(distXGoal*distXGoal + distYGoal*distYGoal);
    if(distFromGoal < nav_goal_tolerence){
        return true;
    } else {
        return false;
    }
}

void ActionExecutorServer::customize_action_cb(const demonstration_learning_msgs::customize_action::ConstPtr& msg){
    std::map<std::string,Action>::iterator iter = actions.find(msg->action_name);

    if ( iter == actions.end() ) {
        ROS_ERROR("Do nothing: Action does not exist in the database.");
        return;
    } else {
        Action custom_action;
        Action original_action = iter->second;
        custom_action.name = msg->action_name;


        std::string speech = msg->speech;
        if (speech.find_first_not_of(' ') != std::string::npos) {
            custom_action.speech = speech;
        } else {
            custom_action.speech  = original_action.speech;
            ROS_ERROR("Given speech is only whitespaces. Using default speech for the action");
        }

        std::string gesture_file_name = msg->gesture_file_name;
        if (gesture_file_name.find_first_not_of(' ') != std::string::npos) {
            custom_action.gesture_trajectory_file = gesture_file_name;
        } else {
            custom_action.gesture_trajectory_file  = original_action.gesture_trajectory_file;
            ROS_ERROR("Trajectory file is only whitespaces. Using default trajectory file for the action");
        }

        custom_action.location.x = original_action.location.x;
        custom_action.location.y = original_action.location.y;

        iter->second = custom_action;
    }
    std::string temp_path = ros::package::getPath("action_executor") + "/database/temp_database_actions.xml";
    save_current_database(temp_path.c_str());
}

bool ActionExecutorServer::init_database_cb(demonstration_learning_msgs::init_action_database::Request &req,
                      demonstration_learning_msgs::init_action_database::Response &res){
    int folder_number = 0;
    std::string template_path = ros::package::getPath("action_executor") + "/database/custom_actions/save";
    boost::filesystem::path dir_path;
    do{
        folder_number++;
        dir_path = template_path + boost::lexical_cast<std::string>(folder_number);
    }while(boost::filesystem::is_directory(dir_path));

    boost::filesystem::create_directory(dir_path);
    res.database_dir = template_path + boost::lexical_cast<std::string>(folder_number);
    return true;
}


std::string ActionExecutorServer::get_file_extension(std::string file_path){
    boost::filesystem::path file(file_path);
    std::string extension = file.extension().c_str();
    return extension;
}

bool ActionExecutorServer::exec_gesture_file(std::string file_path){
    trajectory_executor_client.send_action_goal(file_path);
    /*
    performing_gesture = true;
    robot_inverse_kinematics::execute_gesture_file srv;
    srv.request.file_path = file_path;
		ROS_INFO("Gesture file path: [%s]", file_path.c_str());
    if(exec_gesture_file_client.call(srv)){
        ROS_INFO("Yay we are executing arm gestures");
        performing_gesture = false;
        return true;
    } else {
        ROS_ERROR("Bad service call for exec_gesture_file service");
        performing_gesture = false;
        return false;
    }
    */
}

void ActionExecutorServer::move_straight_cmd_cb(const std_msgs::Float32::ConstPtr &msg){
    ROS_INFO("Moving straight [%f] metres", msg->data);
    tangy.move_straight_no_rotate(msg->data);
    tangy.rotate("map",0);
}

void ActionExecutorServer::save_current_database(std::string filename){
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(default_action_database.c_str());
    if(result.status != pugi::status_ok) {
        std::cout << std::endl << "action file " << default_action_database << " not found!" << std::endl;
        return;
    }

    pugi::xpath_node actions_node;
    actions_node = doc.select_single_node("actions");

    // Save Actions
    for(pugi::xml_node action_node = actions_node.node().child("action"); action_node; action_node = action_node.next_sibling("action")) {
        std::map<std::string,Action>::iterator iter = actions.find(action_node.attribute("name").value());
        Action custom_action = iter->second;
        action_node.attribute("speech").set_value(custom_action.speech.c_str());
        action_node.attribute("gesture").set_value(custom_action.gesture_trajectory_file.c_str());
    }

    doc.save_file(filename.c_str());
}

// Start action server
int main(int argc, char **argv) {
    ros::init(argc, argv, "action_executor_server");
    ros::NodeHandle n;
    srand(time (NULL));

    std::string wait_time_string;
	n.getParam("/server_wait_time", wait_time_string);
	ROS_INFO_STREAM("World State Identifier Server will wait for " << wait_time_string << " seconds");
    int wait_time = boost::lexical_cast<int>(wait_time_string);
    ActionExecutorServer server("action_executor_server",wait_time);
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
