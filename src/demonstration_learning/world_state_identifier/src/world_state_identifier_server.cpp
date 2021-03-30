#include <world_state_identifier/world_state_identifier_server.h>

WorldStateIdentifierServer::WorldStateIdentifierServer(std::string name):
    as_(nh_,name, boost::bind(&WorldStateIdentifierServer::goalCB, this, _1), false)
    , bingo_detection_client("BingoDetection")
    , face_detection_client("FaceDetectionClient")
    , help_indicators_client("HelpDetectionClient")
    , tf_listener()
    , tangy()
    {
    init();
}
WorldStateIdentifierServer::WorldStateIdentifierServer(std::string name, float wait_time):
    as_(nh_,name, boost::bind(&WorldStateIdentifierServer::goalCB, this, _1), false)
    , bingo_detection_client("BingoDetection",wait_time)
    , face_detection_client("FaceDetectionClient",wait_time)
    , help_indicators_client("HelpDetectionClient")
    , tf_listener()
    , tangy(wait_time){
    init();
}
WorldStateIdentifierServer::~WorldStateIdentifierServer(){

}

void WorldStateIdentifierServer::init(){
    as_.start();
    ROS_INFO("[%s] World State Identification server is running" , ros::this_node::getName().c_str());

    nh_.getParam("/nav_goal_tolerence", nav_goal_tolerence);
    nh_.getParam("/saved_world_state_file", save_file);

    //Initialize all publishers and services
    current_world_state_srv = nh_.advertiseService("world_state_identifier/get_current_world_state", &WorldStateIdentifierServer::get_current_world_state_srv, this);
    load_last_state_srv = nh_.advertiseService("world_state_identifier/load_last_state", &WorldStateIdentifierServer::load_state_srv, this);
    called_numbers_sub = nh_.subscribe("action_executor/called_numbers", 1000, &WorldStateIdentifierServer::called_numbers_cb,this);
    notify_help_start_sub = nh_.subscribe("action_executor/notify_help_start", 1000, &WorldStateIdentifierServer::help_start_cb, this);
    notify_help_stop_sub = nh_.subscribe("rqt_task_learning/notify_help_stop", 1000, &WorldStateIdentifierServer::help_stop_cb, this);
    learning_state_sub = nh_.subscribe("rqt_task_learning/learning_state", 1000, &WorldStateIdentifierServer::learning_state_cb, this);
    notfiy_end_task_sub = nh_.subscribe("rqt_task_learning/notify_end_task", 1000, &WorldStateIdentifierServer::end_task_cb, this);

    missing_numbers_pub = nh_.advertise<demonstration_learning_msgs::bingo_numbers_string>("world_state_identifier/missing_numbers", 1000);
    incorrect_numbers_pub = nh_.advertise<demonstration_learning_msgs::bingo_numbers_string>("world_state_identifier/incorrect_numbers", 1000);
    load_saved_triangles_pub = nh_.advertise<help_indicators::triangle>("help_indicators/add_new_triangle", 1000);
    init_pose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1000);
    notify_help_start_pub = nh_.advertise<std_msgs::String>("action_executor/notify_help_start", 1000);

    world_state.activity_state = "START";
    world_state.name = "Unknown";
    world_state.user_activity_state = "Not Detected";
    person_being_helped = NULL;
    world_state.robot_location.pose.position.x = 0;
    world_state.robot_location.pose.position.y = 0;
    world_state.person_being_helped.name = "NULL";
    task_ended = false;
}

void WorldStateIdentifierServer::learning_state_cb(const std_msgs::String::ConstPtr& msg){
    std::string learning_state = msg->data;
    help_indicators_client.start_looking_for_help();
    if(learning_state.compare("new game") == 0){
        world_state.activity_state = "START";
        world_state.name = "Unknown";
        world_state.user_activity_state = "Not Detected";
        world_state.robot_location.pose.position.x = 0;
        world_state.robot_location.pose.position.y = 0;
        set_robot_state();
        person_being_helped = NULL;
        world_state.person_being_helped.name = "NULL";
        task_ended = false;
        init_robot_pose(0,0,0);

    } else if (learning_state.compare("continue game") == 0){
        if(world_state.person_being_helped.name.compare("NULL") == 0){

        } else {
            //world_state.activity_state = "Performing Task";
            ROS_INFO("World State Identifier Loaded File that is in the help phase");
            person_being_helped = &world_state.person_being_helped;
            start_help_pub();

        }
    } else if (learning_state.compare("run policy") == 0){
        world_state.activity_state = "START";
        world_state.name = "Unknown";
        world_state.user_activity_state = "Not Detected";
        world_state.robot_location.pose.position.x = 0;
        world_state.robot_location.pose.position.y = 0;
        set_robot_state();
        person_being_helped = NULL;
        world_state.person_being_helped.name = "NULL";
        task_ended = false;
        init_robot_pose(0,0,0);
    }
}

void WorldStateIdentifierServer::end_task_cb(const std_msgs::String::ConstPtr& msg){
    std::string expected_string = "end task";
     if(expected_string.compare(msg->data) == 0){
         task_ended = true;
     } else {
         ROS_ERROR("Unexpected message published on notify_end_task topic");
     }
}

void WorldStateIdentifierServer::start_help_pub(){
    //All nodes need to know we are helping someone again
    std_msgs::String notify_help_msg;
    notify_help_msg.data = "start help";
    notify_help_start_pub.publish(notify_help_msg);
}

void WorldStateIdentifierServer::called_numbers_cb(const std_msgs::Int32MultiArray::ConstPtr& msg){
    called_numbers = msg->data;
}

void WorldStateIdentifierServer::check_bingo_card(){
    if(called_numbers.size() == 0){
        ROS_ERROR("No called numbers yet. Cannot check Bingo card.");
        return;
    }

    tangy.look_at_card_1();
    //sleep(5);
    ROS_INFO("Checking Bingo Card");
    bingo_detection_client.sendGoalAndWait(called_numbers);
    string card_state = bingo_detection_client.get_GameState();
    int num_checks = 1;
    while(num_checks < 6){
        switch(num_checks){
            case 1:
                bingo_detection_client.sendGoalAndWait(called_numbers);
                break;
            case 2:
                tangy.look_at_card_2();
                bingo_detection_client.sendGoalAndWait(called_numbers);
                break;
            case 3:
                bingo_detection_client.sendGoalAndWait(called_numbers);
                break;
            case 4:
               tangy.look_at_card_3();
               bingo_detection_client.sendGoalAndWait(called_numbers);
               break;
            case 5:
               bingo_detection_client.sendGoalAndWait(called_numbers);
               break;
        }
        card_state = bingo_detection_client.get_GameState();
				if(card_state.compare("error") != 0 && card_state.compare("unknown") !=0){
					break;
				}        
				num_checks++;
    }
    update_card_state();
    tangy.look_back_to_person();

}

void WorldStateIdentifierServer::update_card_state(){
    std::vector<std::string> missing_numbers = bingo_detection_client.get_MissingNumbers();
    std::vector<std::string> incorrect_numbers = bingo_detection_client.get_WrongNumbers();
    demonstration_learning_msgs::bingo_numbers_string bingo_number_msg;
    string card_state = bingo_detection_client.get_GameState();
    ROS_INFO_STREAM("Number of missing numbers: %d" << incorrect_numbers.size());
    ROS_INFO_STREAM(card_state);
    if(!card_state.compare("good card")){
        world_state.user_activity_state = "Correctly Marked";
    } else if (!card_state.compare("bad card")){
        if(missing_numbers.size() != 0){
            ROS_INFO("Sending missing numbers");
            world_state.user_activity_state = "Missing Numbers";
            bingo_number_msg.bingo_numbers = missing_numbers;
            //missing_numbers_pub.publish(bingo_number_msg);
            world_state.bad_bingo_numbers = missing_numbers;
        } else if(incorrect_numbers.size() != 0){
            ROS_INFO("Sending incorrect numbers");
            world_state.user_activity_state = "Incorrectly Marked";
            bingo_number_msg.bingo_numbers = incorrect_numbers;
            //incorrect_numbers_pub.publish(bingo_number_msg);
            world_state.bad_bingo_numbers = incorrect_numbers;
        } else {
            ROS_ERROR("Logic is wrong for world state identifier bingo detection");
        }
    } else if (!card_state.compare("bingo")){
        world_state.user_activity_state = "Bingo"   ;
    } else if (!card_state.compare("error")){
        world_state.user_activity_state = "Occluded";
        ROS_ERROR("Card detection error");
    }
    //Always send missing or incorrect numbers so that subscribers world states are up to date
    missing_numbers_pub.publish(bingo_number_msg);
    incorrect_numbers_pub.publish(bingo_number_msg);
}

void WorldStateIdentifierServer::get_robot_pose(){
    geometry_msgs::PoseStamped poseTo;
    geometry_msgs::PoseStamped poseFrom;

    // Robot position relative to itself
    poseFrom.header.frame_id = "base_link";
    poseFrom.pose.position.x = 0;
    poseFrom.pose.position.y = 0;
    poseFrom.pose.position.z = 0;
    poseFrom.pose.orientation.w = 1;

    // Transform robot position relative to the map
    try {
        ros::Time current_time = ros::Time::now();
        poseFrom.header.stamp = current_time;
        poseTo.header.stamp = current_time;
        while( !(tf_listener.waitForTransform("map", poseFrom.header.frame_id, current_time, ros::Duration(1.0))) ){}
        tf_listener.transformPose("map", poseFrom, poseTo);
    } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    
    ROS_INFO("The robot is at [x = %f], [y = %f], [z = %f]", poseTo.pose.position.x, poseTo.pose.position.y, poseTo.pose.position.z);
    world_state.robot_location = poseTo;
}

void WorldStateIdentifierServer::identify_help(){
    std::vector<help_indicators::triangle> indicators = help_indicators_client.get_help_requests();
    world_state.user_assistance_requests = indicators;
    ROS_INFO("There are [%d] people I have to help!", (int) indicators.size());
}

void WorldStateIdentifierServer::detect_face() {
    ROS_INFO("Trying to detect any faces...");

    face_detection_client.send(10);
    double begin=ros::Time::now().toSec();
    bool timeout=false;

    while(!face_detection_client.goal_completed()&& timeout!=true) {
        double end=ros::Time::now().toSec();
        if(end>(begin+3)){
          timeout=true;
        }
    }

    //the line below no longer makes sense
    //std::string name=faceDetectionClient.person_of_interest();
    //instead i am doing the line below which in this current implementation will always return the name "POI"
    std::string name=face_detection_client.get_POIname();
    ROS_INFO("Face detection client returning: [%s]",name.c_str());
    if(name.compare("No Users")==0||name.compare("Unknown")==0) {
        world_state.name = "";
    } else {
        world_state.name = name;
    }

}

void WorldStateIdentifierServer::goalCB(const world_state_identifier::world_state_identifierGoal::ConstPtr &goal){

    /*-----------Removed Face Detection for Demonstration Learning----------------*/
    /*
    if(goal->person_identification == true){
        detect_face();
    } else {
        world_state.name = "Unknown";
    }
    */
    if(goal->robot_state == true){
        get_robot_pose();
    }
    if(goal->assistance_request == true){
        identify_help();
    }
    if(goal->user_activity_state == true){
        check_bingo_card();
    } else {
        world_state.user_activity_state = "Not Detected";
    }

    set_activity_state();
    set_robot_state();
    world_state_identifier::world_state_identifierActionResult result_;
    result_.result.updated_world_state = world_state;
    as_.setSucceeded(result_.result,"Success");
    save_state(save_file);
}

bool WorldStateIdentifierServer::get_current_world_state_srv(demonstration_learning_msgs::get_current_world_state::Request &req,
                                 demonstration_learning_msgs::get_current_world_state::Response &res){
    res.current_world_state = world_state;
    ROS_INFO_STREAM("Sending current world state");

    return true;
}

void WorldStateIdentifierServer::help_start_cb(const std_msgs::String::ConstPtr& msg){
   std::string expected_string = "start help";
    if(expected_string.compare(msg->data) == 0){
        person_being_helped = &world_state.user_assistance_requests[0];
        world_state.person_being_helped = world_state.user_assistance_requests[0];
        ROS_INFO("Help was started in action executor server. Storing triangle information.");
    } else {
        ROS_ERROR("Unexpected message published on notify_help_start topic");
    }
}

void WorldStateIdentifierServer::help_stop_cb(const std_msgs::String::ConstPtr& msg){
    std::string expected_string = "stop help";
     if(expected_string.compare(msg->data) == 0){
         if(person_being_helped != NULL){
             help_indicators_client.clear_request(person_being_helped->name);
             ROS_INFO("Help is done according to task learning GUI. Clearing triangle from help_indicator_server.");
             person_being_helped = NULL;
             world_state.person_being_helped.name = "NULL";
         } else {
             ROS_ERROR("Can't clear triangle because we are not helping anyone.");
         }

     } else {
         ROS_ERROR("Unexpected message published on notify_help_stop topic.");
     }
}

void WorldStateIdentifierServer::set_robot_state(){
    if(is_robot_at_front()){
        world_state.robot_state = "At front of room";
    } else {
        if(is_robot_at_help()){
            world_state.robot_state = "At player that needs help";
        } else {
            world_state.robot_state = "At player that does not need help";
        }
    }
}

bool WorldStateIdentifierServer::is_robot_at_front(){

    float distX = world_state.robot_location.pose.position.x - 0;
    float distY = world_state.robot_location.pose.position.y - 0;
    float distFromFront = sqrt(distX*distX + distY*distY);
    ROS_INFO("Is robot at front? %.3f %.3f [value=%d]", world_state.robot_location.pose.position.x
                                                      , world_state.robot_location.pose.position.y
                                                      , distFromFront < nav_goal_tolerence);
    if(distFromFront < nav_goal_tolerence){
        return true;
    } else {
        return false;
    }
}

bool WorldStateIdentifierServer::is_robot_at_help(){
    float distX = world_state.robot_location.pose.position.x - world_state.user_assistance_requests[0].x;
    float distY = world_state.robot_location.pose.position.y - world_state.user_assistance_requests[0].y;
    float distFromPlayer = sqrt(distX*distX + distY*distY);
    if(distFromPlayer < nav_goal_tolerence){
        return true;
    } else {
        return false;
    }
}

void WorldStateIdentifierServer::save_state(std::string filename){
    ROS_INFO("Saving world state");
    std::string saved_file = filename;
    // Write to File
    std::ofstream ofs(saved_file.c_str(), std::ios::out|std::ios::binary|std::ios::trunc);

    if(!ofs){
        ROS_ERROR("Failed to open file to save");
    } else {
        ROS_INFO("Saved world state");
    }

    uint32_t serial_size = ros::serialization::serializationLength(world_state);
    boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
    ros::serialization::OStream ostream(obuffer.get(), serial_size);
    ros::serialization::serialize(ostream, world_state);
    ofs.write((char*) obuffer.get(), serial_size);
    ofs.close();
}

bool WorldStateIdentifierServer::load_state_srv(demonstration_learning_msgs::load_last_state::Request &req,
                                            demonstration_learning_msgs::load_last_state::Response &res){
    std::string saved_file = save_file;
    std::ifstream ifs(saved_file.c_str(), std::ios::in|std::ios::binary);
    if(!ifs){
        ROS_ERROR("File failed to load");
        res.current_world_state = world_state;
        return false;
    } else {
        ROS_INFO("Loading saved world state");
    }

    ifs.seekg (0, std::ios::end);
    std::streampos end = ifs.tellg();
    ifs.seekg (0, std::ios::beg);
    std::streampos begin = ifs.tellg();

    uint32_t file_size = end-begin;
    boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
    ifs.read((char*) ibuffer.get(), file_size);
    ros::serialization::IStream istream(ibuffer.get(), file_size);
    ros::serialization::deserialize(istream, world_state);
    ifs.close();

    if(world_state.user_assistance_requests.size() == 0){
        ROS_INFO("No triangles were loaded");
    } else {
        ROS_INFO("[%d] triangles were loaded", world_state.user_assistance_requests.size());
        for(int i=0; i<world_state.user_assistance_requests.size(); i++){
            load_saved_triangles_pub.publish(world_state.user_assistance_requests[i]);
        }
    }


    init_robot_pose(world_state.robot_location.pose.position.x,world_state.robot_location.pose.position.y,0);

    res.current_world_state = world_state;
    return true;
}

void WorldStateIdentifierServer::init_robot_pose(double x, double y, double theta){

    std::string fixed_frame = "map";
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = fixed_frame;
    pose.header.stamp = ros::Time::now();
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;

    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta);
    tf::quaternionTFToMsg(quat,
                          pose.pose.pose.orientation);
    pose.pose.covariance[6*0+0] = 0.5 * 0.5;
    pose.pose.covariance[6*1+1] = 0.5 * 0.5;
    pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
    ROS_INFO("Setting pose: %.3f %.3f %.3f [frame=%s]", x, y, theta, fixed_frame.c_str());
    init_pose_pub.publish(pose);
}

void WorldStateIdentifierServer::set_activity_state(){
    if(task_ended){
        world_state.activity_state = "END";
        ROS_INFO("Setting activity state to END");
    } else {
        if(person_being_helped == NULL){
            world_state.activity_state = "FACILITATE";
        } else {
            world_state.activity_state = "HELP";
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "world_state_identifier_server");
    ros::NodeHandle nh_;
    srand(time (NULL));
    std::string wait_time_string;
    nh_.getParam("/server_wait_time", wait_time_string);

    int wait_time = boost::lexical_cast<int>(wait_time_string);
    WorldStateIdentifierServer *server;
    if(wait_time != 0){
        ROS_INFO_STREAM("World State Identifier Server will wait for " << wait_time_string << " seconds");
        server = new WorldStateIdentifierServer("world_state_identifier_server",wait_time);
    } else {
        ROS_INFO_STREAM("World State Identifier Server will wait until all servers are available");
        server = new WorldStateIdentifierServer("world_state_identifier_server");
    }

    ros::spin();
    return 0;
}
























