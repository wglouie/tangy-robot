#include <help_indicators/help_indicators_client.h>

HelpIndictatorsClient::HelpIndictatorsClient(std::string name){
    help_indicators_client = nh_.serviceClient<help_indicators::get_help_indicators>("get_help_indicators");
    help_indicators_pub = nh_.advertise<std_msgs::String>("help_indicators_go", 1000);
    clear_pub = nh_.advertise<std_msgs::String>("clear_help_indicator", 1000);
}
HelpIndictatorsClient::~HelpIndictatorsClient(){
}
void HelpIndictatorsClient::start_looking_for_help(){
    ROS_INFO("Start looking for help requests...");
    std_msgs::String start_msg;
    start_msg.data = "start";       // Any message other than "stop" will start the help indicators
    help_indicators_pub.publish(start_msg);
}

void HelpIndictatorsClient::stop_looking_for_help(){
    ROS_INFO("Stopping scanning for help requests.");
    std_msgs::String stop_msg;
    stop_msg.data = "stop";
    help_indicators_pub.publish(stop_msg);
}

std::vector<help_indicators::triangle> HelpIndictatorsClient::get_help_requests(){
    // check if theres any help_indicators
    help_indicators::get_help_indicators help_indicator_srv;
    std::vector<help_indicators::triangle> help_indicators_info;
    help_indicator_srv.request.str = "Anything";
    try {
        help_indicators_client.call(help_indicator_srv);
        help_indicators_info = help_indicator_srv.response.help_indicators;
    } catch (std::string error) {
        ROS_ERROR("Failed to call service: get_help_indicators, %s!", error.c_str());
    }
    return help_indicators_info;
}

void HelpIndictatorsClient::clear_request(std::string name){
    if(!name.empty()){
        ROS_INFO("Finished helping [%s]. Sending request to remove from queue...", name.c_str());
        std_msgs::String clear_msg;
        clear_msg.data = name;
        clear_pub.publish(clear_msg);
    }else{
        ROS_WARN("Cannot clear an empty name!");
    }
}
