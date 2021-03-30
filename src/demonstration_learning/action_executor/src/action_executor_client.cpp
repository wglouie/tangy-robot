#include "action_executor/action_executor_client.h"

ActionExecutorClient::ActionExecutorClient(std::string name):
    ac("action_executor_server", true),
    //Stores the name
    action_name(name)
{
    //Get connection to a server
    ROS_INFO("%s Waiting For Server...", action_name.c_str());
    //Wait for the connection to be valid
    ac.waitForServer();
    ROS_INFO("%s Got a Server...", action_name.c_str());

    init();
}

ActionExecutorClient::ActionExecutorClient(std::string name,float wait_time):
    ac("action_executor_server", true),
    //Stores the name
    action_name(name)
{
    //Get connection to a server
    ROS_INFO("%s Waiting For Server...", action_name.c_str());
    //Wait for the connection to be valid
    if(ac.waitForServer(ros::Duration(wait_time))){
        ROS_INFO("%s Got a Server...", action_name.c_str());
    } else {
        ROS_INFO("%s Failed to get a Server...", action_name.c_str());
    }

    init();
}

void ActionExecutorClient::init(){
    action_list_srv_client = nh_.serviceClient<demonstration_learning_msgs::get_action_list>("action_executor/get_action_list");
    exe_spch_and_gest_srv_client = nh_.serviceClient<demonstration_learning_msgs::exe_spch_and_gest>("action_executor/exe_spch_and_gest");
    preview_customization_client = nh_.serviceClient<demonstration_learning_msgs::preview_customization>("action_executor/preview_customization");
    active_goal = false;
}



void ActionExecutorClient::send_action_goal(std::string gui_text_action_name, bool arm_movements, bool speech)
{
    action_executor::action_executorGoal newGoal;
    if (gui_text_action_map.find(gui_text_action_name) == gui_text_action_map.end() ) {
        newGoal.action_name = gui_text_action_name;
    } else {
        newGoal.action_name = gui_text_action_map.at(gui_text_action_name);
    }
    //newGoal.action_name = gui_text_action_map.at(gui_text_action_name);
    newGoal.run_gestures = arm_movements;
    newGoal.run_speech = speech;

    ROS_INFO("Goal has been sent");

    ac.sendGoal(newGoal, boost::bind(&ActionExecutorClient::doneCb, this, _1, _2),
            boost::bind(&ActionExecutorClient::activeCb, this),
            boost::bind(&ActionExecutorClient::feedbackCb, this, _1));
    failed_goal = false;
    active_goal = true;
}

std::map<std::string,std::string> ActionExecutorClient::get_action_list(){
    demonstration_learning_msgs::get_action_list srv;
    srv.request.get_action_list = "get_list";

    std::vector<std::string> action_list;
    std::vector<std::string> gui_text_list;
    if (action_list_srv_client.call(srv)){
        action_list = srv.response.action_list;
        gui_text_list = srv.response.gui_text_list;
        ROS_INFO("Received Action List:");
        for(int i=0; i<action_list.size(); i++){
            ROS_INFO_STREAM(action_list[i]);
        }
    }
    else{
      ROS_ERROR("Failed to call service get_action_list");
    }

    std::map<std::string,std::string> temp_map;
    for(int i=0; i<action_list.size(); i++){
        temp_map.insert(std::pair<std::string,std::string>(gui_text_list[i],action_list[i]));
    }
    gui_text_action_map = temp_map;
    return gui_text_action_map;
}

// Called once when the goal completes
void ActionExecutorClient::doneCb(const actionlib::SimpleClientGoalState &state, const action_executor::action_executorResultConstPtr& result)
{
    active_goal = false;
    ROS_INFO("Action Executor Finished in state [%s]", state.toString().c_str());
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO(state.getText().c_str());
        failed_goal = false;
        goal_error_msg = "";
    } else if (state == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO(state.getText().c_str());
        failed_goal = true;
        goal_error_msg = state.getText();
    }
}

// Called once when the goal becomes active
void ActionExecutorClient::activeCb()
{
    ROS_INFO("Goal just went active...");
}

// Called every time feedback is received for the goal
void ActionExecutorClient::feedbackCb(const action_executor::action_executorFeedbackConstPtr& feedback)
{

}

bool ActionExecutorClient::is_goal_active(){
    return active_goal;
}
bool ActionExecutorClient::did_goal_fail(){
    return failed_goal;
}

std::string ActionExecutorClient::get_error_msg(){
    return goal_error_msg;
}

void ActionExecutorClient::exe_spch_and_gest(std::string speech, std::string gesture_containing_package, std::string gesture_relative_file_path,bool turn_off_display){
    demonstration_learning_msgs::exe_spch_and_gest srv;
        srv.request.speech = speech;
        srv.request.gesture_containing_package = gesture_containing_package;
        srv.request.gesture_relative_file_path = gesture_relative_file_path;
				srv.request.turn_off_display = turn_off_display;
        if (exe_spch_and_gest_srv_client.call(srv)){
            ROS_INFO("Speech and gesture was successful");
        }
        else{
          ROS_ERROR("Failed to call service exe_spch_and_gest");
        }

        return;
}

void ActionExecutorClient::preview_customization(std::string action_name ,std::string speech, std::string gesture_file_path){
    demonstration_learning_msgs::preview_customization srv;
        srv.request.action_name = action_name;
        srv.request.speech = speech;
        srv.request.gesture_file_path = gesture_file_path;
        if (preview_customization_client.call(srv)){
            ROS_INFO("Preview customization was successful");
        }
        else{
          ROS_ERROR("Failed to call service preview customization");
        }

        return;
}



