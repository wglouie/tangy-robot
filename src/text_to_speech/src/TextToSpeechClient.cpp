#include "text_to_speech/TextToSpeechClient.h"

TextToSpeechClient::TextToSpeechClient(std::string name):
    ac("text_to_speech_server", true),
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

TextToSpeechClient::TextToSpeechClient(std::string name,float wait_time):
    ac("text_to_speech_server", true),
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

void TextToSpeechClient::init(){
    active_goal = false;
}



void TextToSpeechClient::send_action_goal(std::string speech)
{
    text_to_speech::txt_to_speechGoal newGoal;
    newGoal.speech = speech;
    newGoal.webbased = true;
    newGoal.language = "en";
    newGoal.gender = text_to_speech::txt_to_speechGoal::MALE;


    ROS_INFO("Goal has been sent");

    ac.sendGoal(newGoal, boost::bind(&TextToSpeechClient::doneCb, this, _1, _2),
            boost::bind(&TextToSpeechClient::activeCb, this),
            boost::bind(&TextToSpeechClient::feedbackCb, this, _1));
    failed_goal = false;
    active_goal = true;
}


// Called once when the goal completes
void TextToSpeechClient::doneCb(const actionlib::SimpleClientGoalState& state, const text_to_speech::txt_to_speechResultConstPtr& result)
{
    active_goal = false;
    remaining_execution_time = 0;
    ROS_INFO("Text to Speech Finished in state [%s]", state.toString().c_str());
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
void TextToSpeechClient::activeCb()
{
    ROS_INFO("Goal just went active...");
}

// Called every time feedback is received for the goal
void TextToSpeechClient::feedbackCb(const text_to_speech::txt_to_speechFeedbackConstPtr& feedback){
    remaining_execution_time = feedback->speech_length;
}

bool TextToSpeechClient::is_goal_active(){
    return active_goal;
}
bool TextToSpeechClient::did_goal_fail(){
    return failed_goal;
}

std::string TextToSpeechClient::get_error_msg(){
    return goal_error_msg;
}



