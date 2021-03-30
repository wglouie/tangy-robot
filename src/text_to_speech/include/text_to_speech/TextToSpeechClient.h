#ifndef TextToSpeechClient_H_
#define TextToSpeechClient_H_
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <text_to_speech/txt_to_speechAction.h>
#include <actionlib/client/terminal_state.h>

class TextToSpeechClient{

public:
    TextToSpeechClient(std::string name);
    TextToSpeechClient(std::string, float wait_time);
    void send_action_goal(std::string speech);
    bool is_goal_active();
    bool did_goal_fail();
    std::string get_error_msg();

private:
    text_to_speech::txt_to_speechActionFeedback feedback_;
    text_to_speech::txt_to_speechActionResult result_;
    actionlib::SimpleActionClient<text_to_speech::txt_to_speechAction> ac;
    void doneCb(const actionlib::SimpleClientGoalState& state, const text_to_speech::txt_to_speechResultConstPtr& result);
	void activeCb();
    void feedbackCb(const text_to_speech::txt_to_speechFeedbackConstPtr& feedback);
    void init();
    double get_exe_time_left;

    std::string action_name;
    ros::NodeHandle nh_;
    bool active_goal;
    bool failed_goal;
    std::string goal_result;
    std::string goal_error_msg;
    double remaining_execution_time;
};

#endif
