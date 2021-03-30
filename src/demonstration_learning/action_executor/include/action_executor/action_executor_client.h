#ifndef ActionExecutorClient_H_
#define ActionExecutorClient_H_
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <action_executor/action_executorAction.h>
#include <actionlib/client/terminal_state.h>
#include <demonstration_learning_msgs/get_action_list.h>
#include <demonstration_learning_msgs/exe_spch_and_gest.h>
#include <demonstration_learning_msgs/preview_customization.h>

class ActionExecutorClient{

public:
    ActionExecutorClient(std::string name);
    ActionExecutorClient(std::string, float wait_time);
    void send_action_goal(std::string gui_text_action_name, bool arm_movements = true, bool speech = true);						//Takes a goal (number of frames) to identify faces
    bool is_goal_active();
    bool did_goal_fail();
    std::string get_error_msg();
    std::map<std::string,std::string> get_action_list();
    void exe_spch_and_gest(std::string speech, std::string gesture_containing_package, std::string gesture_relative_file_path, bool turn_off_display = false);
    void preview_customization(std::string action_name ,std::string speech, std::string gesture_file_path);

private:
    action_executor::action_executorActionFeedback feedback_;
    action_executor::action_executorActionResult result_;
    actionlib::SimpleActionClient<action_executor::action_executorAction> ac;
    ros::ServiceClient action_list_srv_client;
    ros::ServiceClient called_numbers_srv_client;
    ros::ServiceClient exe_spch_and_gest_srv_client;
    ros::ServiceClient preview_customization_client;
    void doneCb(const actionlib::SimpleClientGoalState& state, const action_executor::action_executorResultConstPtr& result);

	void activeCb();
    void feedbackCb(const action_executor::action_executorFeedbackConstPtr& feedback);
    void init();

    std::string action_name;
    ros::NodeHandle nh_;
    bool active_goal;
    bool failed_goal;
    std::string goal_result;
    std::string goal_error_msg;
    std::map<std::string,std::string> gui_text_action_map;

};

#endif
