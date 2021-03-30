#ifndef TrajectoryExecutorClient_H_
#define TrajectoryExecutorClient_H_
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_executor/execute_fileAction.h>
#include <actionlib/client/terminal_state.h>

class TrajectoryExecutorClient{

public:
    TrajectoryExecutorClient(std::string name);
    TrajectoryExecutorClient(std::string, float wait_time);
    void send_action_goal(std::string file_name);
    bool is_goal_active();
    bool did_goal_fail();
    std::string get_error_msg();

private:
    trajectory_executor::execute_fileActionFeedback feedback_;
    trajectory_executor::execute_fileActionResult result_;
    actionlib::SimpleActionClient<trajectory_executor::execute_fileAction> ac;
    void doneCb(const actionlib::SimpleClientGoalState& state, const trajectory_executor::execute_fileResultConstPtr& result);
	void activeCb();
    void feedbackCb(const trajectory_executor::execute_fileFeedbackConstPtr& feedback);
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
