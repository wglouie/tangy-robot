#include "trajectory_executor/trajectory_executor_client.h"

TrajectoryExecutorClient::TrajectoryExecutorClient(std::string name):
    ac("trajectory_executor_server", true),
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

TrajectoryExecutorClient::TrajectoryExecutorClient(std::string name,float wait_time):
    ac("trajectory_executor_server", true),
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

void TrajectoryExecutorClient::init(){
    active_goal = false;
}



void TrajectoryExecutorClient::send_action_goal(std::string file_name)
{
    trajectory_executor::execute_fileGoal newGoal;
    newGoal.file_name = file_name;

    ROS_INFO("Goal has been sent");

    ac.sendGoal(newGoal, boost::bind(&TrajectoryExecutorClient::doneCb, this, _1, _2),
            boost::bind(&TrajectoryExecutorClient::activeCb, this),
            boost::bind(&TrajectoryExecutorClient::feedbackCb, this, _1));
    failed_goal = false;
    active_goal = true;
}


// Called once when the goal completes
void TrajectoryExecutorClient::doneCb(const actionlib::SimpleClientGoalState& state, const trajectory_executor::execute_fileResultConstPtr& result)
{
    active_goal = false;
    remaining_execution_time = 0;
    ROS_INFO("Trajectory Executor Finished in state [%s]", state.toString().c_str());
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
void TrajectoryExecutorClient::activeCb()
{
    ROS_INFO("Goal just went active...");
}

// Called every time feedback is received for the goal
void TrajectoryExecutorClient::feedbackCb(const trajectory_executor::execute_fileFeedbackConstPtr& feedback)
{
    remaining_execution_time = feedback->time_left;
}

bool TrajectoryExecutorClient::is_goal_active(){
    return active_goal;
}
bool TrajectoryExecutorClient::did_goal_fail(){
    return failed_goal;
}

std::string TrajectoryExecutorClient::get_error_msg(){
    return goal_error_msg;
}



