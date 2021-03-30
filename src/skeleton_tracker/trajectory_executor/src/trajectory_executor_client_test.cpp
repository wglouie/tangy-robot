#include <ros/ros.h>
#include <trajectory_executor/trajectory_executor_client.h>
int main (int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_executor_client");

    //Initialize the client
    TrajectoryExecutorClient client(ros::this_node::getName());
    client.send_action_goal("/home/tangy/tangy-robot/src/demonstration_learning/action_executor/database/demonstration_gestures/psi_pose");
    //client.exe_spch_and_gest("Hello world", "action_executor", "/database/demonstration_gestures/psi_pose");
    return 1;
}

