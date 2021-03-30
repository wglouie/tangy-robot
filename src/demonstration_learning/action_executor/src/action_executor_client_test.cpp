#include <ros/ros.h>
#include <action_executor/action_executor_client.h>
int main (int argc, char **argv)
{
    ros::init(argc, argv, "action_executor_client");

    //Initialize the client
    ActionExecutorClient client(ros::this_node::getName());
    client.exe_spch_and_gest("Hello world", "action_executor", "/database/demonstration_gestures/psi_pose");
    return 1;
}

