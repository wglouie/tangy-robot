#include <ros/ros.h>
#include <world_state_identifier/world_state_identifier_client.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "world_state_identifier_client");

    //Initialize the client
    WorldStateIdentifierClient client(ros::this_node::getName());
    world_state_goal goal; //Initializes to true for all immediately
    client.send_identification_goal(goal);
    return 1;
}

