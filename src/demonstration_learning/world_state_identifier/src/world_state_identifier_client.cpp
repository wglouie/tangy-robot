#include "world_state_identifier/world_state_identifier_client.h"

WorldStateIdentifierClient::WorldStateIdentifierClient(std::string name):
    ac("world_state_identifier_server", true),
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

WorldStateIdentifierClient::WorldStateIdentifierClient(std::string name,float wait_time):
    ac("world_state_identifier_server", true),
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

void WorldStateIdentifierClient::init(){
    load_saved_state_client = nh_.serviceClient<demonstration_learning_msgs::load_last_state>("world_state_identifier/load_last_state");
    get_current_world_state_client = nh_.serviceClient<demonstration_learning_msgs::get_current_world_state>("world_state_identifier/get_current_world_state");
    get_world_state_service();
    active_goal = false;
}


void WorldStateIdentifierClient::send_identification_goal(world_state_goal goal)
{
    world_state_identifier::world_state_identifierGoal newGoal;
    newGoal.person_identification = goal.person_id;
    newGoal.robot_state = goal.robot_state;
    newGoal.assistance_request = goal.assist_req;
    newGoal.user_activity_state = goal.user_activity_state;
    ROS_INFO("Goal has been sent");
    //Once again, have to used boost::bind because you are inside a class

    ac.sendGoal(newGoal, boost::bind(&WorldStateIdentifierClient::doneCb, this, _1, _2),
            boost::bind(&WorldStateIdentifierClient::activeCb, this),
            boost::bind(&WorldStateIdentifierClient::feedbackCb, this, _1));

    active_goal = true;
}

// Called once when the goal completes
void WorldStateIdentifierClient::doneCb(const actionlib::SimpleClientGoalState &state, const world_state_identifier::world_state_identifierResultConstPtr& result)
{
    current_world_state = result->updated_world_state;
    ROS_INFO("World State Identifier Finished in state [%s]", state.toString().c_str());
    active_goal = false;
}

// Called once when the goal becomes active
void WorldStateIdentifierClient::activeCb()
{
    ROS_INFO("Goal just went active...");
}

// Called every time feedback is received for the goal
void WorldStateIdentifierClient::feedbackCb(const world_state_identifier::world_state_identifierFeedbackConstPtr& feedback)
{

}

bool WorldStateIdentifierClient::is_goal_active(){
    return active_goal;
}

demonstration_learning_msgs::world_state WorldStateIdentifierClient::get_world_state(){
		get_world_state_service();
    return current_world_state;
}

void WorldStateIdentifierClient::load_saved_state(){
    demonstration_learning_msgs::load_last_state srv;
    srv.request.load_state = "go";

    if (load_saved_state_client.call(srv)){
        current_world_state = srv.response.current_world_state;
        ROS_INFO("Loaded saved world state");
    }
    else{
      ROS_ERROR("Failed to call service load_last_state");
    }

    return;
}

void WorldStateIdentifierClient::get_world_state_service(){

    demonstration_learning_msgs::get_current_world_state srv;
    srv.request.get_world_state = "go";

    if (get_current_world_state_client.call(srv)){
        current_world_state = srv.response.current_world_state;
        ROS_INFO("Got current world state");
    }
    else{
      ROS_ERROR("Failed to call service get_current_world_state");
    }

    return;
}
