#ifndef WorldStateIdentifierClient_H_
#define WorldStateIdentifierClient_H_
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <world_state_identifier/world_state_identifierAction.h>
#include <actionlib/client/terminal_state.h>
#include <demonstration_learning_msgs/load_last_state.h>
#include <demonstration_learning_msgs/get_current_world_state.h>
struct world_state_goal{
    world_state_goal() : person_id(true), robot_state(true), assist_req(true), user_activity_state(true) {}
    bool person_id;
    bool robot_state;
    bool assist_req;
    bool user_activity_state;
};

class WorldStateIdentifierClient{

public:
    WorldStateIdentifierClient(std::string name);
    WorldStateIdentifierClient(std::string, float wait_time);
    void send_identification_goal(world_state_goal goal);
    bool is_goal_active();
    void load_saved_state();
    demonstration_learning_msgs::world_state get_world_state();

private:
    world_state_identifier::world_state_identifierActionFeedback feedback_;
    world_state_identifier::world_state_identifierActionResult result_;
    actionlib::SimpleActionClient<world_state_identifier::world_state_identifierAction> ac;
    ros::ServiceClient load_saved_state_client;
    ros::ServiceClient get_current_world_state_client;

    void doneCb(const actionlib::SimpleClientGoalState& state, const world_state_identifier::world_state_identifierResultConstPtr& result);
	void activeCb();
    void feedbackCb(const world_state_identifier::world_state_identifierFeedbackConstPtr& feedback);
    void init();
    void get_world_state_service();

    bool active_goal;
    std::string action_name;
    ros::NodeHandle nh_;
    demonstration_learning_msgs::world_state current_world_state;
};

#endif
