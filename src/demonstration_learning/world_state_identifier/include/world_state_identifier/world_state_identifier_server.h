#ifndef WORLD_STATE_IDENTIFIER_H
#define WORLD_STATE_IDENTIFIER_H

//Action library headers
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//Identification modules
#include <bingo_detection_client.h>
#include <FaceDetectionClient.h>
#include <help_indicators/get_help_indicators.h>
#include <help_indicators/triangle.h>
#include <help_indicators_client.h>

// Robot state
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// World State Identification Package Messages
#include <world_state_identifier/world_state_identifierAction.h>

// Messages
#include <demonstration_learning_msgs/world_state.h>
#include <demonstration_learning_msgs/bingo_numbers_string.h>
#include <demonstration_learning_msgs/get_current_world_state.h>
#include <demonstration_learning_msgs/load_last_state.h>

#include <tangyRobot.h>
#include <std_msgs/Int32MultiArray.h>
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <boost/assign.hpp>


class WorldStateIdentifierServer{

private:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<world_state_identifier::world_state_identifierAction> as_;

    //Publishers
    ros::Publisher missing_numbers_pub;
    ros::Publisher incorrect_numbers_pub;
    ros::Publisher load_saved_triangles_pub;
    ros::Publisher notify_help_start_pub;
    ros::Publisher init_pose_pub;
    ros::Subscriber notfiy_end_task_sub;
    ros::Subscriber notify_help_start_sub;
    ros::Subscriber notify_help_stop_sub;
    ros::Subscriber learning_state_sub;


    //Clients for servers and services
    BingoDetectionClient bingo_detection_client;
    HelpIndictatorsClient help_indicators_client;
    FaceDetectionClient face_detection_client;
    tf::TransformListener tf_listener;
    ros::ServiceClient called_numbers_srv_client;
    ros::ServiceServer current_world_state_srv;
    ros::ServiceServer load_last_state_srv;
    ros::Subscriber called_numbers_sub;


    //Functions
    void init();

    void check_bingo_card();
    void update_card_state();
    void get_robot_pose();
    void identify_help();
    void detect_face();
    void set_activity_state();
    void set_robot_state();

    void start_help_pub();
    bool is_robot_at_front();
    bool is_robot_at_help();
    void save_state(std::string filename);
    void init_robot_pose(double x, double y, double theta);


    bool load_state_srv(demonstration_learning_msgs::load_last_state::Request &req,
                    demonstration_learning_msgs::load_last_state::Response &res);
    bool get_current_world_state_srv(demonstration_learning_msgs::get_current_world_state::Request &req,
                                     demonstration_learning_msgs::get_current_world_state::Response &res);

    void learning_state_cb(const std_msgs::String::ConstPtr& msg);
    void help_start_cb(const std_msgs::String::ConstPtr& msg);
    void help_stop_cb(const std_msgs::String::ConstPtr& msg);
    void end_task_cb(const std_msgs::String::ConstPtr& msg);
    void called_numbers_cb(const std_msgs::Int32MultiArray::ConstPtr& msg);


    //World State Variables
    demonstration_learning_msgs::world_state world_state;
    std::vector<int> called_numbers;
    help_indicators::triangle *person_being_helped;
    tangyRobot tangy;
    float nav_goal_tolerence;
    std::string save_file;
    bool task_ended;

public:
    //General stuff
    WorldStateIdentifierServer(std::string name);
    WorldStateIdentifierServer(std::string name, float wait_time);
    ~WorldStateIdentifierServer();
    //void pauseCallback(const std_msgs::String::ConstPtr& str);
    //void endGameCallback(const std_msgs::String::ConstPtr& str);
    void goalCB(const world_state_identifier::world_state_identifierGoal::ConstPtr &goal);
};



#endif // WORLD_STATE_IDENTIFIER_H
