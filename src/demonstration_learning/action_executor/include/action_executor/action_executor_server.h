#ifndef ACTION_EXECUTOR_H
#define ACTION_EXECUTOR_H

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <tangyRobot.h>
#include <action_executor/pugixml.h>
#include <action_executor/bingo_number_handler.h>
#include <action_executor/action_executorAction.h>
#include <demonstration_learning_msgs/world_state.h>
#include <demonstration_learning_msgs/bingo_numbers_string.h>
#include <demonstration_learning_msgs/get_action_list.h>
#include <demonstration_learning_msgs/get_last_executed_action.h>
#include <demonstration_learning_msgs/get_current_world_state.h>
#include <demonstration_learning_msgs/exe_spch_and_gest.h>
#include <demonstration_learning_msgs/init_action_database.h>
#include <demonstration_learning_msgs/customize_action.h>
#include <demonstration_learning_msgs/preview_customization.h>
#include <robot_inverse_kinematics/execute_gesture_file.h>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <trajectory_executor_client.h>
#include <TextToSpeechClient.h>


struct Point{
    float x;
    float y;
};

struct Action{
    std::string name;
    std::string gui_text;
    std::string speech;
    std::string gesture_trajectory_file;
    Point location;

};

class ActionExecutorServer{

private:
    //Variables
    ros::NodeHandle nh_;
    std::string last_executed_action;
    actionlib::SimpleActionServer<action_executor::action_executorAction> as_;
    std::map<std::string,Action> actions;
    std::string default_gesture_mode;
    std::vector<std::string> last_missing_numbers;
    std::vector<std::string> last_incorrect_numbers;
    float nav_goal_tolerence;
    bool helping_player;
    bool performing_gesture;
    std::string progress_file_location;
    std::string default_action_database;

    //Publishers, Subscribers and services
    ros::Publisher exec_traj_pub;
    ros::Publisher called_numbers_pub;
    ros::Publisher notify_help_start_pub;
    ros::Subscriber move_straight_sub;
    ros::Subscriber learning_state_sub;
    ros::Subscriber notify_help_stop_sub;
    ros::Subscriber missing_numbers_sub;
    ros::Subscriber incorrect_numbers_sub;
    ros::Subscriber notify_help_start_sub;
    ros::Subscriber customize_action_sub;
    ros::Subscriber save_custom_database_sub;
    ros::ServiceServer action_list_srv;
    ros::ServiceServer last_action_srv;
    ros::ServiceServer exe_spch_and_gest_srv;
    ros::ServiceServer preview_customization_srv;
    ros::ServiceServer init_database_srv;
    ros::ServiceClient get_world_state_client;
    ros::ServiceClient exec_gesture_file_client;
    //Robot and Bingo handlers
    tangyRobot tangy;
    bingo_number_handler number_handler;
    TrajectoryExecutorClient trajectory_executor_client;
    TextToSpeechClient text_to_speech_client;

		boost::thread thread_;
    //Functions
    void populate_action_list(std::string action_file);
    demonstration_learning_msgs::world_state get_world_state();

    void help_stop(const std_msgs::String::ConstPtr& msg);
    void missing_number_callback(const demonstration_learning_msgs::bingo_numbers_string::ConstPtr& msg);
    void incorrect_number_callback(const demonstration_learning_msgs::bingo_numbers_string::ConstPtr& msg);
    void learning_state_callback(const std_msgs::String::ConstPtr& msg);
    bool is_nav_goal_close(demonstration_learning_msgs::world_state current_world_state, Action action_to_execute);
    void help_start_callback(const std_msgs::String::ConstPtr& msg);
    void customize_action_cb(const demonstration_learning_msgs::customize_action::ConstPtr& msg);
    void save_action_database_cb(const std_msgs::String::ConstPtr& msg);
    bool init_database_cb(demonstration_learning_msgs::init_action_database::Request &req,
                          demonstration_learning_msgs::init_action_database::Response &res);
    std::string get_file_extension(std::string file_path);
    bool exec_gesture_file(std::string file_path);
    void move_straight_cmd_cb(const std_msgs::Float32::ConstPtr &msg);
    void save_current_database(std::string filename);
public:
    //General stuff
    ActionExecutorServer(std::string name,float wait_time);
    ~ActionExecutorServer();
    void run_default_gesture(std::string gesture_name);
    bool get_action_list_srv(demonstration_learning_msgs::get_action_list::Request &req,
                             demonstration_learning_msgs::get_action_list::Response &res);
    void publish_called_numbers();
    bool get_last_executed_action_srv(demonstration_learning_msgs::get_last_executed_action::Request &req,
                                      demonstration_learning_msgs::get_last_executed_action::Response &res);
    bool exe_spch_and_gest_srv_cb(demonstration_learning_msgs::exe_spch_and_gest::Request &req,
                    demonstration_learning_msgs::exe_spch_and_gest::Response &res);
    bool preview_customization_cb(demonstration_learning_msgs::preview_customization::Request &req,
                                  demonstration_learning_msgs::preview_customization::Response &res);


    void goalCB(const action_executor::action_executorGoal::ConstPtr &goal);


};


#endif // ACTION_EXECUTOR_H
