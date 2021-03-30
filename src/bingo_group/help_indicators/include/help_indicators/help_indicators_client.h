#include <help_indicators/get_help_indicators.h>
#include <help_indicators/triangle.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <vector>

class HelpIndictatorsClient{

private:
    //Variables
    ros::NodeHandle nh_;
    ros::ServiceClient help_indicators_client;
    ros::Publisher help_indicators_pub;
    ros::Publisher clear_pub;

    //Functions

public:
    //General stuff
    HelpIndictatorsClient(std::string name);
    ~HelpIndictatorsClient();
    void start_looking_for_help();
    void stop_looking_for_help();
    std::vector<help_indicators::triangle> get_help_requests();
    void clear_request(std::string name);
};
