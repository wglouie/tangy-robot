#include <help_indicators/get_help_indicators.h>
#include <help_indicators/triangle.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <vector>



int main(int argc, char** argv) {
    ros::init( argc, argv, "help_indicator_client" );
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<help_indicators::get_help_indicators>("get_help_indicators");
    ros::Publisher go_pub = n.advertise<std_msgs::String>("help_indicators_go", 1000);
    ros::Publisher clear_pub = n.advertise<std_msgs::String>("clear_help_indicator", 1000);
    ros::Publisher head_ready_pub = n.advertise<std_msgs::Bool>("head_ready", 1000);

    sleep(1);

    std_msgs::String stop_msg;
    stop_msg.data = "stop";

    std_msgs::String go_msg;
    go_msg.data = "go";

    std_msgs::Bool head_ready;
    head_ready.data = true;

    // tell the kinect to start looking for triangles
    go_pub.publish(go_msg);

    // turn acknowledgments on
    head_ready_pub.publish(head_ready);

    // check if theres any help_indicators
    std::vector<help_indicators::triangle> help_indicators;

    while(1) {
        help_indicators::get_help_indicators srv;
        srv.request.str = "Anything";
        if(client.call(srv)) {
            help_indicators = srv.response.help_indicators;
        } else {
            ROS_ERROR("Failed to call service get_help_indicators");
            return 1;
        }
        if(help_indicators.empty() == false) {
            ROS_INFO("There is a help_indicator at: (%f, %f, %f)", help_indicators.at(0).x, help_indicators.at(0).y, help_indicators.at(0).z);
            sleep(20);

            // once you check the indicators clear them
            std_msgs::String clear_msg;
            clear_msg.data = help_indicators[0].name;
            clear_pub.publish(clear_msg);
        }
    }
    return 0;
}
