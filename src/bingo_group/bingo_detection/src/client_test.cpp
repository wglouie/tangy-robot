#include "bingo_detection_client.h"

#include <vector>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_bingodetection_client");

    ros::NodeHandle nh_;
    std::string calledNumberFile;

    //Initialize the client    vec=new vector<int> vec;

    BingoDetectionClient client(ros::this_node::getName());
    vector<int> test;

    test.push_back(12);
    client.sendGoal(test);

    ROS_INFO("Sent Goal To Server...");

    sleep(2);

    ROS_INFO("SIZE OF MISSING NUMBERS: %d", client.get_MissingNumbers().size());
    ROS_INFO("SIZE OF WRONG NUMBERS: %d", client.get_WrongNumbers().size());


    //ros::spin();


    return 0;
}
