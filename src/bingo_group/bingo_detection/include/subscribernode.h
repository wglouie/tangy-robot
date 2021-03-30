#ifndef SUBSCRIBERNODE_H
#define SUBSCRIBERNODE_H

#include <ros/ros.h>
#include "std_msgs/String.h"

using namespace std;

class SubscriberNode
{
public:
    SubscriberNode(string inputNodeName, int inputQueueSize);
    ~SubscriberNode();

private:
    //ROS Objects
    ros::NodeHandle n;
    ros::Subscriber sub;
    //Node Specific Variables
    string nodeName;
    int queueSize;

};

#endif // SUBSCRIBERNODE_H
