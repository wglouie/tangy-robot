#include "subscribernode.h"


void nodeCallback(const std_msgs::String::ConstPtr& msg)
{
    printf("\n[%s]", msg->data.c_str());
}


SubscriberNode::SubscriberNode(string inputNodeName, int inputQueueSize):
    nodeName{inputNodeName},
    queueSize{inputQueueSize}
{
    sub = n.subscribe(nodeName, queueSize, nodeCallback);

}

SubscriberNode::~SubscriberNode()
{

}

