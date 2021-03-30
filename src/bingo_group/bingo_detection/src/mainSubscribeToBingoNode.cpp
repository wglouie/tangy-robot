#import "subscribernode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bingoListener");

    SubscriberNode subscribeToBingoDetectionNode("bingoDetectionInfo", 1000);

    ros::spin();

    return 0;
}
