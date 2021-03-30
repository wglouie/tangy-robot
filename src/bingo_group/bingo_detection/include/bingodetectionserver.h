/*
 * Frank Despond
 *
 * Version 1.0---->Refactored from old BingoDetectionAction Code
 *
 * January 30, 2015
 */
#ifndef BINGODETECTIONSERVER_H
#define BINGODETECTIONSERVER_H
//Standard Headers
#include <ros/package.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sstream>
#include <vector>
#include <algorithm>
//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
//Include headers for OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//Include headers for ROS services
#include <actionlib/server/simple_action_server.h>
#include <bingo_detection/BingoDetectionAction.h>

#include "bingocard.h"

#include <string.h>
#include <fstream>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

class BingoDetectionServer
{
public:
    BingoDetectionServer(string name);
    ~BingoDetectionServer();

private:


   // BingoCard *bingo;
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void goalCB();
    void preemptCB();
    void done();


    ros::NodeHandle nh_;

    /*Camera grab variables*/
    std::string cameraTopic;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    /*Action library variables*/
    actionlib::SimpleActionServer<bingo_detection::BingoDetectionAction> as_;
    std::string action_name_;
    bingo_detection::BingoDetectionGoal goal_;
    bingo_detection::BingoDetectionFeedback feedback_;					//Provides feedback on whether card is detected
    bingo_detection::BingoDetectionResult result_;						//Provides feedback on the result of detection
    std::vector<int> calledNumbers;
    int gameType;

    /*fault publisher*/
    ros::Publisher faultpub;

    /*Bingo detection variables*/
    std::string cardDatabaseFolder;
    std::string numberDatabaseFile;

    /*For saving results */
    std::ofstream resultFile;
    int detectionCounter;

		/*Last Analyzed*/
		sensor_msgs::ImageConstPtr last_image;
};


#endif // BINGODETECTIONSERVER_H
