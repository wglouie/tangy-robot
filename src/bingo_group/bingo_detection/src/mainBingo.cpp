/*
 * Frank Despond
 *
 * Version 1.0---->Refactored from old BingoDetectionAction Code
 *
 * January 30, 2015
 */
#include "bingodetectionserver.h"

//Standard Headers
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <sstream>
#include <vector>
//Input and Output Files
#include <iostream>
#include <fstream>
//ActionLib Headers
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <bingo_detection/BingoDetectionAction.h>
#include <algorithm>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "BingoDetection");
    BingoDetectionServer bingoDetection("BingoDetection");
    ros::spin();
    return 0;
}
