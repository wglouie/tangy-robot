/*
 * Frank Despond
 *
 * Version 1.0---->Refactored from old BingoDetection Code
 *
 * January 30, 2015
 */
#ifndef BINGOCARD_H
#define BINGOCARD_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include "processedimage.h"
#include "redmarkers.h"
#include "surfdetection.h"


#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <string.h>
#include <fstream>
#include <vector>
#include "std_msgs/String.h"
#include <sstream>


#include <ros/console.h>

using namespace cv;
using namespace std;

class BingoCard
{
public:
    BingoCard(Mat inputImage,
              string inputCardFolder,
              vector<int> inputCalledNumbers,
              int inputGameType,
              string inputNumberDatabaseFile);
    ~BingoCard();

    int cardHasBingo();
    vector<int> getMissingNumbers();
    vector<int> getWrongNumbers();

private:
    //bingo detection objects
    //RedMarkers *redMarkersCheck;
    //SURFDetection *cardMarkerCheck;
    ProcessedImage imageProcessed;

    //bingo detection variables
    int cardBingo;
    vector<int> missingNumbers;
    vector<int> wrongNumbers;
    string cardFolder;
    vector<int> calledNumbers;
    int gameType;
    string numberDatabaseFile;
    vector<Point2f> cardGridCentroids;

    //to publish to node
    ros::Publisher publisher;
    ros::NodeHandle n;
    std_msgs::String msg;
    std::stringstream ss;

    void checkCard();

};

#endif // BINGOCARD_H
