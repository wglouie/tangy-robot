/*
 * Frank Despond
 *
 * Version 1.0
 *
 * January 29, 2015
 */
#ifndef SURFDETECTION_H
#define SURFDETECTION_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <string.h>
#include <fstream>
#include <vector>

using namespace cv;
using namespace std;

//TODO: Add in stuff for when you dont need to do SURF

class SURFDetection
{
public:

    SURFDetection(string cardFolder, Mat inputImage);
    ~SURFDetection();

    //getters

    //0-straight, 1-right, 2-upsidedown, 3-left
    int getOrientation();
    //call to get card that was a match.. returns -1 if no matches
    // Numbers correspond with the arrangement in the file of cards eg.. duck, car, toy. duck=0, car=1, toy=2
    int getMatchedMarker();



private:


    static std::vector<std::string> cardDatabase;
    int minHessian;
    int cardOrientation;
    int matchedCardMarker;
    std::string numberFile;
    Mat testImage;
    std::string databaseFolder;

    void identifyCardMarker();
    int objectDetect(Mat img_object);




};

#endif // SURFDETECTION_H
