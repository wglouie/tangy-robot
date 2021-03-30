/*
 * Frank Despond
 *
 * Version 1.0---->Refactored from old BingoDetection Code
 *
 * January 29, 2015
 */
#ifndef REDMARKERS_H
#define REDMARKERS_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include "debugopencv.h"

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <string.h>
#include <fstream>

using namespace cv;
using namespace std;

class RedMarkers
{
public:
    //initalize, if no gameType is passed through it is set to lines
    //cardMarker is a number which corresponds with the location of the array of markers
    //cardGrid is the picture of the grid with the markers on it straightened

    RedMarkers(Mat inputCardGrid, vector<int> inputCalledNumbers,
                        int inputGameType, string inputNumberDatabaseFile,
                        int inputCardMarker,
                        vector<Point2f> inputCardGridCentroids);
    ~RedMarkers();
    void printOffResults();

    vector<int> getWrongNumbersMarked();
    vector<int> getMissingNumbers();
    //return 0 for no Bingo and 1 for Bingo
    int checkForBingo();

private:
    bool debug = false;
    int bingo;
    int checkBingoCard[25];
    int cardMarker;
    Mat cardGrid;
    int gameType;
    int numberOfCards;
    string numberDatabaseFile;
    vector<string> cardDatabase;
    vector<Point2f> circleCentroids;
    vector<Point2f> cardGridCentroids;
    vector<Point2f> markerCentroids;

    vector<Mat> individualSquares;


    vector<int> currentCardNumbers;
    vector<int> calledNumbers;
    vector<int> markedNumbers;
    vector<int> wrongNumbersMarked;
    vector<int> missingNumbers;
    vector<vector<int> > numberDatabase;

    void grabNumberDatabase();
    void circleFinder();
    int checkNumbers();
    bool contains(int elem, vector<int> list);
    int closestSquare( Point2f circleOfInterest);
    Point2f centroid(vector<Point> contour);
    Mat forThePaper ;

    debugOpenCV debugger;


};

#endif // REDMARKERS_H
