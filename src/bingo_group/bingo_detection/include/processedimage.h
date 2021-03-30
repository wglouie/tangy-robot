/*
 * Geoff Louie & Frank Despond
 *
 * Version 2.0---->Refactored from old BingoDetection Code ---- GEOFF....
 *
 * January 26, 2015
 */
#ifndef PROCESSEDIMAGE_H
#define PROCESSEDIMAGE_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/photo/photo.hpp"
#include "squarefinder.h"
#include "debugopencv.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

struct cardInformation{
    Mat individualBingoCard;
    Mat individualBigGrid;
    Mat individualBingoCdst;
    vector< vector<Point> > individualDistoredContours;
    int largestIndividualSquare;
    Point2f individualBigGridCentroid;
    float sizeOfBingoGrid;
};

class ProcessedImage
{
public:

    ProcessedImage(Mat rawImage);
    ~ProcessedImage();

    //getters
    Mat getImage();
    Mat getBigGrid();
    vector<Point2f> getCentroids();
    Point2f getCardCentroidLocation();
    Mat getUndistortedImage();

private:

    vector<cardInformation> multipleCards;

    //image initialized when object is constructed
    const bool debug = false;

    //Used for calculating sizes for all cards
    int largestSquare;
    int counter;
    const int numberOfProcesses = 5;

    vector< vector<Point> > distortedContours ,squareCheck;

    Mat image, bigGrid, undistortedImage, cdst, finalImage, protectedImage;

    vector<Point> gridCorners, cardCorners;

    Point2f bigGridCentroid;

    vector<Point2f> centroids;

    //closetCard is the closetCard to Tangy based on Grid size
    //These values are the ones returned

    Point2f closestBigGridCentroid;

    Mat closestCardFinalImage, closestCardCdst,closestCardBigGrid;

    vector< vector<Point> > closestCardDistortedContours;

    int closestCardLargestSquare;

    vector<Point2f> closestCardCentroids;
    int counted;

    Mat testImage; //Remove after paper

    SquareFinder cardFinder;
    debugOpenCV debugger;

    void bingoCardCentroids();
    int findBingoGrid(Mat inputImage, int process);
    void reduceImageSize(Mat &imageToReduce, Mat &regionOfInterest,bool inverse, bool white);
    void whiteBalance(Mat &image);
    void undistortCard(Mat grid, Mat originalImage, Mat &undistortedGrid, vector< vector<Point> > contours, int largestSquare,bool cardOnly);
    int findMultipleCards(Mat inputImage);
    float sizeOfGrid(Mat &imageOfGrid);
    void selectLargestGrid();
    void reset();
};

#endif // PROCESSEDIMAGE_H
