/*
 * Frank Despond
 *
 * Version 1.0
 *
 * January 28, 2015
 */
#ifndef SQUAREFINDER_H
#define SQUAREFINDER_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/photo/photo.hpp"
#include "ros/ros.h"


#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <iostream>

//Make this more self containing in the Future.
//refactored from older Bingo stuff
//Class used to find largest square and Squares in a grid pattern. Used to recognize the card's grid pattern

using namespace cv;
using namespace std;
class SquareFinder
{
public:
    SquareFinder();
    ~SquareFinder();

    //Returns the number of the squares in a grid (Bingo card should return 25)
    int getNumberOfSquaresInGrid();
    //finds the biggest square in the image
    void findBiggestSquare(Mat inputImage, vector<vector<Point> > &bigSquare);
    //Finds squares in a grid pattern and returns them
    void findSquaresInGrid(Mat inputImage, vector<vector<Point> > &gridSquares);
    //draws the squares into the image
    Mat drawSquares(Mat img, vector< vector<Point> > squares,int squarePosition,bool multiSquare);
    //largest outer
    int findOuterBoxOfGrid(vector< vector<Point> > &squares);
private:
    int numberOfSquaresInGrid;
    Point pt[4];


};

#endif // SQUAREFINDER_H
