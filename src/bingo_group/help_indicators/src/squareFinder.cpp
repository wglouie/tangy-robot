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

using namespace cv;
using namespace std;

Point pt[4];
Point cardCentroid2;

double angle(Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;

    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void findSquares4( Mat img, vector< vector<Point> >& triangles )
{

    vector< vector<Point> >contours;
    vector< vector<Point> >triangleTemp;
	vector<Vec4i>hierarchy;
	int boxCount = 0;
    int N = 11;
    CvSize sz = Size( img.size().width, img.size().height);
    Mat timg = img.clone(); // make a copy of input image
    Mat gray = Mat(sz, CV_8UC1);
    Mat tgray;

    double s, t; 
    Mat test =Mat(sz, CV_8UC3);
    tgray = Mat(sz, CV_8UC1);
    cvtColor(timg,tgray,CV_RGB2GRAY);
    threshold(tgray,tgray,10,255,THRESH_BINARY);
    threshold( tgray, gray, 50, 255, CV_THRESH_BINARY );

    findContours( gray,contours,hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );

    // test each contour
    for(int j = 0;j<contours.size();j++)
    {
       // printf("%f\n",arcLength(Mat(contours[j],0),0));
        // approximate contour with accuracy proportional
        // to the contour perimeter
        approxPolyDP(Mat(contours[j]), contours[j],arcLength(Mat(contours[j],0),1)*0.15,1);
        // triangles contours should have 3 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation
/*
	Mat drawing = Mat::zeros( img.size(), CV_8UC3 );
		RNG rng(12345);
	Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	drawContours( drawing, contours, j, color, 2, 8, hierarchy, 0, Point() );
     	imshow("Contour",drawing);
	waitKey();
*/
         if( contours[j].size() != 3 && fabs(contourArea(Mat(contours[j]))) > 5)
         {

             contours.erase (contours.begin()+j);;
         }

       else if( contours[j].size() == 3 && fabs(contourArea(Mat(contours[j]))) > 5) //&&
        {
            s = 0;
            for(int i = 0; i < 3; i++ )
            {
                // find minimum angle between joint
                // edges (maximum of cosine)
                if( i >= 2 )
                {
                    t = fabs(angle(contours[j][i], contours[j][i-2], contours[j][i-1]));
                    s = s > t ? s : t;
                }
            }
            // if cosines of all angles are small triangles[1].x
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence 
            if( s < 1)
            {
                triangleTemp.push_back(vector<Point>());
                for(int i = 0; i < 3; i++ )
				{
                    triangleTemp[boxCount].push_back(contours[j][i]);
				}	
				boxCount = boxCount + 1;
            }
				
		}
	}

 
    triangles = triangleTemp;
}



