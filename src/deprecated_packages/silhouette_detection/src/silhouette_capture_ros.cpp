#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/video/video.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

using namespace cv;
using namespace std;

/******************ALGORITHM********************/
/// Read from Kinect.
/// Find the depth mask seen by the kinect
/// Keep only depths within a certain range
/// Identify contours.
/// Eliminate any shapes that are not big enough to be a silhouette and draw a centroid on them.
/// Calculate the euclidean distance from each point on each contour to its respective centroid.
/// Save the euclidean distances and the contours in a csv file.

const float DISTANCE_RANGE = 0.50; // The variation from distance where we are looking for a silhouette
const int MAX_DISTANCE = 80;// distance in decimetres
const int MIN_AREA_CONTOUR = 6000; // The minimum area that we expect the silhouette to have
const bool POINT_AND_CLICK = true; // When true, shows the Depth Camera, can be used to find distances
const char TEXT_FILE[200] = "src/silhouette_detection/distance.csv"; // where the data will be saved from debug
const char SVM_CSV_FILE[200] = "src/silhouette_detection/svm_data.csv"; // where the data will be saved from distances
const int AVERAGED_FPS_FRAMES = 10;
const int OUTPUT_DOMAIN = 700; // The final domain of the data points for each contour from 0 to OUTPUT_DOMAIN
const int OUTPUT_RANGE = 1; // The final range of the data points for each contour from 0 to OUTPUT_RANGE
const int DWIDTH = 640; // constants for camera width and height, placeholder for now
const int DHEIGHT = 480;

double elapsed_time, fps, placeholder_fps; // fps-related floating point numbers
int fps_index = 1; // index of number of frames since the fps was updated for fps averaging
float depth; // raw depth value from the kinect
float depth_px; // depth value in metres, derived from depth.
bool record_dist = false; // true when right mouse button was clicked and released
int human_tag = 0; // 0 for non-human, 1 for human
int distance_trackbar = 21; // The average depth distance in decimetres where you expect the target
double dWidth;
double dHeight;

Mat depth_mask; // The raw depth image from the kinect
Mat clear_depth_mask; // A clearer depth image, showing the variation in depth better.
Mat filtered_depth_mask; // The binary image containing only the depth range we want
Mat silhouette_mask; // The final image showing possible silhouettes for identification
Point interestPt(0,0); // The pixel where your mouse clicked.
ofstream debug; // full output file for the data
ofstream distances; // csv file containining only info for the svm

struct timespec stop, start; // fps-related timers

void ChangeDistance( int, void* );

void depthROS(const sensor_msgs::Image::ConstPtr& msg)
{
	try {
        	cv_bridge::CvImageConstPtr cv_ptr;
        	cv_ptr = cv_bridge::toCvShare(msg);
		cv_ptr->image.convertTo(depth_mask, CV_32FC1, 1.0, 0);
		//cout << depth_mask.at<float>(DHEIGHT/2, DWIDTH/2) << endl;
    	} catch (const cv_bridge::Exception& e) {
        	ROS_ERROR("cv_bridge exception: %s", e.what());
    	}
}

void OnMouseLeftDepth( int event, int x, int y, int flags, void* ) // called when left mouse button is clicked, records the depth value
{
	if( event == CV_EVENT_LBUTTONUP) 
	{
		interestPt = Point(x,y);
		depth = depth_mask.at<float>(interestPt.y, interestPt.x)*1000;
		cout << "Depth: " << depth/1000.0f << endl;
	}
}

void OnMouseSilhouette( int event, int x, int y, int flags, void* ) // called when mouse button is clicked on silhouette window, records human silhouettes on right click, non-human on left click
{
	if( event == CV_EVENT_LBUTTONUP)
	{
		record_dist = true;
		cout << "Writing human contours to " << TEXT_FILE << " and data for the SVM to " << SVM_CSV_FILE << "." << endl;
		human_tag = 1;
	}

	if( event == CV_EVENT_RBUTTONUP) // doesn't work on tangy, works on my laptop
	{
		record_dist = true;
		cout << "Writing non-human contours to " << TEXT_FILE << " and data for the SVM to " << SVM_CSV_FILE << "." << endl;
		human_tag = 0;
	}
}

int main(int argc, char* argv[])
{
	dWidth = DWIDTH;
	dHeight = DHEIGHT;
	depth_mask = Mat::zeros(Size(dWidth, dHeight), CV_32FC1);
	// ROS PUBLISHER INITIALIZATION
	ros::init(argc, argv, "silhouettes_publisher");
	ros::NodeHandle n;
	ros::Subscriber kinect_sub = n.subscribe("/camera/depth/image", 1000, depthROS);
	ros::Rate loop_rate(10);
	
	if( POINT_AND_CLICK)
	{
		namedWindow("Webcam",CV_WINDOW_AUTOSIZE); // Depth Window
		setMouseCallback( "Webcam", OnMouseLeftDepth, 0 );
	}

	namedWindow("Silhouette Only", CV_WINDOW_AUTOSIZE); // Contours from those silhouettes
	setMouseCallback( "Silhouette Only", OnMouseSilhouette, 0);
	debug.open(TEXT_FILE, ios_base::app);
	distances.open(SVM_CSV_FILE, ios_base::app);
	createTrackbar("Distance: ", "Silhouette Only", &distance_trackbar, MAX_DISTANCE, ChangeDistance );

    	while (1)
	{
		int num_large_contours = 0;

		if( clock_gettime( CLOCK_REALTIME, &start) == -1 ) {
      			perror( "clock gettime" );
      			exit( EXIT_FAILURE );
    		}

		// READ FROM WEBCAM
		filtered_depth_mask = Mat::zeros(Size(dWidth, dHeight), CV_8UC1);
		depth_mask.convertTo( clear_depth_mask, CV_32FC1, 2.55/distance_trackbar); // for imshow
		if( POINT_AND_CLICK) imshow("Webcam", clear_depth_mask);
		ChangeDistance( 0, 0 );

		// CONTOURS
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		findContours(filtered_depth_mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

		vector<Moments> mu( contours.size() );
		vector<Point2f> mc( contours.size() );
		silhouette_mask = Mat::zeros( Size(dWidth, dHeight), CV_8UC3);

		// FIND NUMBER OF LARGE CONTOURS
		for ( int h = 0; h < contours.size(); h++ )
		{
			if( contourArea(contours[h]) >= MIN_AREA_CONTOUR)
			{
				num_large_contours++;
			}
		}

		// DRAW CONTOURS, OUTPUT TO CSV
		for( int i = 0; i < contours.size(); i++ )
		{
			if( contourArea(contours[i]) >= MIN_AREA_CONTOUR)
			{
				// MOMENTS AND CENTROIDS FOR EACH LARGE CONTOUR
				mu[i] = moments( contours[i], false );
				mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );

				// DRAW CONTOURS AND THEIR CENTROIDS
				drawContours( silhouette_mask, contours, i, Scalar(0,0,255), 2, 8, hierarchy, 0, Point() );
				circle( silhouette_mask, mc[i], 4, Scalar(0,255,0), -1, 8, 0 );

				// RECORD IN A CSV FILE ONLY IF WE HAVE ONE LARGE CONTOUR PRESENT
				if( record_dist && num_large_contours == 1)
				{
					int distance[contours[i].size()-1];
					int max_distance = 0;
					float normalized_index[contours[i].size()-1];
					float normalized_distance[contours[i].size()-1];

					debug << "Found a candidate: #" << i << ";" << endl;
					debug << "X, Y, Index, Distance, Human?;" << endl;

					// FIND MAXIMUM DISTANCE BEFORE MODIFYING RANGE
					for( int j = 0; j < contours[i].size(); j++ )
					{
						distance[j] = sqrt(pow(contours[i][j].x-mc[i].x,2)+pow(contours[i][j].y-mc[i].y,2));
						if( distance[j] > max_distance )
						{
							max_distance = distance[j];
						}
					}
					
					// WRITE DATA
					for( int k = 0; k < contours[i].size(); k ++ )
					{
						normalized_index[k] = ((float)k+1)/(float)contours[i].size()*OUTPUT_DOMAIN;
						normalized_distance[k] = (float)distance[k]/(float)max_distance * OUTPUT_RANGE;
						debug << contours[i][k].x << "," << contours[i][k].y * -1 << "," << normalized_index[k] << "," << normalized_distance[k] << "," << human_tag << ";" << endl;
						distances << normalized_index[k] << "," << normalized_distance[k] << "," << human_tag << ";" << endl;
					}
					cout << "Successfully written a contour." << endl;
				}
			}
		}

		// IGNORE IF GREATER THAN OR LESS THAN 1 CONTOUR
		if( record_dist && num_large_contours == 0)
		{
			cout << "Could not find a contour." << endl;
		}else if( record_dist && num_large_contours > 1 )
		{
			cout << "Number of candidate contours is greater than 1. Contours were not written." << endl;
		}

		record_dist = false;

		// GET PSEUDO-FPS
		if (fps_index == AVERAGED_FPS_FRAMES) // updates value every 5th frame
		{
			if( clock_gettime( CLOCK_REALTIME, &stop) == -1 ) {
      				perror( "clock gettime" );
      				exit( EXIT_FAILURE );
    			}
	
			elapsed_time = ( stop.tv_sec - start.tv_sec) + ( stop.tv_nsec - start.tv_nsec);
			fps = 1/elapsed_time * (1000000000/AVERAGED_FPS_FRAMES); // Hack job, but it should work for now.
			if(fps >= 0){ // returns -1 occassionally.
				placeholder_fps = fps;
				char str[200];
				sprintf(str,"FPS: %.0f",fps);
				putText(silhouette_mask, str, Point(0,dHeight-10), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,0), 2, 8, false);
			}
			fps_index = 1; 
		}else if( fps >= 0 )
		{
			char str[200];
			sprintf(str,"FPS: %.0f",fps);
			putText(silhouette_mask, str, Point(0,dHeight-10), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,0), 2, 8, false);
			fps_index++;
		}else{ // write the fps number in red if the number is strange
			char str[200];
			sprintf(str, "FPS: %.0f", placeholder_fps);
			putText(silhouette_mask, str, Point(0,dHeight-10), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255), 2, 8, false);
			fps_index++;
		}

		// WRITE HELP-TEXT TO WINDOW
		char help_str_1[200], help_str_2[200], help_str_3[200];
		sprintf(help_str_1, "Left-click this window to write a non-human contour.");
		sprintf(help_str_2, "Right-click this window to write a human contour."); 
		sprintf(help_str_3, "Left-click the other window to check the distance.");
		putText(silhouette_mask, help_str_1, Point(0,11), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 2, 8, false);
		putText(silhouette_mask, help_str_2, Point(0,22), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 2, 8, false);
		putText(silhouette_mask, help_str_3, Point(0,33), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 2, 8, false);

		imshow("Silhouette Only", silhouette_mask);
		
		if( ros::ok() ) ros::spinOnce();

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       		{
        		cout << "esc key is pressed by user" << endl;
            		break; 
       		}
	}
	debug.close();
	distances.close();
    	return 0;
}

void ChangeDistance( int, void* )
{
	// FILTER BY DISTANCE, write to filtered_depth_mask
	for(int x = 0; x <= dWidth; x++)
	{
		for(int y = 0; y <= dHeight; y++)
		{
			depth_px = depth_mask.at<float>(y, x);
			if( depth_px >= distance_trackbar/10.0 - DISTANCE_RANGE && depth_px <= distance_trackbar/10.0 + DISTANCE_RANGE )
			{
				line(filtered_depth_mask, Point(x,y), Point(x,y), Scalar(255, 255, 255));
			}
		}
	}
	//clear_depth_mask.convertTo( clear_depth_mask, CV_8UC1, 0.05f);
	//cout << "depth: " << distance_trackbar/10.0 - DISTANCE_RANGE << endl;
}
