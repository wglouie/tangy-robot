#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgcodecs/imgcodecs.hpp" // don't think it is needed in 2.4.8, located in core
#include "opencv2/imgproc/imgproc.hpp"
//#include "stdafx.h" // I wonder if this is needed yet...
//#include "opencv2/videoio/videoio.hpp" //test
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

// I have never cleaned up the #includes.... some may be not needed.

using namespace cv;
using namespace std;

/******************ALGORITHM********************/
/// Read from Kinect.
/// Find the depth mask seen by the kinect
/// Keep only depths within a certain range.
/// Identify contours.
/// Eliminate any shapes that are not big enough to be a silhouette and draw a centroid on them.
/// Calculate the euclidean distance from each point on each contour to its respective centroid.
/// Isolate top-most data points of each contour using ISO_LINE_1 and ISO_LINE_2 (white lines in SVM_Map)
/// Compare these distances to an SVM map.
/// Write the percentage of the isolated data points for a contour that are likely human for each contour, identify which SVM fits best

// CONSTANTS
const float DISTANCE_RANGE = 3.0; // The variation from DISTANCE where we are looking for a silhouette
const int MIN_AREA_CONTOUR = 4000; // The minimum area that we expect the silhouette to have
const int AVERAGED_FPS_FRAMES = 10;
const int OUTPUT_DOMAIN = 700; // The final domain of the data points for each contour from 0 to OUTPUT_DOMAIN
const int OUTPUT_RANGE = 500; // The final range of the data points for each contour from 0 to OUTPUT_RANGE
const int MAP_WIDTH = 700; // The width array size in the input csv file
const int MAP_LENGTH = 700; // The length array size in the input csv file
const int NUM_MAPS = 4; // The number of SVM maps
const char CSV_NAME[200] = "src/silhouette_detection/svm_map.csv"; // Name of the CSV file without the number at the end
const int ISO_LINE_1 = 95; // distance points with index less than this are used to calculate humanity
const int ISO_LINE_2 = 550; // distance points with index greater than this are used to calculate humanity
const float KINECT_FOV = 0.994837674; // the FOV of the Kinect camera in radians
const float PREF_DIST_TO_TARGET = 1.0; // How close we want to follow the target
const int DWIDTH = 640; // constants for camera width and height, placeholder for now
const int DHEIGHT = 480;

// IMAGES
Mat depth_mask; // The raw depth image from the kinect
Mat clear_depth_mask; // A clearer depth image, showing the variation in depth better.
Mat range_mask;
Mat filtered_depth_mask; // The binary image containing only the depth range we want
Mat silhouette_mask; // The final image showing possible silhouettes for identification
Mat bgr_mask; // The BGR image from the kinect
ifstream svm_map(CSV_NAME); // The output file from silhouette_svm

// VARIABLES
double elapsed_time, fps, placeholder_fps; // fps-related floating point numbers
int fps_index = 1; // index of number of frames since the fps was updated for fps averaging
unsigned short depth; // raw depth value from the kinect
//float depth_px; // depth value in metres, derived from depth.
bool record_dist = true; // set to false when we don't want to identify human contours
int human_tag = 0; // 0 for non-human, 1 for human
int label_map [NUM_MAPS][MAP_WIDTH][MAP_LENGTH];
//int frame = 0;
//float checked_distance = 0.5; // The average depth distance where you expect the target

// TEMP
int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;

void Erosion( int, void* );
void Dilation( int, void* );

struct timespec stop, start; // fps-related timers	

void depthROS(const sensor_msgs::Image::ConstPtr& msg)
{
	try {
        	cv_bridge::CvImageConstPtr cv_ptr;
        	cv_ptr = cv_bridge::toCvShare(msg);
		// do any conversions here, currently, none
		/*for ( int x = 0; x < DWIDTH; x++ )
		{
			for ( int y = 0; y < dHeight; y++ )
			{
				//cout << "writing" << endl;
				cv_ptr->image.at<float> (y, x) = (float)cv_ptr->image.at<unsigned short>( y, x )/1000.0f;
			}
		}*/
		cv_ptr->image.convertTo(depth_mask, CV_32FC1, 1.0, 0);
		//cout << cv_ptr->image.at<float>( DWIDTH/2, DHEIGHT/2 ) << endl;
		//cout << depth_mask.at<float>( DWIDTH/2, DHEIGHT/2 ) << endl;
		//cout << depth_mask.type() << endl;
    	} catch (const cv_bridge::Exception& e) {
        	ROS_ERROR("cv_bridge exception: %s", e.what());
    	}
}

void bgrROS(const sensor_msgs::Image::ConstPtr& msg)
{
	vector<Mat> rgb, bgr; // ROS publishes an RGB image, but opencv writes a BGR image, so we need to swap channels
	try {
        	cv_bridge::CvImageConstPtr cv_ptr;
        	cv_ptr = cv_bridge::toCvShare(msg);
		// do any conversions here
		cv_ptr->image.convertTo(bgr_mask, CV_8UC3, 1.0, 0);
		split(bgr_mask, rgb);
		bgr.push_back(rgb[2]);// write channels in reverse order
		bgr.push_back(rgb[1]);
		bgr.push_back(rgb[0]);
		merge(bgr, bgr_mask);
    	} catch (const cv_bridge::Exception& e) {
        	ROS_ERROR("cv_bridge exception: %s", e.what());
    	}
}

int main(int argc, char* argv[])
{
	double dWidth = DWIDTH;
	double dHeight = DHEIGHT;
	depth_mask = Mat::zeros(Size(dWidth, dHeight), CV_32FC1);
	bgr_mask = Mat::zeros(Size(dWidth, dHeight), CV_8UC3);
	//cout << getBuildInformation() << endl;
	// ROS PUBLISHER INITIALIZATION
	ros::init(argc, argv, "silhouettes_publisher");
	ros::NodeHandle n;
	ros::Publisher x_pub = n.advertise<std_msgs::String>("sil_x", 1000);
	ros::Publisher distance_pub = n.advertise<std_msgs::String>("sil_distance", 1000);
	ros::Subscriber kinect_sub = n.subscribe("/camera/depth/image", 1000, depthROS);
	ros::Subscriber bgr_sub = n.subscribe("/camera/rgb/image_raw", 1000, bgrROS);
	ros::Rate loop_rate(10);
	float x_to_track = 0; // the x_value of the centroid of the silhouette we want to publish
	float dist_diff = 0; // the distance between the robot and the target
	float theta = 0; // the angle in between the target and the robot in radians

	// FILL label_map
	string value;
	for( int h = 0; h < NUM_MAPS; h++)
	{
		for( int i = 0; i < MAP_WIDTH; i++ )
		{
			for( int j = 0; j < MAP_LENGTH; j++)
			{
				getline ( svm_map , value);
				label_map[h][i][j] = atoi( value.c_str() );
				//cout << label_map[h][i][j] << endl;
			}
		}
	}

	namedWindow("range_mask", CV_WINDOW_AUTOSIZE);
	// TEMP
	/// Create Erosion Trackbar
  	createTrackbar( "Erosion: \tElement:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Final", &erosion_elem, max_elem, Erosion );
	createTrackbar( "Erosion: \tKernel size:\n 2n +1", "Final", &erosion_size, max_kernel_size, Erosion );

	/// Create Dilation Trackbar
	createTrackbar( "Dilation \tElement:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Final", &dilation_elem, max_elem, Dilation );
	createTrackbar( "Dilation \tKernel size:\n 2n +1", "Final", &dilation_size, max_kernel_size, Dilation );

	//Mat erosion_element = getStructuringElement( MORPH_ELLIPSE, Size(2*EROSION_SIZE + 1, 2*EROSION_SIZE  + 1), Point(EROSION_SIZE , EROSION_SIZE ) );
	//Mat dilation_element = getStructuringElement( MORPH_RECT, Size(2*DILATION_SIZE + 1, 2*DILATION_SIZE + 1), Point(DILATION_SIZE, DILATION_SIZE) );
	namedWindow("filtered_depth_mask", CV_WINDOW_AUTOSIZE);
	namedWindow("bgr_mask",CV_WINDOW_AUTOSIZE);
	namedWindow("silhouette_mask",CV_WINDOW_AUTOSIZE);
	namedWindow("SVM Map",CV_WINDOW_AUTOSIZE);
	// RECREATE SVM VISUAL MAP
	Mat I(MAP_LENGTH, MAP_WIDTH, CV_8UC3);
	Vec3b green(0,100,0), blue (100,0,0);
	
	// CREATE WEBCAM
	/*VideoCapture cap(CV_CAP_OPENNI); // open kinect camera
	cap.set( CV_CAP_PROP_OPENNI_REGISTRATION , 0);
    	if (!cap.isOpened()) // if not success, exit program
	{
        	cout << "Cannot open the video cam" << endl;
        	return -1;
	}

   	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); // get the width of frames of the video
   	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); // get the height of frames of the video
	cout << "Frame size : " << dWidth << " x " << dHeight << endl;*/

    	while (1)
	{
		// GET CLOCK START
		if( clock_gettime( CLOCK_REALTIME, &start) == -1 ) {
      			perror( "clock gettime" );
      			exit( EXIT_FAILURE );
    		}

		// PAINT AN EMPTY SVM_MAP
		for ( int i = 0; i < I.rows; i++ )
		{
	    		for ( int j = 0; j < I.cols; j++ )
	    	    	{
	    	        	//Mat sampleMat = (Mat_<float>(1,2) << i, j);
				for( int k = 0; k < NUM_MAPS; k++ )
				{
					if ( label_map[k][i][j] == 1)	
					{
						I.at<Vec3b>( -1*j+MAP_LENGTH-1, i )  = green;
						break;
					}
				}
				if( I.at<Vec3b>( -1*j+MAP_LENGTH-1, i ) != green )	I.at<Vec3b>( -1*j+MAP_LENGTH-1 , i)  = blue;
	    	    	}
		}

		// SHOW ISOLATION LINES
		line(I, Point(ISO_LINE_1, 0), Point(ISO_LINE_1, MAP_LENGTH), Scalar(255,255,255), 3);
		line(I, Point(ISO_LINE_2, 0), Point(ISO_LINE_2, MAP_LENGTH), Scalar(255,255,255), 3);

		// READ FROM WEBCAM
		filtered_depth_mask = Mat::zeros(Size(dWidth, dHeight), CV_8UC1);
		depth_mask.convertTo( clear_depth_mask, CV_32FC1, 255/DISTANCE_RANGE); // for imshow
		//cout << clear_depth_mask.at<unsigned short>( DWIDTH/2, DHEIGHT/2 ) << endl;
		//imshow("range_mask", clear_depth_mask);
		float depth_px;
		range_mask = Mat::zeros(Size(dWidth, dHeight), CV_8UC1);
		for(int x = 0; x <= dWidth; x++)
		{
			for(int y = 0; y <= dHeight; y++)
			{
				depth_px = clear_depth_mask.at<float>(y, x);
				//cout << depth_px << endl; // TEST_5
				if( depth_px <= 255 ) // values within DISTANCE_RANGE
				{
					line(range_mask, Point(x,y), Point(x,y), Scalar(depth_px));
					//rectangle(range_mask, Point(dWidth,dHeight), Point(dWidth+50,dHeight+50), Scalar(255));
				}
			}
		}
		//cout << clear_depth_mask.at<float>(dHeight/2, dWidth/2) << endl;
		// TEMP
		Erosion(0,0);
		Dilation(0,0);		
		//erode( clear_depth_mask, clear_depth_mask, erosion_element );
		//dilate( clear_depth_mask, clear_depth_mask, dilation_element );
		medianBlur(range_mask,range_mask, 23);
		imshow("range_mask", range_mask);

		/*// FILTER BY DISTANCE, write to filtered_depth_mask
		for(int x = 0; x <= dWidth; x++)
		{
			for(int y = 0; y <= dHeight; y++)
			{
				depth_px = (float)depth_mask.at<unsigned short>(y, x)/1000.0f;
				if( depth_px >= 0.75+checked_distance*frame - DISTANCE_RANGE && depth_px <= 0.75+checked_distance*frame + DISTANCE_RANGE )
				{
					line(filtered_depth_mask, Point(x,y), Point(x,y), Scalar(255, 255, 255));
				}
			}
		}*/
		//clear_depth_mask.convertTo( clear_depth_mask, CV_8UC1, 0.05f);
		adaptiveThreshold(range_mask, filtered_depth_mask, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 9, 9);// not needed anymore
		//line(filtered_depth_mask, Point(0, dHeight), Point(dWidth, dHeight), Scalar(255), 9);// Draw a line at the bottom to close any open contours
		imshow("filtered_depth_mask", filtered_depth_mask);

		// CONTOURS
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		findContours(range_mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

		vector<Moments> mu( contours.size() );
		vector<Point2f> mc( contours.size() );
		float percentages[ contours.size() ]; // array of largest percentage for each contour
		float largest_percent = 0; // largest percent found out of all contours
		silhouette_mask = Mat::zeros( Size(dWidth, dHeight), CV_8UC3);
		
		// DRAW CONTOURS
		for( int i = 0; i < contours.size(); i++ )
		{
			if( contourArea(contours[i]) >= MIN_AREA_CONTOUR)
			{
				mu[i] = moments( contours[i], false );
				mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
				drawContours( silhouette_mask, contours, i, Scalar(0,0,255), 2, 8, hierarchy, 0, Point() );
				circle( silhouette_mask, mc[i], 4, Scalar(0,255,0), -1, 8, 0 );
				float normalized_index[contours[i].size()-1];
				float normalized_distance[contours[i].size()-1];
				int human_pts [NUM_MAPS];
				int calculation_points = 0;
				float human_percent [NUM_MAPS];
				int min_x, max_x, min_y, max_y;
				min_x = 1000000;
				min_y = 1000000;
				max_x = 0;
				max_y = 0;
				for( int h = 0; h < NUM_MAPS; h++ )
				{
					human_pts[h] = 0;
				}
				if( record_dist )
				{
					int distance[contours[i].size()-1];
					int max_distance = 0;

					for( int j = 0; j < contours[i].size(); j++ ) // find maximum distance before we can modify the range
					{
						distance[j] = sqrt(pow(contours[i][j].x-mc[i].x,2)+pow(contours[i][j].y-mc[i].y,2));

						if( distance[j] > max_distance ) max_distance = distance[j];
						if( contours[i][j].x > max_x ) max_x = contours[i][j].x;
						if( contours[i][j].y > max_y ) max_y = contours[i][j].y;
						if( contours[i][j].x < min_x ) min_x = contours[i][j].x;
						if( contours[i][j].y < min_y ) min_y = contours[i][j].y;
					}					

					for( int k = 0; k < contours[i].size(); k++ )
					{
						normalized_index[k] = ((float)k+1)/(float)contours[i].size()*OUTPUT_DOMAIN;
						normalized_distance[k] = (float)distance[k]/(float)max_distance * OUTPUT_RANGE;
					}
				}

				// CALCULATE THE HUMANITY
				for( int k = 0; k < contours[i].size(); k++ )
				{
					line(I, Point((int)normalized_index[k], -1*((int)normalized_distance[k])+MAP_LENGTH), Point((int)normalized_index[k], -1*((int)normalized_distance[k])+MAP_LENGTH), Scalar(0,0,255), 3);
					if( (int)normalized_index[k] <= ISO_LINE_1 || (int)normalized_index[k] >= ISO_LINE_2 ) calculation_points++; 
					for( int l = 0; l < NUM_MAPS; l++ )
					{
						if( label_map[l][(int)normalized_index[k]][(int)normalized_distance[k]] == 1 && ((int)normalized_index[k] <= ISO_LINE_1 || (int)normalized_index[k] >= ISO_LINE_2 ))
						{
							line(I, Point((int)normalized_index[k], -1*((int)normalized_distance[k])+MAP_LENGTH), Point((int)normalized_index[k], -1*((int)normalized_distance[k])+MAP_LENGTH), Scalar(255,255,255), 3);
							//cout << human_pts[l] << endl;
							human_pts[l] = human_pts[l] + 1;
							break;
						}
					}
				}
				
				// PRINT LARGEST PERCENTAGE
				percentages[i] = 0;
				int largest_map = 0;
				for( int k = 0; k < NUM_MAPS; k++ ) 
				{
					human_percent[k] = (float)human_pts[k]/(float)calculation_points * 100;
					//cout << "Human Pts: " << human_pts[k] << "\n" << "Calculation Points: " << calculation_points << endl;
					//cout << "k: " << k << endl;
					if( human_percent[k] > percentages[i] )
					{
						percentages[i] = human_percent[k];
						largest_map = k;
					}
				}
				char percent_str[200];
				//cout << percentages[i] << " " << largest_map+1 << endl;
				sprintf(percent_str, "%.0f%% position %i", percentages[i], largest_map+1);
				putText(bgr_mask, percent_str, mc[i], FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,0), 6, 8, false);
				rectangle(bgr_mask, Point(min_x, min_y), Point(max_x, max_y), Scalar(0,255,0)); // DRAW A RECTANGLE AROUND CONTOURS
				
				// Find what to publish to ROS
				if( percentages[i] > largest_percent )
				{
					largest_percent = percentages[i];
					x_to_track = mc[i].x-dWidth/2;
					dist_diff = depth_mask.at<float>(mc[i].y, mc[i].x) - PREF_DIST_TO_TARGET;
					theta = atan((x_to_track*tan(KINECT_FOV/2))/(dWidth/2));
				}			
			}
		}

		// GET FPS ----- BROKEN
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
				putText(bgr_mask, str, Point(0,dHeight-10), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,0), 2, 8, false);
			}
			fps_index = 1;
		}else if( fps >= 0 )
		{
			char str[200];
			sprintf(str,"FPS: %.0f",fps);
			putText(bgr_mask, str, Point(0,dHeight-10), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,0), 2, 8, false);
			fps_index++;
		}else{ // write the fps number in red if the number is strange
			char str[200];
			sprintf(str, "FPS: %.0f", placeholder_fps);
			putText(bgr_mask, str, Point(0,dHeight-10), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255), 2, 8, false);
			fps_index++; 
		}
		//imshow("Silhouette Only", silhouette_mask);

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       		{
        		cout << "esc key is pressed by user" << endl;
            		break; 
       		}
		imshow("bgr_mask", bgr_mask);
		imshow("silhouette_mask", silhouette_mask ); 
		imshow("SVM Map", I);

		// PUBLISH TO ROS <- screws up my matrices for some reason, hence why it is outside of the calculated fps frames because they use bgr_mask to write to
		// Specifically, publish the the distance and angle that the robot needs to reach the correct position in relation to the highest percentage contour it sees.
		if( ros::ok() )
		{
			// PUBLISH X
			std_msgs::String msg_x;
			std::stringstream ss_x;
			ss_x << theta;
			msg_x.data = ss_x.str();
			ROS_INFO("%s", msg_x.data.c_str());
			x_pub.publish(msg_x);

			// PUBLISH DISTANCE
			std_msgs::String msg_dist;
			std::stringstream ss_dist;
			ss_dist << dist_diff;
			msg_dist.data = ss_dist.str();
			ROS_INFO("%s", msg_dist.data.c_str());
			distance_pub.publish(msg_dist);
			ros::spinOnce();
			loop_rate.sleep();
		}
		//cout << largest_percent << endl;
	}
	//cap.release();
    	return 0;
}

// TEMP
/**  @function Erosion  */
void Erosion( int, void* )
{
	int erosion_type;
	if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
	else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
	else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

	Mat erosion_element = getStructuringElement( erosion_type, Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );
	erode( range_mask, range_mask, erosion_element );
}

/** @function Dilation */
void Dilation( int, void* )
{
  	int dilation_type;
  	if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  	else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  	else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  	Mat dilation_element = getStructuringElement( dilation_type, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
	dilate( range_mask, range_mask, dilation_element );
}
