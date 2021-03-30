#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
//#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <string>
#include <fstream>

/************************ALGORITHM****************************/
// This algorithm is based on the tutorial non-linear-svms.
// 1. Read from CSV file.
// 2. Feed data, set parameters of SVM object and train.
// 3. Take the SVM response data and create a visual mapping of it,
//    write response to an output csv file.

using namespace cv;
using namespace std;

// NUMBER OF DATA POINTS IN THE CSV FILE
const int DATA_POINTS = 39560; // when you have time, change this so that it automatically reads from csv. Shouldn't be hard.

//struct timespec start_train, stop_train, stop_draw;

int main()
{
    	int width = 700, height = 700;
    	Mat I = Mat::zeros(height, width, CV_8UC3);
	
    	// TRAINING DATA
	int i = 0;
	string value;
	ifstream file_read ( "src/silhouette_detection/svm_data_amir_4.csv" );
	ofstream output;
	output.open( "src/silhouette_detection/output-amir-4.csv" );
    	int labelsArray[DATA_POINTS];
    	float trainingData[DATA_POINTS][2];

	// READ FROM CSV FILE
	while ( getline ( file_read , value, ','))
	{
		trainingData[i][0] = atof(value.c_str());
		getline ( file_read , value, ',');
		trainingData[i][1] = atof(value.c_str())*500;
		getline ( file_read , value);
		labelsArray[i] = atoi(value.c_str());
		i++;
	}
    	Mat trainData(DATA_POINTS, 2, CV_32FC1, trainingData);
    	Mat labels(DATA_POINTS, 1, CV_32SC1, labelsArray);

	// TRAIN SVM
	// GET CLOCK START
	/*if( clock_gettime( CLOCK_REALTIME, &start_train) == -1 ) {
      		perror( "clock gettime" );
      		exit( EXIT_FAILURE );
    	}*/

	cout << "Starting training process" << endl;
	CvSVMParams params;
	params.svm_type = CvSVM::ONE_CLASS;
	params.kernel_type = CvSVM::LINEAR;
	//params.set_nu(0.01);
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	CvSVM SVM;
    	SVM.train(trainData, labels, Mat(), Mat(), params);
    	cout << "Finished training process" << endl;

	/*if( clock_gettime( CLOCK_REALTIME, &stop_train) == -1 ) {
      		perror( "clock gettime" );
      		exit( EXIT_FAILURE );
    	}
	cout << (( stop_train.tv_sec - start_train.tv_sec) + ( stop_train.tv_nsec - start_train.tv_nsec))/1000000000 << " seconds have elapsed." << endl;*/
	// DRAW RESPONSE, OUTPUT TO CSV
    	Vec3b green(0,100,0), blue (100,0,0);
    	for ( int i = 0; i < I.rows; ++i )
    		for ( int j = 0; j < I.cols; ++j )
    	    	{
    	        	Mat sampleMat = (Mat_<float>(1,2) << i, j);
    	        	float response = SVM.predict(sampleMat);
    	        	if      (response == 1)    
			{
				I.at<Vec3b>(-1*j+height, i)  = green;
				output << "1\n";
			}
    	        	else if (response == 0)
			{
				I.at<Vec3b>(-1*j+height, i)  = blue;
				output << "0\n";
			}
    	    	}

    	int thick = -1;
    	int lineType = 8;
    	float px, py;

	// DRAW TRAINING DATA ON TOP OF RESPONSE
    	/*for (int i = 0; i < DATA_POINTS; ++i)
    	{
    	    	px = trainData.at<float>(i,0);
    	    	py = -1*trainData.at<float>(i,1) + height;
		if( labelsArray[i] == 1 ) circle(I, Point( (int) px,  (int) py ), 3, Scalar(0, 255, 0), thick, lineType);
		else if( labelsArray[i] == 0 ) circle(I, Point( (int) px,  (int) py ), 3, Scalar(255, 0, 0), thick, lineType);
    	}*/
	
	// SUPPORT VECTORS
    	/*thick = 2;
    	lineType  = 8;
    	Mat sv = svm->getSupportVectors();
	
    	for (int i = 0; i < sv.rows; ++i)
    	{
    	    	const float* v = sv.ptr<float>(i);
    	    	//circle(	I,  Point( (int) v[0], -1*(int) v[1] + height ), 6, Scalar(128, 128, 128), thick, lineType);
    	}*/
	
    	imwrite("result-chi2-oc-001-amir-4.png", I);	                   // save the Image
	/*if( clock_gettime( CLOCK_REALTIME, &stop_draw) == -1 ) {
      		perror( "clock gettime" );
      		exit( EXIT_FAILURE );
    	}
	cout << (( stop_draw.tv_sec - stop_train.tv_sec) + ( stop_draw.tv_nsec - stop_train.tv_nsec))/1000000000 << " seconds have elapsed." << endl;*/
    	imshow("SVM for Non-Linear Training Data", I); // show it to the user
	file_read.close();
	output.close();
    	waitKey(0);
}	
