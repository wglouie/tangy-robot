/*
 * Frank Despond
 *
 * Version 1.0
 *
 * January 29, 2015
 */
#include "surfdetection.h"

#define NODEBUG
SURFDetection::SURFDetection(string cardFolder, Mat inputImage)
    :minHessian(1800),
      cardOrientation(-1),
      matchedCardMarker(-1)

{
    databaseFolder = cardFolder;
    testImage = inputImage;
    identifyCardMarker();
    std::cout << "Cards in database: ";
    for (std::vector<std::string>::const_iterator i = cardDatabase.begin(); i != cardDatabase.end(); ++i)
    	std::cout << *i << ' ';
}
SURFDetection::~SURFDetection()
{
    printf("\n\nFIN----->SURF");

}


template<typename T, size_t N>
T * end(T (&ra)[N]) {
    return ra + N;
}

const std::string cardDatabaseInit[] = {
    //TODO:------>NO HARDCODING DATA
    //Have to add the new database into here....
    "duck.jpg","train_art.jpg","robot.jpeg", "mickey_mouse.jpeg"

//"duck.jpg","apples.jpg","train_art.jpg","sandwich.jpg", "mickey_mouse.jpeg", "minnie.jpeg","robot.jpeg", "donald_duck.jpeg"

};

std::vector<std::string> SURFDetection::cardDatabase(cardDatabaseInit, cardDatabaseInit + sizeof(cardDatabaseInit) / sizeof(std::string));

int SURFDetection::getMatchedMarker()
{
    return matchedCardMarker;
}

int SURFDetection::getOrientation()
{
    return cardOrientation;
}

void SURFDetection::identifyCardMarker()
{
    for(int i = 0; i < cardDatabase.size(); i++)
    {

        //printf("Database = %s",databaseFolder.c_str());
        Mat cardMarkerDatabase = imread(databaseFolder+cardDatabase[i],1);			//Read card marker
        //imwrite("SURF_TEST_IMAGE.jpg",undistortedImage);
        // fitImageToScreen("SURF_TEST_IMAGE",undistortedImage);
        //waitKey();
        cardOrientation = objectDetect(cardMarkerDatabase);
        if(cardOrientation != -1)
        {
            matchedCardMarker = i;

            printf("Unique Symbol Detected\n");
            printf("Card Marker: %s\n",cardDatabase[i].c_str());
            //printf("\nDetection Result: %i\n",detectionResult);
            break;
        }
    }
}
int SURFDetection::objectDetect(Mat img_object)
{
    SurfFeatureDetector detector( minHessian );

    //-- Step 1: Detect image keypoints --//
    std::vector<KeyPoint> keypoints_object, keypoints_scene;

    detector.detect( img_object, keypoints_object );
    detector.detect( testImage, keypoints_scene );


    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    //imshow("Test", testImage);
    //waitKey();
    Mat descriptors_object, descriptors_scene;

    extractor.compute( img_object, keypoints_object, descriptors_object );
    extractor.compute( testImage, keypoints_scene, descriptors_scene );


    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_object.rows; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;


    for( int i = 0; i < descriptors_object.rows; i++ )
    { if( matches[i].distance < max(2*min_dist, 0.01))
    { good_matches.push_back( matches[i]); }
    }

    Mat img_matches;
    drawMatches( img_object, keypoints_object, testImage, keypoints_scene,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Localize the object from img_1 in img_2
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }

    //CODE CHANGED HERE

    //printf("/n Number of keypoints: %i   %i/n",obj.size(),scene.size());


    if(obj.size() < 4)
        return -1;

    vector<uchar> mask;
    Mat H = findHomography( obj, scene, CV_RANSAC,3.0,mask);					//DONT COMMENT THIS ,3.0,mask
    Mat test = testImage.clone();
    //-- This helps visualize the outliers removed by the RANSAC algorithm --
    for(unsigned int i = 0; i<scene.size();i++) {
        if (mask[i]) // Outlier
            circle(test,scene[i],8,Scalar(0,0,255),3);
    }

    drawMatches( img_object, keypoints_object, testImage, keypoints_scene,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                reinterpret_cast<const vector<char>&> (mask), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );




    //CODE CHANGED HERE


    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( img_object.cols, 0 );
    obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
    obj_corners[3] = cvPoint( 0, img_object.rows );
    std::vector<Point2f> scene_corners(4);


    perspectiveTransform( obj_corners, scene_corners, H);


    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );			//comment out
    line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );			//comment out
    line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );			//comment out
    line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );			//comment out

#ifdef DEBUG
    //fitImageToScreen("SURF Feature Matching",img_matches);
    //imwrite("//home//tangerine//ICRA EXPERIMENTS//Bingo Detection//SURF_FEATURE_MAPPING.jpg",img_matches);
    cv::imwrite("..//Results//surfMatch.jpg",img_matches);
#endif
     //waitKey(10);

    //--Vector cosine rule to determine the orientation of the card from the unique symbol--
    //--a = left edge, b = top edge, d = right edge
    //--angle2 = top left corner, angle3 = top right corner--
    double a = sqrt ((scene_corners[3].y-scene_corners[0].y)*(scene_corners[3].y-scene_corners[0].y)+(scene_corners[3].x-scene_corners[0].x)*(scene_corners[3].x-scene_corners[0].x));

    double b = sqrt ((scene_corners[1].y-scene_corners[0].y)*(scene_corners[1].y-scene_corners[0].y)+(scene_corners[1].x-scene_corners[0].x)*(scene_corners[1].x-scene_corners[0].x));

    double d = sqrt ((scene_corners[2].y-scene_corners[1].y)*(scene_corners[2].y-scene_corners[1].y)+(scene_corners[2].x-scene_corners[1].x)*(scene_corners[2].x-scene_corners[1].x));
    double e = sqrt ((scene_corners[0].y-scene_corners[1].y)*(scene_corners[0].y-scene_corners[1].y)+(scene_corners[0].x-scene_corners[1].x)*(scene_corners[0].x-scene_corners[1].x));

    double angle = atan2(scene_corners[1].y - scene_corners[0].y, scene_corners[1].x - scene_corners[0].x) * 180.0/ CV_PI ;
    double angle2 = acos(((scene_corners[3].y-scene_corners[0].y)*(scene_corners[1].y-scene_corners[0].y) + (scene_corners[3].x-scene_corners[0].x)*(scene_corners[1].x-scene_corners[0].x))/(a*b))* 180.0/ CV_PI;
    double angle3 = acos(((scene_corners[2].y-scene_corners[1].y)*(scene_corners[0].y-scene_corners[1].y) + (scene_corners[2].x-scene_corners[1].x)*(scene_corners[0].x-scene_corners[1].x))/(e*d))* 180.0/ CV_PI;
    /*
     printf("the point 0 is x=%f   y=%f\n",scene_corners[0].x,scene_corners[0].y);
     printf("the point 1 is x=%f   y=%f\n",scene_corners[1].x,scene_corners[1].y);
     printf("the point 2 is x=%f   y=%f\n",scene_corners[2].x,scene_corners[2].y);
     printf("the point 3 is x=%f   y=%f\n",scene_corners[3].x,scene_corners[3].y);
     printf("-- Rotated Angle : %f \n ", angle);
     printf("------the first angle between points is %f \n", angle2);
     printf("------the second angle between points is %f \n", angle3);
     */

    if ( angle2 > 70 && angle2 < 110 && angle3 > 70 && angle3 < 110){
        //fitImageToScreen("Successful Unique Marker Detection",img_matches);
        //resize(img_matches,img_matches,Size(640,480),0,0,INTER_LINEAR);
        //imshow("Successful Unique Marker Detection",img_matches);

        if(scene_corners[3].y-scene_corners[0].y > 0 && scene_corners[2].y-scene_corners[1].y > 0 && abs (scene_corners[1].y-scene_corners[0].y) < 30 )
        {
            printf("Card is Straight\n");
            return 0;	//Straight
        }
        if(scene_corners[1].y-scene_corners[0].y > 0 && scene_corners[2].y-scene_corners[3].y > 0 && abs (scene_corners[0].y-scene_corners[3].y) < 30 )
        {
            printf("Card is turned Right\n");
            return 1;	//Turned right
        }
        if(scene_corners[0].y-scene_corners[3].y > 0 && scene_corners[1].y-scene_corners[2].y > 0 && abs (scene_corners[3].y-scene_corners[2].y) < 30 )
        {
            printf("Card is turned Down\n");
            return 2;	//Turned down
        }
        if(scene_corners[0].y-scene_corners[1].y > 0 && scene_corners[3].y-scene_corners[2].y > 0 && abs (scene_corners[0].y-scene_corners[3].y) < 30 )
        {
            printf("Card is turned Left\n");
            return 3;	//Turned left
        }
    }
    else{
        return -1;		//Not detected
    }

}
