#include <help_indicators/help_indicators.h>
// #define DEBUG_HELP_INDICATORS
#define KINECT_CALI
using namespace cv;

vector<help_indicators::triangle> Help::find_triangles(const sensor_msgs::ImageConstPtr& ir_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(ir_msg, sensor_msgs::image_encodings::TYPE_8UC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    Mat img = cv_ptr->image;
    Mat test = Mat::zeros(img.rows,img.cols, CV_8UC3);
    #ifdef DEBUG_HELP_INDICATOR
    imshow("Raw Image",img);
    #endif

	//Threshold IR values for sensible image visualization (greater than 1 is white)
    #ifdef KINECT_CALI
    for (int i = 0; i < img.cols; i++) {
            for (int j = 0; j < img.rows; j++) {
	        Vec3b &intensityTarget = test.at<Vec3b>(j, i);       
                if (img.at<unsigned char>(j, i) >1) {
                    img.at<unsigned char>(j, i) = 255;
                } else {
                    img.at<unsigned char>(j, i) = 0;
                }
            }
        }
    #else
    for (int i = 0; i < img.cols; i++) {
        for (int j = 0; j < img.rows; j++) {
	    Vec3b &intensityTarget = test.at<Vec3b>(j, i);     
            if (img.at<unsigned char>(j, i) >1 && j > 170 && j < 220) {
                img.at<unsigned char>(j, i) = 255;
            } else {
                img.at<unsigned char>(j, i) = 0;
            }

        }
    }
    #endif
   #ifdef DEBUG_HELP_INDICATOR
    imshow("filtered Image",img);
    //imshow("colored Image",test);
    #endif

    //Blur and Dilate the image to remove noise while filling in holes between points
    Mat kernel = (Mat_<uchar>(3,3) << 1,1,1,1,1,1);
    // erode(img,img,kernel,Point(-1,-1),1,1,morphologyDefaultBorderValue());
    #ifdef DEBUG_HELP_INDICATOR
    // imshow("Image Erode", img);
    // waitKey(30);
    #endif   	
 		
    // dilate(img,img,kernel,Point(-1,-1),1,1,morphologyDefaultBorderValue());
    #ifdef DEBUG_HELP_INDICATOR
    // imshow("Dilate", img);
    #endif   
    //Speckle filtering 	 		
    medianBlur(img,img, 5);
    #ifdef DEBUG_HELP_INDICATOR
    imshow("Median Blur", img);
    #endif
   	 		  
    //Adaptive threshold acts like edge detection
    adaptiveThreshold(img, img, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 9, 9);

    pub_filteredIR.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, img).toImageMsg());

    //Capture triangle using the hough lines methodology
    //Hough lines for the triangle
    vector<Vec4i> lines;     

    //Find lines in triangle - try to connect some (if problem finding grid lines change values)  
    HoughLinesP(img, lines, 1, CV_PI/180,10, 20, 5);		    

    //Draw those guessed lines on an image
    Mat cdst;
    cdst = Mat::zeros(img.size(), CV_8UC3);
    Mat triangles =  Mat::zeros(img.size(), CV_8UC3);
    cvtColor(img, triangles, CV_GRAY2BGR);

    for( size_t i = 0; i < lines.size(); i++ ) {
        Vec4i l = lines[i];
        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 12, 8);

    }
    #ifdef DEBUG_HELP_INDICATOR
    imshow("Hough Lines", cdst);
    #endif

    vector< vector<Point> > triCheck;
    //Find triangles
    findSquares4(cdst,triCheck);			 	    

    vector<help_indicators::triangle> new_triangles; 
    if(triCheck.size() > 0) {
        for(int i = 0; i<triCheck.size(); i++) {
	    std::vector<Point> tri = triCheck[i];
            line(triangles, tri[0], tri[1], Scalar(0,255,0),4);
            line(triangles, tri[1], tri[2], Scalar(0,255,0),4);
            line(triangles, tri[2], tri[0], Scalar(0,255,0),4);

            int x = ((tri[0].x +tri[1].x +tri[2].x)/3);
            int y = ((tri[0].y +tri[1].y +tri[2].y)/3);

	    help_indicators::triangle new_triangle;
	    new_triangle.x = x;
	    new_triangle.y = y; 
	    new_triangle.added = false;
	    for(int i = 0; i < 19; i++) {
	        new_triangle.seen.push_back(false);
	    }
	    new_triangle.seen.push_back(true);
	    new_triangles.push_back(new_triangle);
        }
    }
    // bring up window with triangles
    imshow("Image", triangles);
    waitKey(30);
    //ROS_INFO("Find tirangles");
    //Mat rgb_overlay_image = overlay_detection_image(triangles);
    //imshow("Image", rgb_overlay_image);
    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_overlay_image).toImageMsg();
    //pub_detection_image.publish(msg);
    return new_triangles;
}

/*
void Help::add_new_triangle_cb(const help_indicators::triangle::ConstPtr& msg){
    ROS_INFO("Adding triangle that has been manually generated");
    help_indicators::triangle triangle;
    count++;
    triangle.name = name + boost::lexical_cast<std::string>(count);
    triangle.seen = msg->seen;
    triangle.x = msg->x;
    triangle.y = msg->y;
    triangle.z = msg->z;
    triangle.added = msg->added;
    transformed_help_indicators.push_back(triangle);

    ROS_INFO("There are now [%d] triangles", transformed_help_indicators.size());
}
*/

/*
void Help::rgb_image_cb(const sensor_msgs::ImageConstPtr& msg){
    try
    {
      rgb_image = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

*/

cv::Mat Help::overlay_detection_image(cv::Mat &triangle_image){
    cv::Mat result = rgb_image.clone();

    for(int i=0; i<triangle_image.cols; i++){
        for(int j=0; j<triangle_image.rows; j++){
            Vec3b &intensity = triangle_image.at<Vec3b>(j,i);
            if(intensity.val[0] != 0 || intensity.val[1] != 0 || intensity.val[2] != 0){
                if(intensity.val[0] != 255 || intensity.val[1] != 255 || intensity.val[2] != 255)
                result.at<Vec3b>(j,i) = triangle_image.at<Vec3b>(j,i);
            }
        }
    }

    return result;
}


vector<help_indicators::triangle> Help::find_color_triangles(const sensor_msgs::ImageConstPtr& ir_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(ir_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    Mat img = cv_ptr->image;

    for (int i = 0; i < img.cols; i++) {
        for (int j = 0; j < img.rows; j++) {
            Vec3b &inten = img.at<Vec3b>(j, i);
            float blue = inten.val[0];
            float green = inten.val[1];
            float red = inten.val[2];

            float redGreen = red/green;
            float redBlue = red/blue;
    
            float greenRed = green/red;
            float greenBlue = green/blue;

            if(greenBlue < 1.2 || greenRed < 1.2) {
                inten.val[0] = 0;
                inten.val[1] = 0;
                inten.val[2] = 0;
            }
        }
    }

    cvtColor(img, img, CV_BGR2GRAY , 0 );
    imshow("Processed", img);
    waitKey(30);	

    //Blur and Dilate image to remove noise while filling in holes between points
    Mat kernel = (Mat_<uchar>(3,3) << 1,1,1,1,1,1,1,1,1);
    erode(img,img,kernel,Point(-1,-1),1,1,morphologyDefaultBorderValue());
    dilate(img,img,kernel,Point(-1,-1),1,1,morphologyDefaultBorderValue());
    //Adaptive threshold acts like edge detection
    adaptiveThreshold(img, img, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 9, 9);

    imshow("thresh", img);
    waitKey(30);
    //Capture triangle using the hough lines methodology
    //Hough lines for the triangle
    vector<Vec4i> lines;
    //Find lines in the triangle - try to connect some (if problem finding grid lines change values)
    HoughLinesP(img, lines, 1, CV_PI/180,10, 20, 5);

    //Draw those guessed lines on an image
    Mat cdst;
    cdst = Mat::zeros(img.size(), CV_8UC3);
    Mat triangles =  Mat::zeros(img.size(), CV_8UC3);

    for( size_t i = 0; i < lines.size(); i++ ) {
        Vec4i l = lines[i];
        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, 8);
    }
    vector< vector<Point> > triCheck;
    //Find triangles
    findSquares4(cdst,triCheck);

    vector<help_indicators::triangle> new_triangles;
    if(triCheck.size() > 0) {
        for(int i = 0; i<triCheck.size(); i++) {
            std::vector<Point> tri = triCheck[i];
            line(triangles, tri[0], tri[1], Scalar(0,255,0),4);
            line(triangles, tri[1], tri[2], Scalar(0,255,0),4);
            line(triangles, tri[2], tri[0], Scalar(0,255,0),4);

            int x = ((tri[0].x +tri[1].x +tri[2].x)/3);
            int y = ((tri[0].y +tri[1].y +tri[2].y)/3);

            help_indicators::triangle new_triangle;
            new_triangle.x = x;
            new_triangle.y = y;
            new_triangle.added = false;
            for(int i = 0; i < 19; i++) {
                new_triangle.seen.push_back(false);
            }
            new_triangle.seen.push_back(true);
            new_triangles.push_back(new_triangle);
        }
    }
    // bring up window with triangles
    imshow("Image", triangles);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", triangles).toImageMsg();
    pub_detection_image.publish(msg);
    waitKey(30);

    return new_triangles;
}
