//
//  main_new.cpp
//
//
//  Created by Frank Despond on 2014-11-02.
//
///////////////////////////////
/*
 
 This has been edited for adding a second goal for the bingo game.
 
 the second goal is to get the game type (lines, corners, etc)

 
 */
///////////////////////////////

//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
//Include headers for OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//Include headers for ROS services
#include <actionlib/server/simple_action_server.h>
#include <bingo_detection/BingoDetectionAction.h>

#include "Bingo.h"
#include <string.h>
#include <fstream>


namespace enc = sensor_msgs::image_encodings;

//static const char WINDOW[] = "Image being tested";
class BingoDetectionAction
{
    ros::NodeHandle nh_;
    
    /*Camera grab variables*/
    std::string cameraTopic;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    
    /*Action library variables*/
    actionlib::SimpleActionServer<bingo_detection::BingoDetectionAction> as_;
    std::string action_name_;
    bingo_detection::BingoDetectionGoal goal_;
    bingo_detection::BingoDetectionFeedback feedback_;					//Provides feedback on whether card is detected
    bingo_detection::BingoDetectionResult result_;						//Provides feedback on the result of detection
    std::vector<int> calledNumbers;
    int gameType;
    
    /*Bingo detection variables*/
    Bingo bingoStuff;
    std::string cardDatabaseFolder;
    std::string numberDatabaseFile;
    
    /*For saving results */
    std::ofstream resultFile;
    int detectionCounter;
public:
    BingoDetectionAction(std::string name)
    : as_(nh_, name, false),	//Initialize action server
    action_name_(name),
    it_(nh_)										//Initialize image transport
    {
        /* Action library initialize */
        as_.registerGoalCallback(boost::bind(&BingoDetectionAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&BingoDetectionAction::preemptCB, this));
        as_.start();
        
        /* Camera grab stuff */
        image_pub_ = it_.advertise("out", 1);							//Advertise output images
        image_sub_ = it_.subscribe("in", 1, &BingoDetectionAction::imageCb, this);		//Subscribe to camera images
        //cv::namedWindow(WINDOW);
        
        /* Bingo detection stuff */
        nh_.getParam("bingo_detection_server/cardDatabaseFolder", cardDatabaseFolder);			//Grab album directory from parameter server
        nh_.getParam("bingo_detection_server/numberDatabaseFile", numberDatabaseFile);
       // ROS_INFO("/cardDatabaseFolder = [%s]", cardDatabaseFolder.c_str());
        //printf("Database Folder: %s\n",numberDatabaseFile.c_str());
        bingoStuff = Bingo(cardDatabaseFolder,numberDatabaseFile);						//Initialize Bingo class
        /*For saving results*/
        resultFile.open ("//bingoTestResults//BingoDetectionResults.txt",std::ofstream::app);
        detectionCounter = 0;
    }
    
    ~BingoDetectionAction()
    {
        resultFile.close();
        //cv::destroyWindow(WINDOW);
    }
    /* Execute this code when images are placed on the advertised topic */
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        if(!as_.isActive())
            return;
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //Uncomment for streaming Bingo detection
        //Testing reducing highlights
            Mat image = cv_ptr->image;

            //Mat image = imread("/home/frank/catkin_ws/src/bingoDetection/TestImage.jpg",1); //For testing without cameras
            //Mat image = imread("/home/frank/BingoCardHSV.jpg",1); //For testing without cameras

          //  std::string imageResult;       		   // string which will contain the result
           // std::ostringstream imageConvert;  			   // stream used for the conversion
            //imageConvert << detectionCounter;     // insert the textual representation of 'Number' in the characters in the stream
            //imageResult = imageConvert.str(); // set 'Result' to the contents of the stream
            //cv::imwrite(("..//Results//" + imageResult + ".jpg").c_str(),cv_ptr->image);




            bingoStuff.inputImage = image;

            //bingoStuff.fitImageToScreen("Image being tested",bingoStuff.inputImage);

            //Show image being tested



            //Detect bingo card
            int cardState;
            Point3f locationCard;
            cardState = bingoStuff.detectBingoCard(calledNumbers, locationCard, gameType);
            ROS_INFO("x %f, y %f, z %f \n", locationCard.x, locationCard.y, locationCard.z);
            detectionCounter++;
        if(cardState > -1)//sees card
        {
            resultFile.open ("//bingoTestResults//BingoDetectionResults.txt",std::ofstream::app);
            Convert << imageResult;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            result_.bingoCardState = cardState;
            result_.missingNumbers = bingoStuff.missingNumbers;
            result_.wrongNumbers = bingoStuff.wrongNumbersMarked;
            
            /*Save results to file */
            std::string Result;       		   // string which will contain the result
            std::ostringstream Convert;  		   // stream used for the conversion
            Convert << "Detection Counter: " << detectionCounter << "\n";
            Convert << "State: " << cardState << "\n";
            
            //Called numbers
            Convert << "Called Numbers: ";
            for(int i = 0; i<calledNumbers.size(); i++)
                Convert << calledNumbers[i] << ", ";
            Convert << "\n";
            
            //Numbers Marked Incorrectly
            Convert << "Numbers marked incorrectly: ";
            for(int i = 0; i<bingoStuff.wrongNumbersMarked.size(); i++)
                Convert << bingoStuff.wrongNumbersMarked[i] << ", ";
            Convert << "\n";
            
            //Missing Number
            Convert << "Numbers missing: ";
            for(int i = 0; i<bingoStuff.missingNumbers.size(); i++)
                Convert << bingoStuff.missingNumbers[i] << ", ";
            Convert << "\n\n";
            
            Result = Convert.str(); // set 'Result' to the contents of the stream
            resultFile << Result;
            /*Save results to file */
            
            feedback_.cardDetected = 1;
            as_.publishFeedback(feedback_);
            as_.setSucceeded(result_);
            
            
            8as_.publishFeedback(feedback_);
            resultFile.close();
        }
        else //does not see card
        {
            bingo_detection::BingoDetectionGoal goal_;
           /* resultFile.open ("//bingoTestResults//BingoDetectionResults.txt",std::ofstream::app);
            Save results to file 
            std::string Result;       		   // string which will contain the result
            std::ostringstream Convert;  		   // stream used for the conversion
            Convert << "Detection Counter: " << detectionCounter << "\n";
            Convert << "State: " << cardState << "\n\n";
            Result = Convert.str(); // set 'Result' to the contents of the stream
            resultFile << Result;
            Save results to file */
            
            
            result_.bingoCardState = cardState;
            std::vector<int> empty_vector;
            result_.missingNumbers = empty_vector;
            result_.wrongNumbers = empty_vector;
            as_.publishFeedback(feedback_);
            as_.setSucceeded(result_);
            
            
            
            resultFile.close();
        }
        
    }
    /* Execute this code when a goal is provided from action client */
    /* This function will identify a card marker, detect the bingo card, and determine the state of the card */
    void goalCB()
    {
        feedback_.cardDetected = 0;
        //2 goals--- calledNumbers and the type of game being played (eg lines, corners etc)
        goal_= *as_.acceptNewGoal();
        calledNumbers = goal_.calledBingoNumbers;
        gameType = goal_.newGameType;

    }
    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "BingoDetection");
    BingoDetectionAction BingoDetection("BingoDetection");
    ros::spin();
    return 0;
}
