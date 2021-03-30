/*
 * Frank Despond
 *
 * Version 1.0---->Refactored from old BingoDetectionAction Code
 *
 * January 30, 2015
 */
#include "bingodetectionserver.h"

#define NODEBUG
BingoDetectionServer::BingoDetectionServer(string name):
    as_(nh_, name, false),	//Initialize action server
    action_name_(name),
    it_(nh_),detectionCounter(0), gameType(0)

{
    as_.registerGoalCallback(boost::bind(&BingoDetectionServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&BingoDetectionServer::preemptCB, this));
    as_.start();

    /* Camera grab stuff */
		last_image = NULL;
    image_pub_ = it_.advertise("out", 1);							//Advertise output images
    image_sub_ = it_.subscribe("in", 1, &BingoDetectionServer::imageCb, this);		//Subscribe to camera images

    nh_.getParam("/bingo_detection_server/cardDatabaseFolder", cardDatabaseFolder);			//Grab album directory from parameter server
    nh_.getParam("/bingo_detection_server/numberDatabaseFile", numberDatabaseFile);

    faultpub = nh_.advertise<std_msgs::Empty>("environment_faults", 1);

    #ifdef DEBUG
    resultFile.open ("//bingoTestResults//BingoDetectionResults.txt",std::ofstream::app);
    #endif

    //bingo = NULL;

}

BingoDetectionServer::~BingoDetectionServer()
{
    printf("\nFIN");
}

//void BingoDetectionServer::done()
//{
//    delete bingo;

//}

void BingoDetectionServer::goalCB()
{
    feedback_.cardDetected = 0;
    //2 goals--- calledNumbers and the type of game being played (eg lines, corners etc)
    goal_= *as_.acceptNewGoal();
    calledNumbers = goal_.calledBingoNumbers;
    gameType = goal_.newGameType;

}

void BingoDetectionServer::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void BingoDetectionServer::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    if(last_image != NULL){
			image_pub_.publish(last_image);
		}
    if(!as_.isActive())
        return;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
				last_image = msg;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Mat img=cv_ptr->image;
    //Mat img = imread("..//paperPictures//duck.jpg",1); //For testing without cameras

    std::ostringstream convert;

    time_t rawtime;
    struct tm * timeinfo;

    time (&rawtime);
    timeinfo = localtime (&rawtime);
   // printf ("Current local time and date: ", asctime(timeinfo));


    vector<int> wrongNumbers, missingNumbers;
    #ifdef DEBUG
    std::string imageResult;       		   // string which will contain the result
    std::ostringstream imageConvert;  			   // stream used for the conversion
    imageConvert << asctime(timeinfo);     // insert the textual representation of 'Number' in the characters in the stream
    imageResult = imageConvert.str(); // set 'Result' to the contents of the stream
    cv::imwrite(("..//Results//" + imageResult + ".jpg").c_str(),img);
    #endif

    BingoCard bingo(img, cardDatabaseFolder, calledNumbers, gameType, numberDatabaseFile);


    if(bingo.cardHasBingo()!=-1)
    {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        result_.bingoCardState = bingo.cardHasBingo();
        result_.missingNumbers = bingo.getMissingNumbers();
        result_.wrongNumbers = bingo.getWrongNumbers();
        wrongNumbers = bingo.getWrongNumbers();
        missingNumbers = bingo.getMissingNumbers();
        #ifdef DEBUG
        resultFile.open ("..//Results//BingoDetectionResults.txt",std::ofstream::app);

        convert << "Card Date Tag: " << imageResult << "\n";
        //Called numbers
        convert << "Called Numbers: ";
        for(int i = 0; i<calledNumbers.size(); i++)
            convert << calledNumbers[i] << ", ";
        convert << "\n";

        //Numbers Marked Incorrectly
        convert << "Numbers marked incorrectly: ";
        for(int i = 0; i<wrongNumbers.size(); i++)
            convert << wrongNumbers[i] << ", ";
        convert << "\n";

        //Missing Number
        convert << "Numbers missing: ";
        for(int i = 0; i<missingNumbers.size(); i++)
            convert << missingNumbers[i] << ", ";
        convert << "\n\n";

        resultFile << convert.str(); // set 'resultFile' to the contents of the stream


        #endif

        feedback_.cardDetected = 1;
        as_.publishFeedback(feedback_);
        as_.setSucceeded(result_);
        resultFile.close();



    }
    else //does not see card
    {

        ROS_INFO("%s: FAILED", action_name_.c_str());

        faultpub.publish(std_msgs::Empty()); //publish a dummy message to trip the fault detection

        result_.bingoCardState = -1;

        std::vector<int> empty_vector;
        result_.missingNumbers = empty_vector;
        result_.wrongNumbers = empty_vector;
        as_.setSucceeded(result_);

    }

    //done();
}

