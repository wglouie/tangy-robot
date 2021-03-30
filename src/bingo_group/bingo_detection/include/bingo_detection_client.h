/************************************************************************************************
// Frank Despond
//
// Date: January 23, 2015
//
// Version: 1.0
***************************************************************************************************/
//This should set the value to 0-4 which is either lines, corners, box, cross and full
//from 0-4 lines, corners,box, cross, full

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <bingo_detection/BingoDetectionAction.h>
#include <vector>
#include <string>


using namespace std;

class BingoDetectionClient{

public:

    //Client's attributes
    actionlib::SimpleActionClient<bingo_detection::BingoDetectionAction> ac;
    //states
    bool done;
    bool firstPlay;
    bool running;
    bool ready;
    
    
    int num_times_succeeded;
    int num_times_failed;

    /*Important variables*/
    vector<int> wrongNumbers;
    vector<int> missingNumbers;
    enum state{BINGO, GOODCARD, BADCARD, ERROR, UNKNOWN};
    
    state gameState;


    BingoDetectionClient(std::string name):
        //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
        ac("BingoDetection", true),
        //Stores the name
        action_name_(name)
    {

        //Get connection to a server
        ROS_INFO("[%s] Waiting For Server...", action_name_.c_str());
        //Wait for the connection to be valid
        ac.waitForServer();
        ROS_INFO("[%s] Got a Server...", action_name_.c_str());
        
        gameState=UNKNOWN;

        done = false;
        running = false;
        ready = true;
        firstPlay = true;
        num_times_succeeded=0;
        num_times_failed=0;

    }


    BingoDetectionClient(std::string name, float wait_time):
        //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
        ac("BingoDetection", true),
        //Stores the name
        action_name_(name)
    {

        //Get connection to a server
        ROS_INFO("[%s] Waiting For Server...", action_name_.c_str());
        //Wait for the connection to be valid

        if( ac.waitForServer(ros::Duration(wait_time)) ){
            ROS_INFO("%s Got a Server...", action_name_.c_str());
        } else {
            ROS_INFO("%s Failed to get a Server...", action_name_.c_str());
        }

        gameState=UNKNOWN;

        done = false;
        running = false;
        ready = true;
        firstPlay = true;
        num_times_succeeded=0;
        num_times_failed=0;

    }

    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const bingo_detection::BingoDetectionResultConstPtr& result)
    {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());

        done = true;
        running = false;
        ready = false;
        wrongNumbers = result->wrongNumbers;
        missingNumbers = result->missingNumbers;
        
        ROS_INFO("The bingoCardState is %d.", result->bingoCardState );

        if(result->bingoCardState<= -1){
            gameState=ERROR;
        }else if(result->bingoCardState!=result->bingoCardState){
            gameState=ERROR; ///to catch a NaN error FRANK
        }else if(result->bingoCardState==0){
            if(wrongNumbers.empty() && missingNumbers.empty()){
              gameState=GOODCARD;
            }else{
              gameState=BADCARD;
            }
        }else if(result->bingoCardState>=1){
            if(wrongNumbers.empty() && missingNumbers.empty()){
              gameState=BINGO;
            }else{
              gameState=BADCARD;
            }
            
        }
    }


    /**
    * Called once when the goal becomes active
    */
    void activeCb()
    {
        ROS_INFO("[%s] Goal just went active...", action_name_.c_str());

        done = false;
        running = true;
        ready = false;
    }


    /**
    * Called every time feedback is received for the goal
    * @param feedback
    */
    void feedbackCb(const bingo_detection::BingoDetectionFeedbackConstPtr& feedback)
    {
        ROS_INFO("[%s} Receiving feedback...", action_name_.c_str());
    }


    /**
    * Send a goal to the server
    * @param newGoal
    */
    void sendGoal(vector<int> previouslyCalledNumbers)
    {
        if(!firstPlay)
          reset();
        else
          firstPlay = false;
        bingo_detection::BingoDetectionGoal newGoal;
        if(previouslyCalledNumbers.empty()){
          ROS_INFO("There have not been any previously called numbers.");
        }else{
          ROS_INFO("There have been %d numbers called.", (int) previouslyCalledNumbers.size());
        }
        newGoal.calledBingoNumbers = previouslyCalledNumbers;
      	newGoal.newGameType = 0;
  
        //Once again, have to used boost::bind because you are inside a class
        ac.sendGoal(newGoal, boost::bind(&BingoDetectionClient::doneCb, this, _1, _2),
                                boost::bind(&BingoDetectionClient::activeCb, this),
                                boost::bind(&BingoDetectionClient::feedbackCb, this, _1));
                                
    }
     
    void sendGoalAndWait(vector<int> previouslyCalledNumbers)
    {
        if(!firstPlay)
          reset();
        else
          firstPlay = false;
        bingo_detection::BingoDetectionGoal newGoal;
        if(previouslyCalledNumbers.empty()){
          ROS_INFO("There have not been any previously called numbers.");
        }else{
          ROS_INFO("There have been %d numbers called.", (int) previouslyCalledNumbers.size());
        }
        newGoal.calledBingoNumbers = previouslyCalledNumbers;
        newGoal.newGameType = 0;
        //Once again, have to used boost::bind because you are inside a class
        ac.sendGoal(newGoal, boost::bind(&BingoDetectionClient::doneCb, this, _1, _2),
                                boost::bind(&BingoDetectionClient::activeCb, this),
                                boost::bind(&BingoDetectionClient::feedbackCb, this, _1));
                                
        ac.waitForResult();
    }

	 void sendGoal(vector<int> previouslyCalledNumbers, int gameType)
    {
        if(!firstPlay)
          reset();
        else
          firstPlay = false;
        bingo_detection::BingoDetectionGoal newGoal;
        if(previouslyCalledNumbers.empty()){
          ROS_INFO("There have not been any previously called numbers.");
        }else{
          ROS_INFO("There have been %d numbers called.", (int)  previouslyCalledNumbers.size());
        }
        newGoal.calledBingoNumbers = previouslyCalledNumbers;
        newGoal.newGameType = gameType;

        //Once again, have to used boost::bind because you are inside a class
        ac.sendGoal(newGoal, boost::bind(&BingoDetectionClient::doneCb, this, _1, _2),
                                boost::bind(&BingoDetectionClient::activeCb, this),
                                boost::bind(&BingoDetectionClient::feedbackCb, this, _1));
                                
    }




void sendGoalAndWait(vector<int> previouslyCalledNumbers, int gameType)
    {
        if(!firstPlay)
          reset();
        else
          firstPlay = false;
        bingo_detection::BingoDetectionGoal newGoal;
        if(previouslyCalledNumbers.empty()){
          ROS_INFO("There have not been any previously called numbers.");
        }else{
          ROS_INFO("There have been %d numbers called.", (int) previouslyCalledNumbers.size());
        }
        newGoal.calledBingoNumbers = previouslyCalledNumbers;
	      newGoal.newGameType = gameType;
  
        //Once again, have to used boost::bind because you are inside a class
        ac.sendGoal(newGoal, boost::bind(&BingoDetectionClient::doneCb, this, _1, _2),
                                boost::bind(&BingoDetectionClient::activeCb, this),
                                boost::bind(&BingoDetectionClient::feedbackCb, this, _1));
                                
        ac.waitForResult();

    }    /**
    * Called for reset the action client (it cannot be running)
    */
    void reset()
    {
        done = false;
        running = false;
        ready = true;
        wrongNumbers.clear();
        missingNumbers.clear();
        

    }
    
    vector<string> get_MissingNumbers(){
        vector<string> missingNumbers_strings;
        for(int i=0; i<missingNumbers.size(); i++){
            stringstream ss;
            if(missingNumbers.at(i)<=15){
              ss<<"B-"<<missingNumbers.at(i);
            }else if(missingNumbers.at(i)<=30){
              ss<<"I-"<<missingNumbers.at(i);
            }else if(missingNumbers.at(i)<=45){
              ss<<"N-"<<missingNumbers.at(i);
            }else if(missingNumbers.at(i)<=60){
              ss<<"G-"<<missingNumbers.at(i);
            }else if(missingNumbers.at(i)<=75){
              ss<<"O-"<<missingNumbers.at(i);
            }
            
            
            missingNumbers_strings.push_back(ss.str());
        }
      
      
        return missingNumbers_strings;
    }
    
    vector<string> get_WrongNumbers(){
        vector<string> wrongNumbers_strings;
        for(int i=0; i<wrongNumbers.size(); i++){

            stringstream ss;
            if(wrongNumbers.at(i)<=15){
              ss<<"B-"<<wrongNumbers.at(i);
            }else if(wrongNumbers.at(i)<=30){
              ss<<"I-"<<wrongNumbers.at(i);
            }else if(wrongNumbers.at(i)<=45){
              ss<<"N-"<<wrongNumbers.at(i);
            }else if(wrongNumbers.at(i)<=60){
              ss<<"G-"<<wrongNumbers.at(i);
            }else if(wrongNumbers.at(i)<=75){
              ss<<"O-"<<wrongNumbers.at(i);
            }
            
            
            wrongNumbers_strings.push_back(ss.str());
        }
      
        return wrongNumbers_strings;
    }
    
    /*Returns the result of the detected card in a string
      good card--card is correctly played
      bad card -- missing or wrong numbers
      bingo -- winning condition
      error -- could not read card, or something else went wrong */
    string get_GameState(){
      if(gameState==10){
        ROS_ERROR("Bingo search client did not execute!");
      }
      if(gameState==GOODCARD){
        return "good card";
      }else if(gameState==BADCARD){
        return "bad card";
      }else if(gameState==BINGO){
        return "bingo";
      }else if(gameState==ERROR){
        return "error";
      }else{
        return "unknown";
      }

    }

    void waitForResult(){
        ROS_INFO("[%s] Waiting for result...", action_name_.c_str());
        ac.waitForResult();
    }

private:
    string action_name_;

};
