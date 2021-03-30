#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <bingo_detection/BingoDetectionAction.h>
#include <vector>
#include <fstream>

#include "text_to_speech/tts.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////
/*
There are two goals sent to the server

1) vector<int> called Numbers ------  vector of the previously called numbers for tangy to check

2) string gameType ------ string of the game type being played
                          ("lines", "corners", "box", "cross" and "full")
                          default is "lines"


*/
////////////////////////////////////////////////////////////////////////////////////////////////

class BingoDetectionClient{
public:
    BingoDetectionClient(std::string name,std::string file):
		//Set up the client. It's publishing to topic "test_action", and is set to auto-spin
		ac("BingoDetection", true),
		//Stores the name
		action_name(name)
	{
		//Get connection to a server
		ROS_INFO("%s Waiting For Server...", action_name.c_str());
		//Wait for the connection to be valid
        calledNumberDatabase(file);
		ac.waitForServer();
		ROS_INFO("%s Got a Server...", action_name.c_str());
	}
	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,
			const bingo_detection::BingoDetectionResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ROS_INFO("Result: %d", result->bingoCardState);
        ROS_INFO("SIZE OF MISSING NUMBERS: %d", result->missingNumbers.size());
        ROS_INFO("SIZE OF WRONG NUMBERS: %d", result->wrongNumbers.size());

        ros::shutdown();
	}

	// Called once when the goal becomes active
	void activeCb()
	{
		ROS_INFO("Goal just went active...");
    }
	
	// Called every time feedback is received for the goal
	void feedbackCb(const bingo_detection::BingoDetectionFeedbackConstPtr& feedback)
	{
		//ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->currentSampleNumber);
	}

    //Send a goal to the server //Frank:::: Need to have additonal input for the game type
    void send(std::vector<int> goal, std::string gameType)
	{
        //games are {lines, corners,box, cross, full};
		bingo_detection::BingoDetectionGoal newGoal;
		newGoal.calledBingoNumbers = goal;
        if(gameType == "lines") newGoal.newGameType = 0;
        else if(gameType == "box") newGoal.newGameType = 1;
        else if(gameType == "cross") newGoal.newGameType = 2;
        else if(gameType == "corners") newGoal.newGameType = 3;
        else if(gameType == "full") newGoal.newGameType = 4;
        else newGoal.newGameType = 0;



		
		//Once again, have to used boost::bind because you are inside a class
		ac.sendGoal(newGoal, boost::bind(&BingoDetectionClient::doneCb, this, _1, _2),
					boost::bind(&BingoDetectionClient::activeCb, this),
					boost::bind(&BingoDetectionClient::feedbackCb, this, _1));
	}
    std::vector<int> calledNumbers;
private:
	actionlib::SimpleActionClient<bingo_detection::BingoDetectionAction> ac;
	std::string action_name;
    int calledNumberDatabase(std::string);
};

int BingoDetectionClient::calledNumberDatabase(std::string calledDatabaseFile)
{
    std::ifstream inFile(calledDatabaseFile.c_str());
    std::string line;
    std::getline(inFile, line);
    std::istringstream iss(line);
    int temp;

    printf("\n DATABASE FILE: %s",calledDatabaseFile.c_str());
    while(iss >> temp){
                calledNumbers.push_back(temp);

    }
    /*
    printf("Number of called numbers: %i",calledNumbers.size());
    for(int i = 0; i<calledNumbers.size(); i++)
        printf("\n%i ",calledNumbers[i]);
    */

    inFile.close();
}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "test_bingodetection_client");

    ros::NodeHandle nh_;
    std::string calledNumberFile;
    nh_.getParam("/bingo_detection_client/calledNumbersFile", calledNumberFile);

    //Usage check to make sure the client is being used properly

	std::string i;

	//Initialize the client
    BingoDetectionClient client(ros::this_node::getName(),calledNumberFile);

    std::cout << "Please Pick a game 'lines' 'box' corners' 'cross' 'full' \n";
    std::cin >> i;
    client.send(client.calledNumbers,i);

	ROS_INFO("Sent Goal To Server...");

    sleep(5);


	ros::spin();


	return 0;
}
