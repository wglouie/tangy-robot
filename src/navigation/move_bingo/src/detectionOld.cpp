#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <bingo_detection/BingoDetectionAction.h>
#include <vector>
#include <fstream>



#include <drrobot_h20_player/HeadCmd.h>
#include <geometry_msgs/Twist.h>



class BingoDetectionClient{
public:
	actionlib::SimpleActionClient<bingo_detection::BingoDetectionAction> ac;
	

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
		CState = result->bingoCardState;
		//ros::shutdown();
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
		ROS_INFO("State: %d", feedback->cardDetected);

		DState = feedback->cardDetected;
		
		
	}

	//Send a goal to the server
	void send(std::vector<int> goal)
	{
		bingo_detection::BingoDetectionGoal newGoal;
		newGoal.calledBingoNumbers = goal;
		
		//Once again, have to used boost::bind because you are inside a class
		ac.sendGoal(newGoal, boost::bind(&BingoDetectionClient::doneCb, this, _1, _2),
					boost::bind(&BingoDetectionClient::activeCb, this),
					boost::bind(&BingoDetectionClient::feedbackCb, this, _1));
	}
    std::vector<int> calledNumbers;
	int CState;
	int DState;
private:
	
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

   	//To Do
	
		//get card detection to fully run through by itself - scraped
 
		//get the called numbers thing to work - scraped
		//figure out values that are given out after completion - 
		//detects properly: CState = 0 or 1, DState = 1
		//doesn't detect : CState = -1, DState = 0
		//figure out how to autorun card detection server multiple times and stop when it detects
		//problem is that DState isn't changing after it detects a valid check
		//put into other code

	

    	ros::init(argc, argv, "bingodetection_client");

    	int card_state = 0;
    	int detection_state = 0;

	int overall_state = 1;


	ros::NodeHandle nh_;
	std::string calledNumberFile;
	nh_.getParam("/Bingo/bingo_detection_client/calledNumbersFile", calledNumberFile);

	//Usage check to make sure the client is being used properly
	


	ros::NodeHandle n_;
	ros::Publisher pub_head;

	pub_head = n_.advertise<drrobot_h20_player::HeadCmd>("/cmd_head", 1);
	//topic is cmd_head
	//pub_head is publisher

	drrobot_h20_player::HeadCmd cmdhead_;
	//cmdhead_ is a message 

	

	int neck_x_rotation_ = -4000;
	int neck_z_rotation_= 4000;
	int mouth_= 0;
	int upper_head_ = 0;
	int left_eye_ = 0;
	int right_eye_ = 0;
	int flag_ = 0;
	
	printf("\nstart\n");

	int temp_x = 0;
	int temp_z = 0;

	int i = 0;
	bool check = false;

	ros::Rate loop_rate(1);

	//Send commands to HEAD servos
	cmdhead_.neck_x_rotation = -4000; //neck_x_rotation_;
	cmdhead_.neck_z_rotation = 4000; //neck_z_rotation_;
        cmdhead_.mouth = mouth_;
        cmdhead_.upper_head = upper_head_;
        cmdhead_.left_eye = left_eye_;
        cmdhead_.right_eye = right_eye_;
	cmdhead_.flag = flag_;

	while (overall_state != 3){

		printf("\npublish %d\n", i);

		pub_head.publish(cmdhead_);

		printf("\nsleep");
			
		//loop_rate.sleep();

		sleep(1);
		ros::spinOnce();



		if (overall_state == 1)
		{
			
	
			i++;

			if (i == 40)
			{
				//check = true;

				i = 3;
				if (check)
					overall_state = 3;

				cmdhead_.neck_x_rotation *=-1;
				cmdhead_.neck_z_rotation *=-1;
				check = true;

			
			}

			if (i == 2)
			{
				cmdhead_.neck_x_rotation = 0; 
				cmdhead_.neck_z_rotation = -100;
			}



			if ((i % 8) == 7) // don't make this 0 or 3
			{
				temp_x = cmdhead_.neck_x_rotation;
				temp_z = cmdhead_.neck_z_rotation;
				cmdhead_.neck_x_rotation = 0; //neck_x_rotation_;
				cmdhead_.neck_z_rotation = 0; //neck_z_rotation_;
				//neck_x_rotation_ = 0;
				//neck_z_rotation_= 0;
				
				overall_state = 2;


				//sleep(10);
				//cmdhead_.neck_x_rotation = temp_x;
				//cmdhead_.neck_z_rotation = temp_z;

			}



		}


		if (overall_state == 2)
		{

			sleep(1);
			//Initialize the client
			BingoDetectionClient client(ros::this_node::getName(),calledNumberFile);

			//sleep(2);
			client.send(client.calledNumbers);

		
			ROS_INFO("Sent Goal To Server...");
			do{
				//ros::spin();
						
				client.ac.waitForResult();
				sleep(1);

				ROS_INFO("In Loop CState: %d", card_state);
				ROS_INFO("IL Detection State: %d", detection_state);
						
				card_state = client.CState;
				detection_state = client.DState;
				
				if (card_state == -1 || card_state == 0 || card_state == 1)
				{
					overall_state = 1; 
					cmdhead_.neck_x_rotation = temp_x;
					cmdhead_.neck_z_rotation = temp_z;	

				}
			

				if (detection_state == 1)
				{
					overall_state = 3;
				}
		
				ROS_INFO("In Loop CState: %d", card_state);
				ROS_INFO("IL Detection State: %d", detection_state);
			
				
			}while (card_state >1 || card_state <-1);
		//}	
			card_state = -2;
		
			ROS_INFO("Card State: %d", card_state);
			ROS_INFO("Detection State: %d", detection_state);
			//i++;

			
	
		}


	}
	
	//ROS_INFO("Tangy has detected the Bingo Card and stopped moving its head!");
	ROS_INFO("Done");

	return 0;
}
