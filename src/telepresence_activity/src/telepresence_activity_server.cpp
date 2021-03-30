#include <telepresence_activity/telepresence_activity_server.hpp>
//#include "telepresence_activity_server.hpp"

#include <geometry_msgs/Twist.h>
#include <drrobot_h20_player/HeadCmd.h>

TelepresenceActivityServer::TelepresenceActivityServer(std::string name):
	aS_(nh_, name, false),	//Initialize action server
//	aS_(nh_ , name , boost::bind(&TelepresenceActivityServer::executeCB , this , _1) , false),
        action_name_(name)
	{
	
	//start the clients
	robotGuiClient = new RobotGuiClient("robot_gui_client");
	//navigationClient = new NavigationClient("navigation_client");
	userTrackingClient = new UserTrackingClient("user_tracking_client");

	/* Action library initialize */
	aS_.registerGoalCallback(boost::bind(&TelepresenceActivityServer::TelepresenceActivityServerGoalCB, this));
	aS_.registerPreemptCallback(boost::bind(&TelepresenceActivityServer::TelepresenceActivityServerPreemptCB, this));
	
	aS_.start();

	pub_head = nh_.advertise<drrobot_h20_player::HeadCmd>("cmd_head", 1);
	}


void TelepresenceActivityServer::resetHead(){
	
	//	double maxVel = 1.0;
//	double maxTurn = 1.0;
	int neck_x_rotation_ = 0;
	int neck_z_rotation_= 0;
	int mouth_= 0;
	int upper_head_ = 0;
	int left_eye_ = 0;
	int right_eye_ = 0;
	int flag_ = 2;
	bool dirty = false;

//	cmdvel_.linear.x = maxVel;
//        cmdvel_.angular.z = maxTurn;
        cmdhead_.neck_x_rotation = neck_x_rotation_;
        cmdhead_.neck_z_rotation = neck_z_rotation_;
        cmdhead_.mouth = mouth_;
        cmdhead_.upper_head = upper_head_;
        cmdhead_.left_eye = left_eye_;
        cmdhead_.right_eye = right_eye_;
        cmdhead_.flag = flag_;
	
	ROS_INFO("neck_x_rotation_: %d", cmdhead_.neck_x_rotation);
	ROS_INFO("neck_z_rotation_: %d", cmdhead_.neck_z_rotation);
	ROS_INFO("mouth_ %d", cmdhead_.mouth);
	ROS_INFO("upper_head_: %d", cmdhead_.upper_head);
	ROS_INFO("left_eye_: %d", cmdhead_.left_eye);
	ROS_INFO("right_eye_: %d", cmdhead_.right_eye);
	ROS_INFO("flag_	: %d", cmdhead_.flag);
        
	pub_head.publish(cmdhead_);
	ROS_INFO("resetHead");
}



/* Execute this code when a goal is provided from action client */
/* This function will start the telepresence GUI */
void TelepresenceActivityServer::TelepresenceActivityServerGoalCB()
{   
	//start the clients
	robotGuiClient = new RobotGuiClient("robot_gui_client");
	//navigationClient = new NavigationClient("navigation_client");
	userTrackingClient = new UserTrackingClient("user_tracking_client");

	goal_= aS_.acceptNewGoal();

	user_ = goal_->user;
	skypeUser_ = goal_->skypeUser;
	nameOfCaller_ = goal_->nameOfCaller;

	ROS_INFO("[%s] Received user: %s", action_name_.c_str(), user_.c_str());
	ROS_INFO("[%s] Received skypeUser: %s", action_name_.c_str(), skypeUser_.c_str());
	ROS_INFO("[%s] Received nameOfCaller: %s", action_name_.c_str(), nameOfCaller_.c_str());

	resetHead();
	sleep(2);

	
	bool 	end = false, 
	//	navigating = false,
		trackingUser = true,
		telepresence = false,
		sessionOn = false,
		acceptedSession = false,
	//	navigateBack = false,
	//	sessionCompleted = false,
	//	gotFinalPosition = false, 
		startTrackingUser = true;


	startTrackingUser = true;
	//------------tracking the user

	//sleep(10); 
	printf("\n\n\n***************************************\
	         \n\n inRange = %d\n\n\n\n********************************", userTrackingClient->inRange);
    	sleep(3);

	while(!end){
		if(trackingUser){

			if(startTrackingUser){
				std::string s;
				std::stringstream ss;

				ss << "echo " << user_ << "\" Where are you?\" | festival --tts";
				s = ss.str();
				system(s.c_str());

                		ROS_INFO("[%s] Tracking user [%s] / checking user distance", action_name_.c_str(),user_.c_str());
				userTrackingClient->sendGoal(user_);
				startTrackingUser = false;
			}
			ROS_INFO("[%s] inRange = %d\n\n", action_name_.c_str(), userTrackingClient->inRange);
			if(userTrackingClient->inRange == 1){
				ROS_INFO("%s is in range\n", user_.c_str());
			//while(userTrackingClient->inRange != 1);
				ROS_INFO("[%s]  User found -> offer the skype session", action_name_.c_str());
				trackingUser = false;
			 	telepresence = true;
			}else{
				//ROS_INFO("%s is not in rang\ninRange = %d\n", user_.c_str(), userTrackingClient->inRange);
		        }
		}
		else if(telepresence){
			if(!sessionOn){
				ROS_INFO("[%s] Offer session", action_name_.c_str());
				
				interface_goal.activity = "telepresence";
				interface_goal.code = -1;
				interface_goal.speech = nameOfCaller_.c_str();
				interface_goal.text = skypeUser_.c_str();

				ROS_INFO("[%s] Sending Goal To Robot GUI Server...", action_name_.c_str());
				robotGuiClient->sendGoal(interface_goal);

				sessionOn = true;
				continue;

			}
			else if (sessionOn){
				ROS_INFO("[%s] Session on -> monitoring session/ tracking user", action_name_.c_str());
				//if(robotGuiClient->sessionStatus == 3 || robotGuiClient->sessionStatus == 4){
				while(!(robotGuiClient->sessionStatus == 3 || robotGuiClient->sessionStatus == 4));
					telepresence = false;
					sessionOn = false;
					end = true;
					userTrackingClient->stopTracking();
				//}
			}
		}
	}
	result_.end = 1;
	aS_.setSucceeded(result_);
}

void TelepresenceActivityServer::TelepresenceActivityServerPreemptCB()
{
	ROS_INFO("[%s]: Preempted", action_name_.c_str());
	// set the action state to preempted
	aS_.setPreempted();
	//tab_ = 0;
	//newGoal = false;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "telepresence_activity_server");

  TelepresenceActivityServer server(ros::this_node::getName());
  ROS_INFO("[%s] server is running", ros::this_node::getName().c_str());

//	sleep(5);

//  server.resetHead();
//server.resetHead();
  //while(ros::ok){
	sleep(3);
//	server.resetHead();
//}
  ros::spin();

  return 0;
}

