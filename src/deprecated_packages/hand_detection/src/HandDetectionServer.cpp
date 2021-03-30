#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "std_msgs/Int32MultiArray.h"

//Include Action libraries
#include <actionlib/server/simple_action_server.h>
#include <hand_detection/HandDetectionAction.h>

#include <string.h>
#include <math.h>
#include <vector>

std::string users_ID[15] = {"1","2","3","4","5","6","7","8","9","10","11","12","13","14","15"};		//User tf frame being tracked
//TF child frames being tracked
std::string left_hand = "/left_hand_";									
std::string right_hand = "/right_hand_";
std::string left_elbow = "/left_elbow_";
std::string right_elbow = "/right_elbow_";
std::string head = "/head_";
//TF parent frame being tracked
std::string skeleton_string = "";
std::string kinect_frame = skeleton_string + "/openni_depth_frame";

//simple position structure
struct Position{
	float x;
	float y;
	float z;
};

//Skeleton structure for hand raising (You can add more if necessary)
struct Skeleton{
	Position left_hand;
	Position right_hand;
	Position left_elbow;
	Position right_elbow;
	Position head;
};

class HandDetectionAction{
protected:
	ros::NodeHandle nh_;

	/* Kinect skeleton subscriptions */	
	ros::Subscriber sub_; 
	

	/*Action library variables*/
	actionlib::SimpleActionServer<hand_detection::HandDetectionAction> as_;
	std::string action_name_;
	hand_detection::HandDetectionFeedback feedback_;
	hand_detection::HandDetectionResult result_;
	

	/*Hand detection variables*/
	int captureMode;
	int gesture_frames[15];
	std::vector<float> x;
	std::vector<float> y;
	std::vector<float> z;
	
	int numberOfUsers;
public:
	HandDetectionAction(std::string name) 
		:as_(nh_, name, false), 
		 action_name_(name)
	{
		captureMode = 0;
		/* Initialize action library */
		as_.registerGoalCallback(boost::bind(&HandDetectionAction::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&HandDetectionAction::preemptCB, this));
		as_.start();
		/* Initialize hand detection subscription */
		ROS_INFO("Subscribing to users topic");
		sub_ = nh_.subscribe("users",1,&HandDetectionAction::usersCallback,this);
		
	}
	void goalCB()
	{		

		//Accept the new goal
		ROS_INFO("\n\n CaptureMode: %i",captureMode);
		captureMode = as_.acceptNewGoal()->captureMode;
		ROS_INFO("\n\n CaptureMode: %i",captureMode);
		for(int i=0; i<15; i++)
		{
			gesture_frames[i] = 0;
		}
	}
	void preemptCB()
	{
		ROS_INFO("%s: Preempted", action_name_.c_str());
		// set the action state to preempted
		as_.setPreempted();
		captureMode=0;
	}

	//Callback for user message being received from OPENNI that provides users being tracked
	void usersCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
	{
		ROS_INFO("Size is %d" , array->data.size()); 
		ROS_INFO("Got inside the Users Callback!");
		if(captureMode==0 || !as_.isActive())
		{
			ROS_INFO("CAPTURE MODE OFF");
			return;
		}
		/*Clear out position holders every new image*/
		numberOfUsers = 0;	
		x.clear();
		y.clear();
		z.clear();


		tf::TransformListener listener;
		std::string target_frame;
		//print all the remaining numbers
		tf::StampedTransform transform;
		Skeleton userSkeleton;	

		/*
		//GRAB ROBOT X,Y,Z POSITION IN THE MAP BITCH
                std::string map_frame = "/map";
                std::string base_frame = "/base_link";
                tf::StampedTransform transform;

                listener.waitForTransform(map_frame,base_frame, ros::Time(0),ros::Duration(10) );
                                //Grab transform
                listener.lookupTransform(map_frame,base_frame,ros::Time(0),transform);
                int robotPositionZ = transform.getOrigin().z();
                int robotPositionY = transform.getOrigin().y();
                int robotPositionX = transform.getOrigin().x();
		*/		

		/*
		std::vector<std::string> frameList;
		listener.getFrameSbingodetectionAC.bingoCardStatetrings(frameList);
		
		printf("Number of frames: %i\n",frameList.size());
		for(int i =0; i<frameList.size(); i++)
		{
			printf("Frame: %s \n",frameList[i].c_str());
		}
		*/


		//Loop to test all users being tracked where it* is the userID number
		


		for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
		{
				//ROS_INFO("\nUser %i",*it);
			
				/*OBTAIN ALL REAL WORLD COORDINATES OF JOINT INFORMATION*/		
				//Left hand
				try{
					//Wait for transform to be sent
					listener.waitForTransform(kinect_frame,skeleton_string+left_hand+users_ID[*it-1], ros::Time(0), 									ros::Duration(10) );
					//Grab transform
					listener.lookupTransform(kinect_frame,skeleton_string+left_hand+users_ID[*it-1], 										ros::Time(0),transform);
					//Obtain necessary data points
					userSkeleton.left_hand.z = transform.getOrigin().z();
					userSkeleton.left_hand.y = transform.getOrigin().y();
					userSkeleton.left_hand.x = transform.getOrigin().x();
				}
				catch(tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					continue;
				}
				//Left elbow
				try{
					listener.waitForTransform(kinect_frame,skeleton_string+left_elbow+users_ID[*it-1], ros::Time(0),  										ros::Duration(10) );
					listener.lookupTransform(kinect_frame,skeleton_string+left_elbow+users_ID[*it-1],
									ros::Time(0),transform);
					userSkeleton.left_elbow.z = transform.getOrigin().z();
				}
				catch(tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					continue;
				}
				//Right hand
				try{
					listener.waitForTransform(kinect_frame,skeleton_string+right_hand+users_ID[*it-1], ros::Time(0),
									 ros::Duration(10) );
					listener.lookupTransform(kinect_frame,skeleton_string+right_hand+users_ID[*it-1],
									ros::Time(0),transform);
					userSkeleton.right_hand.z = transform.getOrigin().z();
					userSkeleton.right_hand.y = transform.getOrigin().y();
					userSkeleton.right_hand.x = transform.getOrigin().x();
				}
				catch(tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					continue;
				}
				//Right Elbow
				try{
					listener.waitForTransform(kinect_frame,skeleton_string+right_elbow+users_ID[*it-1], ros::Time(0),
									 ros::Duration(10) );
					listener.lookupTransform(kinect_frame,skeleton_string+right_elbow+users_ID[*it-1],
									ros::Time(0),transform);
					userSkeleton.right_elbow.z = transform.getOrigin().z();
				}
				catch(tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					continue;
				//Head	
				}try{
					listener.waitForTransform(kinect_frame,skeleton_string+head+users_ID[*it-1], ros::Time(0),
									 ros::Duration(10) );
					listener.lookupTransform(kinect_frame,skeleton_string+head+users_ID[*it-1],ros::Time(0),transform);
					userSkeleton.head.z = transform.getOrigin().z();
					userSkeleton.head.x = transform.getOrigin().x();
				}
				catch(tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					continue;
				}
			
				/* MAKE DECISION ON WHETHER A USERS HAND IS UP */
				if( ( (userSkeleton.left_hand.z > userSkeleton.head.z) || 
					(userSkeleton.left_hand.z > userSkeleton.left_elbow.z + 0.05) ) &&
			     	        (fabs(userSkeleton.left_hand.x - userSkeleton.head.x) > 0.02) )
				{
					gesture_frames[*it] += 1;
					if(gesture_frames[*it] > 20)
					{
						x.push_back(userSkeleton.left_hand.x);
						y.push_back(userSkeleton.left_hand.y);
						z.push_back(userSkeleton.left_hand.z);
						numberOfUsers++;
						ROS_INFO("User %i: HAND IS RAISED",*it);						
					}	
				}
				else if(((userSkeleton.right_hand.z > userSkeleton.head.z) || 
					 (userSkeleton.right_hand.z > userSkeleton.right_elbow.z + 0.05)) && 
					 (fabs(userSkeleton.right_hand.x - userSkeleton.head.x) > 0.02) )
				{
					gesture_frames[*it] += 1;
					if(gesture_frames[*it] > 20)
					{
						x.push_back(userSkeleton.right_hand.x);
						y.push_back(userSkeleton.right_hand.y);
						z.push_back(userSkeleton.right_hand.z);
						numberOfUsers++;
						ROS_INFO("User %i: HAND IS RAISED",*it);
					}
				}
				else
				{			
				   if(gesture_frames[*it] != 0)	
				     gesture_frames[*it] -= 1;								
				}				
				ROS_INFO("User %i number of frames: %i",*it,gesture_frames[*it]);
		}
		feedback_.x = x;
		feedback_.y = y;
	 	feedback_.z = z;
	 	feedback_.numberOfHands = numberOfUsers;
		as_.publishFeedback(feedback_);
		return;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "HandDetection");
	HandDetectionAction handDetection("HandDetection");
	ROS_INFO("Hand Detection Node has started");
	ros::spin();
	return 0;
}

