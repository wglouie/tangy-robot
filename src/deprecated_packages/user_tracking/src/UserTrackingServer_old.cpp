#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <face_detection/identifyAction.h>
#include <drrobot_h20_player/HeadCmd.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/server/simple_action_server.h>
#include <user_tracking/UserTrackingAction.h>

#define RESOLUTION_X 640
#define RESOLUTION_Y 480

bool received_goal;		//variable for goalCB

struct UserInfo{
	int id;
	float x;
	float y;
};

class UserTrackingAction{
public:

	ros::NodeHandle node_;  
	ros::Subscriber head_pos_sub_;

	ros::NodeHandle nh_;
	ros::Subscriber head_coord_sub_;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	
	//Action library variables
	actionlib::SimpleActionServer<user_tracking::UserTrackingAction> as_;
	user_tracking::UserTrackingFeedback feedback_;
	user_tracking::UserTrackingResult result_;
	
	UserTrackingAction(std::string name):
		
		as_(nh_, name, false),
		//Set up the client. It's publishing to topic "test_action", and is set to auto-spin
		FaceDetectionClient("face_detection/face_detection", true),
		MoveBaseClient("move_base", true),
		//Stores the name
		action_name(name)
	{
		//Register goal and feedback callbacks
		as_.registerGoalCallback(boost::bind(&UserTrackingAction::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&UserTrackingAction::preemptCB, this));
		as_.start();
		
		latest_time = ros::Time::now();
		//Get connection to FaceDetection server
//		ROS_INFO("%s Waiting For FaceDetection Server...", action_name.c_str());
		//Wait for FaceDetection connection to be valid
//		FaceDetectionClient.waitForServer();
//		ROS_INFO("%s Got FaceDetection Server...", action_name.c_str());

//		ROS_INFO("%s Waiting For Navigation Server...", action_name.c_str());
		//Wait for Navigation connection to be valid
//		MoveBaseClient.waitForServer();
//		ROS_INFO("%s Got Navigation Server...", action_name.c_str());

		pub_ = n_.advertise<drrobot_h20_player::HeadCmd>("cmd_head", 1);
		pub_base = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		
	}
	
	~UserTrackingAction(void)
	{
	}
	
	void goalCB()
	{
		//Accept the new goal
		goal_ = as_.acceptNewGoal()->userName;
		
		ROS_INFO("userName: %s", goal_.c_str());
		//Get connection to FaceDetection server
		ROS_INFO("%s Waiting For FaceDetection Server...", action_name.c_str());
		//Wait for FaceDetection connection to be valid
		FaceDetectionClient.waitForServer();
		ROS_INFO("%s Got FaceDetection Server...", action_name.c_str());

		ROS_INFO("%s Waiting For Navigation Server...", action_name.c_str());
		//Wait for Navigation connection to be valid
		MoveBaseClient.waitForServer();
		ROS_INFO("%s Got Navigation Server...", action_name.c_str());

		received_goal = true;
	}
	
	void preemptCB()
	{
		ROS_INFO("%s: Preempted", action_name.c_str());
		// set the action state to preempted
		FaceDetectionClient.cancelAllGoals();
		MoveBaseClient.cancelAllGoals();
		as_.setPreempted();
	}
	
	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,
			const face_detection::identifyResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ROS_INFO("Result: %d", result->numberOfFaces);
		ros::shutdown();
	}

	// Called once when the goal becomes active
	void activeCb()
	{
		ROS_INFO("Goal just went active...");
	}
	
	void neckPositionCB(const drrobot_h20_player::HeadCmd::ConstPtr& neck_pos)
	{
		neck_pos_x = neck_pos->neck_x_rotation;
		neck_pos_z = neck_pos->neck_z_rotation;	  
	}
	
	//Gets head coordinates for user 1 from sensor
	void markerCoordinatesCB(const visualization_msgs::Marker::ConstPtr& marker_pos)
	{  
		if (marker_pos->points.size() > 0)
		{
			if(marker_pos->header.stamp != latest_time)
			{
				UserInfo usr_info;
				if(!first_user_detected)
				{
					
					usr_info.x = marker_pos->points[4].x;
					usr_info.y = marker_pos->points[4].y;
					usr_info.id = marker_pos->id;
					users_info_new.push_back(usr_info);
					first_user_detected = true;
					ROS_INFO("Inside If3!!!!!!");
					latest_time = marker_pos->header.stamp;
				}
				else
				{	
					users_info_old = users_info_new;
					users_info_new.clear();

					usr_info.x = marker_pos->points[4].x;
					usr_info.y = marker_pos->points[4].y;
					usr_info.id = marker_pos->id;
					users_info_new.push_back(usr_info);
					first_user_detected = true;
					//latest_time = marker_pos->header.stamp;
					ROS_INFO("Inside Else!!!!!!");
				}
			}
			else
			{
				UserInfo usr_info;
				usr_info.x = marker_pos->points[4].x;
				usr_info.y = marker_pos->points[4].y;
				usr_info.id = marker_pos->id;
				users_info_new.push_back(usr_info);
					ROS_INFO("Inside Else2!!!!!!");
				first_user_detected = false;
			}
		}
	}
	
	//converts position of a tracked face from Axis to meters (from pixels)
	float convertAxisToXtion(float axis_x_pos)
	{
		float estimated_conversion;
		//1 meter = 3779.528 pixels
		estimated_conversion = -(axis_x_pos-63) / 3779.528;
		
		return estimated_conversion;
	}
	
	//converts position of tracked user from Xtion to pixels (from meters)
	float convertXtionToAxis(float xtion_x_pos)
	{
		float estimated_conversion;
		
		estimated_conversion = -xtion_x_pos * 3779.528;

		return estimated_conversion;
	}

	// Called every time feedback is received for the goal
	void feedbackCb(const face_detection::identifyFeedbackConstPtr& feedback)
	{
		//Subscribe for neck position values and skeleton transform coordinates
		head_pos_sub_ = node_.subscribe<drrobot_h20_player::HeadCmd>("head_pose", 1, boost::bind(&UserTrackingAction::neckPositionCB, this, _1));
		head_coord_sub_ = nh_.subscribe<visualization_msgs::Marker>("tracked_users_markers" , 1000, boost::bind(&UserTrackingAction::markerCoordinatesCB, this, _1));

		int neck_x_rotation_ = 0;
		int neck_z_rotation_= 0;
		int mouth_= 0;
		int upper_head_ = 0;
		int left_eye_ = 0;
		int right_eye_ = 0;

		double maxVel = 0;
		double maxTurn = 0;
		bool dirty = false;
		
		int userid = -1;
		int user_id_xtion = -1;
		int user_index_xtion = -1;
		
//		marker_user_id = -1;
	
		ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->currentSampleNumber);
		ROS_INFO("Faces: %d", feedback->numberOfFaces);
		
		//Sleep if no detection at beginning
		if (feedback->identified_users_size == 0)
		{
			usleep(1000000);
		}
		else
		{
//			while (feedback->identified_users_size == 0)
//				ROS_INFO("feedback = %d", feedback->identified_users_size);
//			while (marker_user_id == -1);
		  
		  
			neck_x_rotation_ = 0;
			neck_z_rotation_ = 0;
			mouth_ = 0;
			upper_head_ = 0;
			left_eye_ = 0;
			right_eye_ = 0;
		  
//			maxVel = 0;
			dirty = true;
		  
			bool shouldmove = false;
                  
			//finding the person in the array
			for (int i = 0; i < feedback->users_identified.size(); i++)
			{
				if (feedback->users_identified[i] == goal_)
				{
					ROS_INFO("User recognized: %s", feedback->users_identified[i].c_str());
					userid = i;
					break;
				}
			}
	
			//if we have detected the user
			if (userid >= 0)
			{
				//calculate x and y displacements of face using axis camera
				x_pos = feedback->x[userid] - (RESOLUTION_X/2);
				y_pos = feedback->y[userid] - (RESOLUTION_Y/2);

				//Getting information from axis camera feedback
				ROS_INFO("Names: %s", feedback->users_identified[userid].c_str());
				ROS_INFO("X: %d", feedback->x[userid]);
				ROS_INFO("Y: %d", feedback->y[userid]);
				ROS_INFO("DX: %d", x_pos);
				ROS_INFO("DY: %d", y_pos);
				ROS_INFO("Neck position X: %d", neck_pos_x);
				ROS_INFO("Neck position Z: %d", neck_pos_z);

				//Matching axins info with asus info

				estimated_x = convertAxisToXtion(x_pos);
				ROS_INFO("estimated_x: %f", estimated_x);

				last_message_received = ros::Time::now() - latest_time;

				std::stringstream t1;
				t1 << last_message_received;
				ROS_INFO("last_message: %s", t1.str().c_str());

				std::stringstream t2;
				t2 << latest_time;
				ROS_INFO("latest_time: %s", t2.str().c_str());

				
				//if last message recieved from users markers is more than 1 second, clear old and new user info
				if(last_message_received <= ros::Duration(1.0))
				{
					ROS_INFO("users_info_old.size(): %d",users_info_old.size());
					for (int i = 0; i < users_info_old.size(); i++)
					{
						ROS_INFO("users_info_old[%d].y + 0.2: %f", i, users_info_old[i].y + 0.2);
						ROS_INFO("users_info_old[%d].y - 0.2: %f", i, users_info_old[i].y - 0.2);
						if (estimated_x <= (users_info_old[i].y + 0.2) && estimated_x >= (users_info_old[i].y - 0.2))
						{
							user_id_xtion = i;
							user_index_xtion = users_info_old[i].id;
							ROS_INFO("-------Found a body in the range!-------");
							break;
						}
					}
				}
				else
				{
					users_info_old.clear();
					users_info_new.clear();
					first_user_detected = false;
				}

				//do the turning based on the x,y cposition coming from the axia camera
				shouldmove = true;
			}
			//When not detecting with Axis camera
			else
			{
				ROS_INFO("User not detected with Axis");
				
				//Matching previous axis info with asus info

				estimated_x = convertAxisToXtion(prev_x_pos);
				ROS_INFO("estimated_x: %f", estimated_x);
				
				last_message_received = ros::Time::now() - latest_time;

				std::stringstream t1;
				t1 << last_message_received;
				ROS_INFO("t1: %s", t1.str().c_str());

				std::stringstream t2;
				t2 << latest_time;
				ROS_INFO("latest_time: %s", t2.str().c_str());


				//if last message received from users markers is more than 1 second, ckear old and new user info
				if(last_message_received <= ros::Duration(1.0))
				{
					for (int i = 0; i < users_info_old.size(); i++)
					{
						if (estimated_x <= (users_info_old[i].y + 0.2) && estimated_x >= (users_info_old[i].y - 0.2) && 
							prev_marker_head_x <= (users_info_old[i].x + 0.2) && prev_marker_head_x >= (users_info_old[i].x - 0.2))
						{
							user_id_xtion = i;
							user_index_xtion = users_info_old[i].id;
							x_pos = convertXtionToAxis(users_info_old[i].y);
							ROS_INFO("-------Found a body in the range!-------");
							break;
						}
					}
				}
				else
				{
					  users_info_old.clear();
					  users_info_new.clear();
					  first_user_detected = false;
				}
				shouldmove = true;
			}

			if (shouldmove)
			{
				//To track a certain person: change 'names[user_num]' to the person's name
				//if (feedback->users_identified[0] == names[user_num] && active_user[marker_user_id - 1] == 1)

				ROS_INFO("Goal: %s", goal_.c_str());
				ROS_INFO("userid: %d", userid);
				ROS_INFO("user_id_xtion: %d", user_id_xtion);
				ROS_INFO("user_index_xtion: %d", user_index_xtion);
				if(user_id_xtion > -1)
				{
					ROS_INFO("Xtion position x: %f", users_info_old[user_id_xtion].x);
					ROS_INFO("Xtion position y: %f", users_info_old[user_id_xtion].y);
				}
				
				//TILTING HEAD
				//Tilt head using AXIS camera info
/*				if (y_pos < -85)
				{
					ROS_INFO("Axis: Tilt Up");
					neck_x_rotation_ = 40;
				}
				else if (y_pos > 85)
				{
					ROS_INFO("Axis: Tilt Down");
					neck_x_rotation_ = -40;
				}
				else
				{
					ROS_INFO("Axis: Don't Move");
					neck_x_rotation_ = 0;
				}
				
*/				//TURNING
				//Turn using axis camera info
				if (userid > -1)
				{
					if (x_pos < -85)
					{
						ROS_INFO("Axis: Turn body Left");
						maxTurn = 0.2;
					}
					else if (x_pos > 100)
					{
						ROS_INFO("Axis: Turn body Right");
						maxTurn = -0.2;
					}
					else
					{
						ROS_INFO("Axis: No Turn");
						maxTurn = 0;
						if (prev_x_pos == x_pos)
						{
							maxTurn = 0;
						}
					}
				}
				//using the xtion info to turn
				else if (user_id_xtion > -1)
				{
					if (users_info_old[user_id_xtion].x <= 1 && users_info_old[user_id_xtion].y > 0.15)
					{
						ROS_INFO("XTion: Turn body Left");
						maxTurn = 0.4;
					}
					else if (users_info_old[user_id_xtion].x <= 1 && users_info_old[user_id_xtion].y < -0.15)
					{
						ROS_INFO("XTion: Turn body Right");
						maxTurn = -0.4;
					}
					else if (users_info_old[user_id_xtion].x < 3 && users_info_old[user_id_xtion].x > 1 
											&& users_info_old[user_id_xtion].y > 0.5)
					{
						ROS_INFO("XTion: Turn body Left");
						maxTurn = 0.3;
					}
					else if (users_info_old[user_id_xtion].x < 3 && users_info_old[user_id_xtion].x > 1 
											&& users_info_old[user_id_xtion].y < -0.5)
					{
						ROS_INFO("XTion: Turn body Right");
						maxTurn = -0.3;
					}
					else
					{
						ROS_INFO("XTion: No Turn");
						maxTurn = 0;    
					}
				}
		    		else
				{
					//TODO:
				}
		    
				//MOVING FORWARD
				if (user_id_xtion > -1)
				{
					//Move body using positions from sensor
					if (users_info_old[user_id_xtion].x > 1.5)
					{
						ROS_INFO("Move Forward");
						maxVel = 0.2;
					}
					else if (users_info_old[user_id_xtion].x < 0.5 && users_info_old[user_id_xtion].x > 0)
					{
						//ROS_INFO("Move Back");
						//maxVel = -0.1;
					}

					distance_check = abs(users_info_old[user_id_xtion].x - prev_marker_head_x);
					//If the sensor reading jumps, stop moving
					if (distance_check >= 0.5)
					{
						ROS_INFO("Sensor Reading Jumped");
						maxVel = 0;
					}
					if (users_info_old[user_id_xtion].x < 1.5)
					{
						feedback_.inRange = 1;
					}
					else
					{
						feedback_.inRange = 0;
					}
					as_.publishFeedback(feedback_);
				}
				else if (userid > -1)
				{
					//TODO: try to move using the axis info
					ROS_INFO("Cannot match user with sensor! But I can still detect with Axis");
				}
				else
				{
					ROS_INFO("Cannot match user with both sensors!");
					neck_x_rotation_ = 0;
					neck_z_rotation_ = 0;
					maxVel = 0;
					maxTurn = 0;
				}

				//recording previous data
				prev_x_pos = x_pos;
				prev_y_pos = y_pos;
				prev_user_id_xtion = user_id_xtion;
				if (user_id_xtion > -1)
				{
					prev_marker_head_x = users_info_old[user_id_xtion].x;
					prev_marker_head_y = users_info_old[user_id_xtion].y;
				}
			}
			else
			{
				ROS_INFO("User not deteced, stop movement!");
				neck_x_rotation_ = 0;
				neck_z_rotation_ = 0;
				maxVel = 0;
				maxTurn = 0;
			}
		      
		    //Send commands to head servos
		    cmdhead_.neck_x_rotation = neck_x_rotation_;
		    cmdhead_.neck_z_rotation = neck_z_rotation_;
		    cmdhead_.mouth = mouth_;
		    cmdhead_.upper_head = upper_head_;
		    cmdhead_.left_eye = left_eye_;
		    cmdhead_.right_eye = right_eye_;
		    
		    pub_.publish(cmdhead_);
		    
		    //Send commands to base servos
		    cmdvel_.linear.x = maxVel;
		    cmdvel_.angular.z = maxTurn;
		    
//		    ROS_INFO("MaxTurn: %f", maxTurn);

		    //Will publish base servo commands if maxTurn or maxVel is not 0
		    //or if either maxTurn or maxVel is 0 and previous command is not 0
		    //This will keep it from publishing everytime
		    if (maxTurn != 0 || (maxTurn == 0 && previous_cmdvel_.angular.z != 0) || maxVel != 0 || (maxVel == 0 && previous_cmdvel_.linear.x != 0))
		    {
			  //pub_base.publish(cmdvel_);
		    }

		    previous_cmdvel_.linear.x = maxVel;
		    previous_cmdvel_.angular.z = maxTurn;


		}
	}

	//Send a goal to the server
	void send(int goal)
	{
		face_detection::identifyGoal newGoal;
		newGoal.numberOfSamples = goal;
	//	ROS_INFO("Goal: %i\n", goal);
		//Once again, have to used boost::bind because you are inside a class
		FaceDetectionClient.sendGoal(newGoal, boost::bind(&UserTrackingAction::doneCb, this, _1, _2),
					boost::bind(&UserTrackingAction::activeCb, this),
					boost::bind(&UserTrackingAction::feedbackCb, this, _1));
	}
private:
	actionlib::SimpleActionClient<face_detection::identifyAction> FaceDetectionClient;
	std::string action_name;
	
	std::string goal_;
	int neck_pos_x;
	int neck_pos_z;
	
	int x_pos;
	int y_pos;
	int prev_x_pos;
	int prev_y_pos;
	int prev_user_id_xtion;
	
	float estimated_x;
	float prev_marker_head_x;
	float prev_marker_head_y;
	float distance_check;
	
	bool first_user_detected;
	
	std::vector<UserInfo> users_info_old;
	std::vector<UserInfo> users_info_new;
	ros::Duration last_message_received;
	ros::Time latest_time;
	
	drrobot_h20_player::HeadCmd cmdhead_;
	geometry_msgs::Twist cmdvel_;
	geometry_msgs::Twist previous_cmdvel_;       
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Publisher pub_base;
};
			
int main (int argc, char **argv)
{

	ros::init(argc, argv, "user_tracking");
	
	UserTrackingAction UserTracking("user_tracking");
	
	ros::init(argc, argv, "test_faceDetection_client");
	int sampleGoal = -1;
	//Usage check to make sure the client is being used properly

	//Initialize FaceDetection client
	UserTrackingAction client(ros::this_node::getName());
	ROS_INFO("Sent Goal %d To Server...", sampleGoal);

	ros::Rate loop_rate(1);

	while(!received_goal){
		ROS_INFO("waiting");
		ros::spinOnce();
		loop_rate.sleep();
	}

	if (received_goal == true)
	{
	  client.send(sampleGoal);

	  ROS_INFO("Sent Goal %d To Server...", sampleGoal);
		
	  received_goal = false;
	}

	ros::spin();


	return 0;
}
