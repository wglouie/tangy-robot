#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <face_detection/identifyAction.h>
#include <drrobot_h20_player/HeadCmd.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>

#define RESOLUTION_X 640
#define RESOLUTION_Y 480

int marker_user_id;
float marker_head_x[15];
float marker_head_y[15];

class UserTrackingClient{
public:

	ros::NodeHandle node_;  
	ros::Subscriber head_pos_sub_;

	ros::NodeHandle nh_;
	ros::Subscriber head_coord_sub_;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	
	UserTrackingClient(std::string name):
		//Set up the client. It's publishing to topic "test_action", and is set to auto-spin
		ac("face_detection/face_detection", true),
		MoveBaseClient("move_base", true),
		//Stores the name
		action_name(name)
	{
		//Get connection to FaceDetection server
		ROS_INFO("%s Waiting For FaceDetection Server...", action_name.c_str());
		//Wait for FaceDetection connection to be valid
		ac.waitForServer();
		ROS_INFO("%s Got FaceDetection Server...", action_name.c_str());

		ROS_INFO("%s Waiting For Navigation Server...", action_name.c_str());
		//Wait for Navigation connection to be valid
		MoveBaseClient.waitForServer();
		ROS_INFO("%s Got Navigation Server...", action_name.c_str());

		pub_ = n_.advertise<drrobot_h20_player::HeadCmd>("cmd_head", 1);
		pub_base = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		
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
	
	void neckpositionCB(const drrobot_h20_player::HeadCmd::ConstPtr& neck_pos)
	{
		neck_pos_x = neck_pos->neck_x_rotation;
		neck_pos_z = neck_pos->neck_z_rotation;	  
	}
	
	//Gets head coordinates for user 1 from sensor
	void markerCoordinatesCB(const visualization_msgs::Marker::ConstPtr& marker_pos)
	{  
		int num;
		
		marker_user_id = marker_pos->id;
		num = marker_user_id - 1;
		
		if (marker_pos->points.size() > 0)
		{
		  marker_head_x[num] = marker_pos->points[4].x;
		  marker_head_y[num] = marker_pos->points[4].y;
		}
	}

	// Called every time feedback is received for the goal
	void feedbackCb(const face_detection::identifyFeedbackConstPtr& feedback)
	{
		//Subscribe for neck position values and skeleton transform coordinates
		head_pos_sub_ = node_.subscribe<drrobot_h20_player::HeadCmd>("head_pose", 1, boost::bind(&UserTrackingClient::neckpositionCB, this, _1));
		head_coord_sub_ = nh_.subscribe<visualization_msgs::Marker>("tracked_users_markers" , 1000, boost::bind(&UserTrackingClient::markerCoordinatesCB, this, _1));

		int neck_x_rotation_ = 0;
		int neck_z_rotation_= 0;
		int mouth_= 0;
		int upper_head_ = 0;
		int left_eye_ = 0;
		int right_eye_ = 0;

		double maxVel = 0;
		double maxTurn = 0;
		bool dirty = false;
		
		int userid = 0;
		
		marker_user_id = -1;
	
		ROS_INFO("Got Feedback of Progress to Goal: %d", feedback->currentSampleNumber);
		ROS_INFO("Faces: %d", feedback->numberOfFaces);
		
		//Sleep if no detection at beginning
		//Still a problem if no one detected when people walk off screen
//		if (feedback->identified_users_size == 0)
//		{
		  //if neck position is at maxima/minima and no one detected, turn robot in direction of neck position
		  //else sleep
//		  usleep(1000000);
//		}
//		else
//		{
		  while (feedback->identified_users_size == 0);
		  while (marker_user_id == -1);
		  
		  ROS_INFO("Names: %s", feedback->users_identified[0].c_str());
		  ROS_INFO("X: %d", feedback->x[userid]);
		  ROS_INFO("Y: %d", feedback->y[userid]);
		  int x_pos = feedback->x[userid] - (RESOLUTION_X/2);
		  int y_pos = feedback->y[userid] - (RESOLUTION_Y/2);
		  ROS_INFO("DX: %d", x_pos);
		  ROS_INFO("DY: %d", y_pos);
  		  ROS_INFO("Neck position X: %d", neck_pos_x);
		  ROS_INFO("Neck position Z: %d", neck_pos_z);

		  ROS_INFO("marker_head_x[%d] = %f", marker_user_id - 1, marker_head_x[marker_user_id - 1]);
		  ROS_INFO("marker_head_y[%d] = %f", marker_user_id - 1, marker_head_y[marker_user_id - 1]);

		  neck_x_rotation_ = 0;
		  neck_z_rotation_ = 0;
		  mouth_ = 0;
		  upper_head_ = 0;
		  left_eye_ = 0;
		  right_eye_ = 0;
		  
//		  maxVel = 0;
		  dirty = true;
		  
		  //Converts position of a tracked face to meters (from pixels)
		  //1 meter = 3779.528 pixels
		  convert = feedback->x[userid] / 3779.528;
		  if (feedback->x[userid] < (RESOLUTION_X/2))
		  {
		    convert = convert * -1;
		  }
		  ROS_INFO("convert: %f", convert);
		  
		  user_num = 0;
		  
		  if (convert <= (marker_head_y[marker_user_id - 1] + 0.2) && convert > (marker_head_y[marker_user_id - 1] - 0.2))
		  {
		    ROS_INFO("---------------------------------");
		    current_name = feedback->users_identified[0];
		    if (prev_name != current_name)
		    {
		      if (feedback->users_identified[0] != "   ")
		      {
			names.push_back(current_name);
			prev_name = current_name;
			ROS_INFO("New user recognized");
		      }
		    }
		  }
		  
		  //Check for previous identified users
		  if (names.size() > 0)
		  {
		    for (int i = 0; i < marker_user_id; i++)
		    {
		      if (names[i] == current_name)
		      {
			ROS_INFO("Previous user recognized");
			user_num = i;
			break;
		      }
		    }
		  }
		  ROS_INFO("user_num: %d", user_num);
		  
		  //Set active user and inactive users (active = 1, inactive = 0)
		  for (int j = 0; j < names.size(); j++)
		  {
		    if (names[j] == names[marker_user_id - 1] && names[j] == feedback->users_identified[0])
		    {
		      active_user[j] = 1;
		    }
		    else
		    {
		      if (feedback->users_identified[0] == "   ")
		      {
			active_user[j] = 0;
		      }
		      active_user[j] = 0;
		    }
		  }
		  
		  if (names.size() > 0)
		  {
		    //To track a certain person: change 'names[user_num]' to the person's name
		    if (feedback->users_identified[0] == names[user_num] && active_user[marker_user_id - 1] == 1)
		    {
		      //Turn body using of head transform from sensor information
		      if (feedback->numberOfFaces == 1 || feedback->numberOfFaces == 2)
		      {
			if (marker_head_x[marker_user_id - 1] <= 1 && marker_head_y[marker_user_id - 1] > 0.15) // 1400 is the minimal, previously: feedback->numberOfFaces == 1 && neck_pos_z <= 2000
			{
				  ROS_INFO("Turn body Right");
				  maxTurn = 0.4;
			}
			else if (marker_head_x[marker_user_id - 1] <= 1 && marker_head_y[marker_user_id - 1] < -0.15) //5500 is the max, prev:feedback->numberOfFaces == 1 && neck_pos_z >= 4900
			{
				  ROS_INFO("Turn body Left");
				  maxTurn = -0.4;
			}
			else if (marker_head_x[marker_user_id - 1] < 3 && marker_head_x[marker_user_id - 1] > 1 && marker_head_y[marker_user_id - 1] > 0.5)
			{
				  ROS_INFO("Turn body Right");
				  maxTurn = 0.3;
			}
			else if (marker_head_x[marker_user_id - 1] < 3 && marker_head_x[marker_user_id - 1] > 1 && marker_head_y[marker_user_id - 1] < -0.5)
			{
				  ROS_INFO("Turn body Left");
				  maxTurn = -0.3;
			}
			else if (neck_pos_z < 2000 || neck_pos_z > 4900)
			{
				  ROS_INFO("Stop Turning");
				  maxTurn = 0;
			}
			else
			{
				  ROS_INFO("No Turn body");
				  maxTurn = 0;    
			}
		      }
		  
		      //Pan head using AXIS camera and FaceDetection coordinates
		      //Modified x_pos tolerance to center the robot's face
		      if (x_pos < -60)
		      {
			    ROS_INFO("Pan Right");
			    neck_z_rotation_ = -70;
		      }
		      else if (x_pos > 200)
		      {
			    ROS_INFO("Pan Left");
			    neck_z_rotation_ = 70;
		      }
		      else
		      {
			    ROS_INFO("Don't Move");
			    neck_z_rotation_ = 0;
		      }
		      
		      //Tilt head using AXIS camera and FaceDetection coordinates
		      if (y_pos < -85)
		      {
			    ROS_INFO("Tilt Up");
			    neck_x_rotation_ = 40;
		      }
		      else if (y_pos > 85)
		      {
			    ROS_INFO("Tilt Down");
			    neck_x_rotation_ = -40;
		      }
		      else
		      {
			    ROS_INFO("Don't Move");
			    neck_x_rotation_ = 0;
		      }
		      
		      //Turn body using positions from sensor 
/*		      if (marker_head_y[marker_user_id - 1] < -0.3)
		      {
			    ROS_INFO("Left Turn");
			    maxTurn = -0.3;
		      }
		      else if (marker_head_y[marker_user_id - 1] > 0.3)
		      {
			    ROS_INFO("Right Turn");
			    maxTurn = 0.3;
		      }
		  
*/		      //Move body using positions from sensor
		      if (marker_head_x[marker_user_id - 1] > 1.5)
		      {
			    ROS_INFO("Move Forward");
			    maxVel = 0.2;
		      }
		      else if (marker_head_x[marker_user_id - 1] < 0.5 && marker_head_x[marker_user_id - 1] > 0)
		      {
			    //ROS_INFO("Move Back");
			    //maxVel = -0.1;
		      }
		      
		      //Turns robot first and then moves forward
		      //Moves forward if robot within 0.15 range and beyond 1
		      //Turns left or right if beyond 1 (or less than 0.5) and out of range of 0.5 (or 0.3) 
/*		      //Position robot using sensor to track user
		      if ((marker_head_x[marker_user_id - 1] > 1 && marker_head_y[marker_user_id - 1] < -0.5) || (marker_head_x[marker_user_id - 1] < 0.5 &&  marker_head_y[marker_user_id - 1] < -0.3))
		      {
			ROS_INFO("Left Turn");
			maxTurn = -0.3;
		      }
		      else if ((marker_head_x[marker_user_id - 1] > 1 && marker_head_y[marker_user_id - 1] > 0.5) || (marker_head_x[marker_user_id - 1] < 0.5 &&  marker_head_y[marker_user_id - 1] > 0.3))
		      {
			ROS_INFO("Right Turn");
			maxTurn = 0.3;
		      }
		      else if (marker_head_x[marker_user_id - 1] > 1.5 && marker_head_y[marker_user_id - 1] < 0.15 && marker_head_y[marker_user_id - 1] > -0.15)
		      {
			ROS_INFO("Move Forward");
			maxVel = 0.1;
		      }
		      else if (marker_head_x[marker_user_id - 1] < 0.5 && marker_head_y[marker_user_id - 1] < 0.15 && marker_head_y[marker_user_id - 1] > -0.15)
		      {
			ROS_INFO("Move Back");
			maxVel = -0.1;
		      }
*/
		      //Send coordinates to navigate to user if farther than 1.5 meters, if not, cancel
		      if (marker_head_x[marker_user_id - 1] > 1.5)
		      {
			//Initialize where to send coordinates
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.frame_id = "/base_link";

			//Coordinates calculated to be a distance away from user
			//goal.target_pose.pose.position.x = marker_head_x[marker_user_id - 1] - 0.5;
			if (marker_head_y[marker_user_id - 1] > 0.4)
			{
				//goal.target_pose.pose.position.y = marker_head_y[marker_user_id - 1] - 0.3;
			}
			else if (marker_head_y[marker_user_id - 1] < -0.4)
			{
				//goal.target_pose.pose.position.y = marker_head_y[marker_user_id - 1] + 0.3;
			}
			//goal.target_pose.pose.position.x = 0.3;
			goal.target_pose.pose.orientation.w = 1.0;

			ROS_INFO("Goal target pos x: %f", goal.target_pose.pose.position.x);
			ROS_INFO("Goal target pos y: %f", goal.target_pose.pose.position.y);

			//Send coordinates
			//MoveBaseClient.sendGoal(goal);
			//float dur = 2.0;
			//MoveBaseClient.waitForResult(ros::Duration(dur));
			//sleep(1);
		      }
		      else
		      {
			    MoveBaseClient.cancelAllGoals();
		      }
		    }
		    else
		    {
		      ROS_INFO("Stop Movements");
		      neck_x_rotation_ = 0;
		      neck_z_rotation_ = 0;
		      
		      //Change user_num if current name and names do not match
		      if (user_num != (marker_user_id - 1) && user_num < marker_user_id /*&& feedback->users_identified[0] != names[user_num]*/)
		      {
			user_num = user_num + 1;
		      }
		      else
		      {
			user_num = 0;
		      }
		    }
		  
		  //Send commands to head servos
		  cmdhead_.neck_x_rotation = neck_x_rotation_;
		  cmdhead_.neck_z_rotation = neck_z_rotation_;
		  cmdhead_.mouth = mouth_;
		  cmdhead_.upper_head = upper_head_;
		  cmdhead_.left_eye = left_eye_;
		  cmdhead_.right_eye = right_eye_;
		  
//		  pub_.publish(cmdhead_);
		  
		  //Send commands to base servos
		  cmdvel_.linear.x = maxVel;
		  cmdvel_.angular.z = maxTurn;
		  
//		  ROS_INFO("MaxTurn: %f", maxTurn);

		  //Will publish base servo commands if maxTurn or maxVel is not 0
		  //or if either maxTurn or maxVel is 0 and previous command is not 0
		  //This will keep it from publishing everytime
		  if (maxTurn != 0 || (maxTurn == 0 && previous_cmdvel_.angular.z != 0) || maxVel != 0 || (maxVel == 0 && previous_cmdvel_.linear.x != 0))
		  {
		  	pub_base.publish(cmdvel_);
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
		ac.sendGoal(newGoal, boost::bind(&UserTrackingClient::doneCb, this, _1, _2),
					boost::bind(&UserTrackingClient::activeCb, this),
					boost::bind(&UserTrackingClient::feedbackCb, this, _1));
	}
private:
	actionlib::SimpleActionClient<face_detection::identifyAction> ac;
	std::string action_name;
	
	int neck_pos_x;
	int neck_pos_z;
	int user_num;
	int active_user[];
	
	float prev_head_pos_x;
	float prev_head_pos_y;
	float convert;
	
	std::string current_name;
	std::string prev_name;
	std::vector<std::string> names;
	
	drrobot_h20_player::HeadCmd cmdhead_;
	geometry_msgs::Twist cmdvel_;
	geometry_msgs::Twist previous_cmdvel_;       
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Publisher pub_base;
};
			
int main (int argc, char **argv)
{
	ros::init(argc, argv, "test_faceDetection_client");
	int sampleGoal = 50;
	//Usage check to make sure the client is being used properly


	//Initialize the client
	UserTrackingClient client(ros::this_node::getName());

	client.send(sampleGoal);

	ROS_INFO("Sent Goal %d To Server...", sampleGoal);

	ros::spin();


	return 0;
}
