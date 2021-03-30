#ifndef NAVIGATIONSERVER_H
#define NAVIGATIONSERVER_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <ctime>
#include <string>
#include <tangy_move/navServerCmd.h>

#define PI 3.14159265
class navigationServer {
	public:
		// functions
		navigationServer();
		~navigationServer();
		void initialize(ros::NodeHandle nh);
		void move_straight (double distance);
		void move_straight_manual (double distance);
		void move(std::string frame, double x,double y);
		void face(std::string frame, double x, double y);
		void rotate(std::string frame, double theta);
		void rotate_no_wait(std::string frame, double theta);
		void rotate_manual (double angle);
		void pause();
		void unpause();

	private:
		// variables
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> actionClient;
		tf::TransformListener tfListener; 
		ros::NodeHandle nh;

		// functions
		double checkDistTravelled(geometry_msgs::PoseStamped start); 
		bool checkGoalDist(geometry_msgs::PoseStamped goalPose, float tolerance);
		bool checkGoalAngle(double theta);

		// rotate loop
		void rotate_loop(std::string frame, double goalTheta);
		
		// move straight loop
		void move_straight_loop (double distance, geometry_msgs::PoseStamped start);
		
		// control callback for stopping navigation
		void controlCb(const std_msgs::String::ConstPtr& msg);
		void debug_print(std::string string);
		// variable telling us whether or not to stop
		bool paused;

		geometry_msgs::Twist createTwist(double theta);
		geometry_msgs::PoseStamped createPose(std::string frame, double x, double y, double z, double ax, double ay, double az);
		geometry_msgs::PoseStamped getRobotPose(std::string frame);
		geometry_msgs::PoseStamped transformPose(std::string frameTo, geometry_msgs::PoseStamped poseFrom);
		ros::Publisher velCommand;
		ros::Publisher marker_pub;
		ros::Publisher pose_pub;
		visualization_msgs::Marker marker;

		// debug pubkisher
		ros::Publisher debug_pub;
		std_msgs::String debug_string;
};



#endif
