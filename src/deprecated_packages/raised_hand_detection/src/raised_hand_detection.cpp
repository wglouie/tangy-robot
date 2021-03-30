#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <boost/algorithm/string/predicate.hpp>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <vector>

#define LEFT_HAND 0
#define LEFT_ELBOW 1
#define LEFT_SHOULDER 2
#define NECK 3
#define HEAD 4
#define RIGHT_SHOULDER 5
#define RIGHT_ELBOW 6
#define RIGHT_HAND 7

visualization_msgs::Marker r_marker , line_marker;
std::string ns , tf_prefix , fixed_frame;
int rate , id;
double scale , lifetime , color_a , color_b , color_g , color_r;

std::stringstream sstr;
std::vector<visualization_msgs::Marker> usr_markers;
std::vector<int> users_raising_hand;
std::vector<int>::iterator iter;

// Callback for the /tracked_users_markers topic
void trackedMarkersCallback(const visualization_msgs::Marker::ConstPtr& marker_msg)
{
	if(marker_msg->points.size() > 0)
	{
		// Add this user to 'usr_markers' vector
		usr_markers.push_back(*marker_msg);
	}
}

// Reading parameters that may be on ros parameter server
void readParams(const ros::NodeHandle& nh)
{
		std::stringstream ss;
		nh.param<std::string>("ns" , ns , "raised_hand_detection");
		nh.param<int>("/"+ns+"/rate" , rate ,10);
		nh.param<int>("/"+ns+"/id" , id , 0);
		nh.param<double>("/"+ns+"/scale" , scale , 0.05);
		nh.param<double>("/"+ns+"/lifetime" , lifetime , 0);
		nh.param<std::string>("/"+ns+"/tf_prefix", tf_prefix , "/upper_frames");
		nh.param<std::string>("/"+ns+"/fixed_frame" , fixed_frame , "openni_depth_frame");
		nh.param<double>("/"+ns+"/color/a" , color_a , 1.0);
		nh.param<double>("/"+ns+"/color/b" , color_b , 0.0);
		nh.param<double>("/"+ns+"/color/g" , color_g , 0.0);
		nh.param<double>("/"+ns+"/color/r" , color_r , 1.0);
		ss << tf_prefix << "/" << fixed_frame;
		fixed_frame = ss.str();
}

// Initializing 'r_marker' and 'line_marker'
void initialize_markers(const ros::NodeHandle& nh)
{
		r_marker.header.frame_id = fixed_frame;
		r_marker.ns = ns;
		r_marker.id = 10;
		r_marker.type = visualization_msgs::Marker::POINTS;
		r_marker.action = visualization_msgs::Marker::ADD;
		r_marker.lifetime = ros::Duration(0);
		r_marker.scale.x = scale;
		r_marker.scale.y = scale;
		r_marker.color.r = 0.0;
		r_marker.color.g = 1.0;
		r_marker.color.b = 0.0;
		r_marker.color.a = 1.0;

		line_marker.header.frame_id = fixed_frame;
		line_marker.ns = ns;
		line_marker.id = 11;
		line_marker.type = visualization_msgs::Marker::LINE_STRIP;
		line_marker.action = visualization_msgs::Marker::ADD;
		line_marker.lifetime = ros::Duration(0);
		line_marker.scale.x = 0.03;
		line_marker.scale.y = 0.03;
		line_marker.color.r = 1.0;
		line_marker.color.g = 0.0;
		line_marker.color.b = 0.0;
		line_marker.color.a = 1.0;
}

// Setting 'line_marker.points' according to an expected vector of points (user marker points)
void assign_line_marker_points(std::vector<geometry_msgs::Point> p , int size)
{
	line_marker.points.clear();
	if(size==8)						//expected size
	{
		line_marker.points.push_back(p[LEFT_HAND]);
		line_marker.points.push_back(p[LEFT_ELBOW]);
		line_marker.points.push_back(p[LEFT_SHOULDER]);
		line_marker.points.push_back(p[NECK]);
		line_marker.points.push_back(p[HEAD]);
		line_marker.points.push_back(p[NECK]);
		line_marker.points.push_back(p[RIGHT_SHOULDER]);
		line_marker.points.push_back(p[RIGHT_ELBOW]);
		line_marker.points.push_back(p[RIGHT_HAND]);
	}
} 	

int main (int argc , char **argv)
{
	ros::init(argc , argv , "raised_hand_detection");
	ROS_INFO("Initializing raised_hand_detection node...");
	
	ros::NodeHandle nh;
	readParams(nh);
	ros::Subscriber users_markers_sub = nh.subscribe("tracked_users_markers" , 100 , trackedMarkersCallback);
	ros::Rate loop_rate(rate);
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("highlighted_user" , 100);
	initialize_markers(nh);
	
	while(ros::ok())
	{
		usr_markers.clear();
		ros::spinOnce();

		// For every user marker received by trackedMarkersCallback()
		for(int i=0 ; i<usr_markers.size() ; i++)
		{
			
			// IF the user's left hand is raised
			if(((usr_markers[i].points[LEFT_HAND].z > usr_markers[i].points[HEAD].z + 0.03)&&
			(usr_markers[i].points[LEFT_HAND].z > usr_markers[i].points[LEFT_ELBOW].z + 0.02))&&
			((sqrt(pow(usr_markers[i].points[LEFT_HAND].x - usr_markers[i].points[HEAD].x , 2) + 
			pow(usr_markers[i].points[LEFT_HAND].y - usr_markers[i].points[HEAD].y , 2))) > 0.12))
			{
				// IF this user ID is not in users_raising_hand yet					
				iter = std::find(users_raising_hand.begin() , users_raising_hand.end() , usr_markers[i].id);	
				if(iter == users_raising_hand.end())								
				{
					//ROS_INFO("User %d: HAND IS RAISED", usr_markers[i].id);
					
					// Add this user to the vector of users raising hand
					users_raising_hand.insert(users_raising_hand.begin() , usr_markers[i].id);
				}
			}
			// IF the user's right hand is raised
			else if(((usr_markers[i].points[RIGHT_HAND].z > usr_markers[i].points[HEAD].z + 0.03)&&
				(usr_markers[i].points[RIGHT_HAND].z > usr_markers[i].points[RIGHT_ELBOW].z + 0.02))&&
				((sqrt(pow(usr_markers[i].points[RIGHT_HAND].x - usr_markers[i].points[HEAD].x , 2) + 
				pow(usr_markers[i].points[RIGHT_HAND].y - usr_markers[i].points[HEAD].y , 2))) > 0.12))
			{
				// IF this user ID is not in users_raising_hand yet					
				iter = std::find(users_raising_hand.begin() , users_raising_hand.end() , usr_markers[i].id);	
				if(iter == users_raising_hand.end())								
				{
					//ROS_INFO("User %d: HAND IS RAISED", usr_markers[i].id);
					
					// Add this user to the vector of users raising hand
					users_raising_hand.insert(users_raising_hand.begin() , usr_markers[i].id);
				}
			}
			// IF the user is not raising hand
			else
			{
				// IF this user ID is in users_raising_hand					
				iter = std::find(users_raising_hand.begin() , users_raising_hand.end() , usr_markers[i].id);
				if(iter != users_raising_hand.end())								
				{
					//ROS_INFO("User %d: HAND IS LOWERED", usr_markers[i].id);
					
					// Remove this user from the vector of users raising hand
					users_raising_hand.erase(iter);
				}
			}
		}
		
		r_marker.header.stamp = ros::Time::now();
		line_marker.header.stamp = r_marker.header.stamp;
		// If there is any user raising his/her hand
		if(users_raising_hand.size() > 0)
		{
			int index;
			// Look for the user that raised hand before any other
			for(index = 0 ; index < usr_markers.size() ; index++)
			{
				if(usr_markers[index].id == users_raising_hand.back())
					break;
			}
			// If found this user, copy its points to r_marker.points
			if(index != usr_markers.size())
			{
				r_marker.points = usr_markers[index].points;
				r_marker.text = usr_markers[index].text;			
			}			
			else
				r_marker.points.clear();
		}
		else
			r_marker.points.clear();
		// Publishing the red markers
		marker_pub.publish(r_marker);
		
		assign_line_marker_points(r_marker.points , r_marker.points.size());
		// Publishgint the line markers
		marker_pub.publish(line_marker);

		loop_rate.sleep();
	}
}
