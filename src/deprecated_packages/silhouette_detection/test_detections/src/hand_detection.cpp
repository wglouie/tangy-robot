#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <boost/algorithm/string/predicate.hpp>
	
	int rate , id , users_detected;
	double scale , lifetime;
	std::string tf_prefix , fixed_frame , ns;
	
	std::string tracked_user = 0;

	// Marker object needed to publish 
	// used_tracker_markers
	visualization_msgs::Marker markers;

	//Read and set supported parameters for this node
	void readParams(const ros::NodeHandle& nh)
	{
		std::stringstream ss;		
		
		nh.param<int>("rate" , rate ,20);
		nh.param<int>("id" , id , 0);
		nh.param<double>("scale" , scale , 0.05);
		nh.param<double>("lifetime" , lifetime , 0);
		nh.param<std::string>("tf_prefix" , tf_prefix , "hd_frames");		
		nh.param<std::string>("fixed_frame" , fixed_frame , "openni_depth_frame");
		ss << tf_prefix << "/" << fidex_frame;
		fixed_frame = ss.str();

		nh.param<std::string>("ns" , ns , "hand_detect_markers");	
	}

	void initialize_markers(const ros::NodeHandle& nh)
	{
		
		markers.header.frame_id = fixed_frame;
		markers.ns = ns;
		markers.id = id;
		markers.type = visualization_msgs::Marker::POINTS;
		markers.action = visualization_msgs::Marker::ADD;
		markers.lifetime = ros::Duration(0);
		markers.scale.x = scale;
		markers.scale.y = scale;
		markers.color.r = 0.0;
		markers.color.g = 1.0;
		markers.color.b = 0.0;
		markers.color.a = 1.0;
	
		ROS_INFO("Markers initialized");
	}

	//Declaring the main function of this node
	int main (int argc , char **argv)
	{
		//Local variables		
		std::vector<std::string> all_frames , user_frames;
		tf::TransformListener tf_listener;		
				
		ros::init(argc , argv , "hand_detection");
		ROS_INFO("Initializing hand_detection node...");

		ros::NodeHandle nh("~");
		
		//Function called so that the node can
		// look for external parameters
		readParams(nh);
		
		//Setting the loop rate for this node
		ros::Rate loop_rate(rate);
	
		ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("tracked_user_markers" , 1000);

		initialize_markers(nh);

		//Waiting for transforms
		try
		{
			tf_listener.waitForTransform(fixed_frame.c_str() , fixed_frame.c_str() , ros::Time::now() , ros::Duration(10.0));
		}		
		catch(tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}
		
		//Main loop of this node
		while(ros::ok())
		{
			user_frames.clear();
			tf_listener.getFrameStrings(all_frames);
			if(all_frames.size() < 1)
			{
				loop_rate.sleep();
				continue;
			}
			for(int i=0 ; i<all_frames.size() ; i++)
			{
				
			}
		}
	}


/*
	std::stringstream ss;
	std::string tf_prefix , fixed_frame;
	int rate;
	
	visualization_msgs::Marker markers;
	std::vector<std::string> skeleton_frames;

void initialize_markers(const ros::NodeHandle& nh)
{
	int id;	
	double scale , lifetime;
	std::string ns;

	nh.param<double>("scale" , scale , 0.07);
	nh.param<double>("lifetime" , lifetime , 0);
	nh.param<std::string>("ns" , ns , "skeleton_markers");
	nh.param<int>("id" , id , 0);

	
	markers.header.frame_id = fixed_frame;
	markers.ns = ns;
	markers.id = id;
	markers.type = visualization_msgs::Marker::POINTS;
	markers.action = visualization_msgs::Marker::ADD;
	markers.lifetime = ros::Duration(lifetime);
	markers.scale.x = scale;
	markers.scale.y = scale;
	markers.color.r = 0.0;
	markers.color.g = 1.0;
	markers.color.b = 0.0;
	markers.color.a = 1.0;

	ROS_INFO("Markers initialized");
}

int main(int argc , char **argv)
{
	ros::init(argc , argv , "markers_from_tf");
	ROS_INFO("Initializing skeleton markers node...");
	
	//creating a nodehandle for private parameters
	ros::NodeHandle nh("~");

	nh.param<int>("rate" , rate , 20);

	ros::Rate loop_rate(rate);

	//retrieving parameter tf_prefix from parameters server default value = '/skeleton'
	nh.param<std::string>("tf_prefix", tf_prefix , "/skeleton");
	
	//parameter fixed_frame does not need to be changed from default 'openni_depth_frame'	
	nh.param<std::string>("fixed_frame" ,fixed_frame , "openni_depth_frame");
	
	//prepend tf_prefix to the fixed frame	
	ss << tf_prefix << "/" << fixed_frame;	
	fixed_frame = ss.str();

	tf::TransformListener tf_listener;

	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("skeleton_markers" , 1000);

	initialize_markers(nh);
	try{
		tf_listener.waitForTransform(fixed_frame.c_str(), fixed_frame.c_str() , 
						ros::Time() , ros::Duration(10.0));
	}
	catch(tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
	}	
	std::vector<std::string> actual_frames;
	while(ros::ok())
	{
		actual_frames.clear();
		skeleton_frames.clear();
		tf_listener.getFrameStrings(actual_frames);
		if(actual_frames.size() < 1)
		{	
			loop_rate.sleep();
			continue;
		}
		for (int i = 0 ; i < actual_frames.size() ; i++){
			//ROS_INFO("frame %d = %s ", i , actual_frames[i].c_str());
			if(boost::starts_with(actual_frames[i].c_str() , tf_prefix))
			{
				skeleton_frames.push_back(actual_frames[i]);
			}

		}
		markers.header.stamp = ros::Time::now();
		markers.points.clear();	

		for(int i = 0; i < skeleton_frames.size() ; i++)
		{
			if(skeleton_frames[i] == fixed_frame)
				continue;
			
			try
			{
				geometry_msgs::Point position;
				tf::StampedTransform transform;
				tf_listener.lookupTransform(fixed_frame, skeleton_frames[i], ros::Time(0), transform);
				position.x = transform.getOrigin().x();
				position.y = transform.getOrigin().y();
				position.z = transform.getOrigin().z();
				markers.points.push_back(position);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("Exception when looking at %s and %s",skeleton_frames[i].c_str() , fixed_frame.c_str());
				continue;
			}
		}
		ROS_INFO("Markers size is %d" , markers.points.size());
		marker_pub.publish(markers);
		loop_rate.sleep();
	}
	
}
*/

