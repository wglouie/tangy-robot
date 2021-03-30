#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <boost/algorithm/string/predicate.hpp>
#include <algorithm>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <vector>

	int rate , id; 
	double scale , lifetime , color_a , color_b , color_g , color_r;
	std::string tf_prefix , fixed_frame , ns;
	std::vector<int32_t> users_vector;
	std::vector<int32_t> id_vector;
	std::vector<int32_t> published_ids;
	std::vector<int32_t>::iterator iter;
	std::stringstream sstr;

	visualization_msgs::Marker markers;
	std::vector<visualization_msgs::Marker> markers_vector;

	//Skeleton structure for hand raising (Using upper body only. You can add more if necessary)
	struct Skeleton
	{
		geometry_msgs::Point left_hand;
		geometry_msgs::Point left_elbow;
		geometry_msgs::Point left_shoulder;
		geometry_msgs::Point neck;
		geometry_msgs::Point head;
		geometry_msgs::Point right_shoulder;
		geometry_msgs::Point right_elbow;
		geometry_msgs::Point right_hand;
	};

	Skeleton user_skeleton;	

	//Callback for the '/users'  topic 
	void usersCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
	{
		users_vector.clear();
		for(int i = 0 ; i < msg->data.size() ; i++)
		{
			users_vector.push_back(msg->data[i]);
		}
	}

	//Read and set expected parameters for this node
	void readParams(const ros::NodeHandle& nh)
	{
		std::stringstream ss;
		nh.param<std::string>("ns" , ns , "users_markers");
		nh.param<int>("/"+ns+"/rate" , rate ,10);
		nh.param<int>("/"+ns+"/id" , id , 0);
		nh.param<double>("/"+ns+"/scale" , scale , 0.05);
		nh.param<double>("/"+ns+"/lifetime" , lifetime , 0);
		nh.param<std::string>("/"+ns+"/tf_prefix", tf_prefix , "upper_frames");
		nh.param<std::string>("/"+ns+"/fixed_frame" , fixed_frame , "openni_depth_frame");
		nh.param<double>("/"+ns+"/color/a" , color_a , 1.0);
		nh.param<double>("/"+ns+"/color/b" , color_b , 0.0);
		nh.param<double>("/"+ns+"/color/g" , color_g , 1.0);
		nh.param<double>("/"+ns+"/color/r" , color_r , 0.0);
		ss << fixed_frame;
		fixed_frame = ss.str();
	}
	
	// Set markers attributes, such as color, size, etc.
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
		markers.color.r = color_r;
		markers.color.g = color_g;
		markers.color.b = color_b;
		markers.color.a = color_a;
	}

	// Add user skeleton points to markers object
	void add_to_markers(Skeleton &u_skel , visualization_msgs::Marker &marker)
	{
		marker.points.push_back(u_skel.left_hand);  	// points[0] = left_hand			
		marker.points.push_back(u_skel.left_elbow);	// points[1] = left_elbow
		marker.points.push_back(u_skel.left_shoulder);	// points[2] = left_shoulder
		marker.points.push_back(u_skel.neck);		// points[3] = neck
		marker.points.push_back(u_skel.head);		// points[4] = head
		marker.points.push_back(u_skel.right_shoulder);	// points[5] = right_shoulder	
		marker.points.push_back(u_skel.right_elbow);	// points[6] = right_elbow
		marker.points.push_back(u_skel.right_hand);	// points[7] = right_hand
	}
	
	// Attaching frame positions to user skeleton 
	void attach_user_skeleton(tf::TransformListener &tf_listener , int u_id)
	{
		tf::StampedTransform transform;
		try
				{	
					sstr.str("");
					sstr << "head_" << u_id;
					tf_listener.lookupTransform(fixed_frame , sstr.str() , ros::Time(0) , transform);
					user_skeleton.head.x = transform.getOrigin().x();
					user_skeleton.head.y = transform.getOrigin().y();
					user_skeleton.head.z = transform.getOrigin().z();
					
					sstr.str("");
					sstr << "neck_" << u_id;
					tf_listener.lookupTransform(fixed_frame , sstr.str() , ros::Time(0) , transform);
					user_skeleton.neck.x = transform.getOrigin().x();
					user_skeleton.neck.y = transform.getOrigin().y();
					user_skeleton.neck.z = transform.getOrigin().z();

					sstr.str("");
					//sstr << tf_prefix << "/left_hand_" << u_id;
					sstr << "left_hand_" << u_id;
					tf_listener.lookupTransform(fixed_frame , sstr.str() , ros::Time(0) , transform);
					user_skeleton.left_hand.x = transform.getOrigin().x();
					user_skeleton.left_hand.y = transform.getOrigin().y();
					user_skeleton.left_hand.z = transform.getOrigin().z();

					sstr.str("");
					sstr << "left_elbow_" << u_id;
					tf_listener.lookupTransform(fixed_frame , sstr.str() , ros::Time(0) , transform);
					user_skeleton.left_elbow.x = transform.getOrigin().x();
					user_skeleton.left_elbow.y = transform.getOrigin().y();
					user_skeleton.left_elbow.z = transform.getOrigin().z();

					sstr.str("");
					sstr << "left_shoulder_" << u_id;
					tf_listener.lookupTransform(fixed_frame , sstr.str() , ros::Time(0) , transform);
					user_skeleton.left_shoulder.x = transform.getOrigin().x();
					user_skeleton.left_shoulder.y = transform.getOrigin().y();
					user_skeleton.left_shoulder.z = transform.getOrigin().z();

					sstr.str("");
					sstr << "right_hand_" << u_id;
					tf_listener.lookupTransform(fixed_frame , sstr.str() , ros::Time(0) , transform);
					user_skeleton.right_hand.x = transform.getOrigin().x();
					user_skeleton.right_hand.y = transform.getOrigin().y();
					user_skeleton.right_hand.z = transform.getOrigin().z();
				
					sstr.str("");
					sstr << "right_elbow_" << u_id;
					tf_listener.lookupTransform(fixed_frame , sstr.str() , ros::Time(0) , transform);
					user_skeleton.right_elbow.x = transform.getOrigin().x();
					user_skeleton.right_elbow.y = transform.getOrigin().y();
					user_skeleton.right_elbow.z = transform.getOrigin().z();

					sstr.str("");
					sstr << "right_shoulder_" << u_id;
					tf_listener.lookupTransform(fixed_frame , sstr.str() , ros::Time(0) , transform);
					user_skeleton.right_shoulder.x = transform.getOrigin().x();
					user_skeleton.right_shoulder.y = transform.getOrigin().y();
					user_skeleton.right_shoulder.z = transform.getOrigin().z();
				}
				catch(tf::TransformException ex)
				{
					//ROS_INFO("user %d frames were not found", u_id);
					user_skeleton.head.x = -5.0;  // -5.0 is being used to flag an error while attaching user's skeleton frames
				}
	}

	//Declaring the main function of this node
	int main (int argc , char **argv)
	{
		// Declaring variables
		std::vector<std::string> all_frames , user_frames;
		
		ros::init(argc , argv , "users_markers");
		ROS_INFO("Initializing users_markers node...");
		tf::TransformListener tf_listener;
		ros::NodeHandle nh;
		
		// Subscribing to /users topic (published by 'openni_tracker_mod'  node)
		ros::Subscriber users_sub = nh.subscribe("users" , 1000 , usersCallback);
		
		// Reading external parameters
		readParams(nh);
		
		// Setting up the loop rate for this node
		ros::Rate loop_rate(rate);
	
		// Initializing publisher object
		ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("tracked_users_markers" , 1000);

		// Initializing marker objects
		initialize_markers(nh);

		//Waiting for transforms
		try
		{
			tf_listener.waitForTransform(fixed_frame.c_str() , fixed_frame.c_str() , ros::Time::now() , ros::Duration(10.0));
		}
		catch(tf::TransformException ex)
		{
			ROS_INFO("%s", ex.what());
		}
		
		//Main loop of this node
		while(ros::ok())
		{	
			// Getting every published frame
			tf_listener.getFrameStrings(all_frames);
			markers.header.stamp = ros::Time::now();
			
			// If '/tf' has no published frame yet
			if(all_frames.size() < 1)
			{
				loop_rate.sleep();
				continue;
			}
			
			// For every detected user (number of users is retrieved by subscribing to '/users' topic)
			for(int i=0 ; i < users_vector.size() ; i++)
			{
				// Create a Skeleton struct for every user
				attach_user_skeleton(tf_listener , users_vector[i]);

				// If didn't get any error while attaching user's skeleton , proceed:
				if(user_skeleton.head.x != -5.0) // -5.0 indicates that an error occurred
					add_to_markers(user_skeleton , markers);

				// Changing ID and TEXT components to correspond to the user
				markers.id = users_vector[i];
				sstr.str("");
				sstr << "user " << users_vector[i];
				markers.text = sstr.str();

				// Publishing markers
				marker_pub.publish(markers);
				markers.points.clear();

				// Inserting pushing this user id into published_ids (for this main loop iteration)
				published_ids.push_back(users_vector[i]);

				// If this user id is new (it needs to expand 'id_vector', so we know that this user has been detected at least once) 
				iter = std::find(id_vector.begin() , id_vector.end() , users_vector[i]);
				if(iter == id_vector.end())
					id_vector.push_back(users_vector[i]);
			}
			// For every user that has been detected at least once, verify whether it's markers have been published or not.
			// If not, publish a marker with cleared 'points' component. It means that user is no longer detected
			for(int j = 0; j < id_vector.size() ; j++)
			{
				iter = std::find(published_ids.begin() , published_ids.end() , id_vector[j]);
				if(iter == published_ids.end())
				{	
					markers.id = id_vector[j];
					markers.points.clear();
					sstr.str("");
					sstr << "user " << id_vector[j];
					markers.text = sstr.str();
					marker_pub.publish(markers);
				}
			}

			// Clearing published ids for the next loop iteration.
			published_ids.clear();
			
			loop_rate.sleep();
			ros::spinOnce();
		}
	}
