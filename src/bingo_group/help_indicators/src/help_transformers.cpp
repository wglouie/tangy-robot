#include <help_indicators/help_indicators.h>
#include <visualization_msgs/Marker.h>

bool head_ready;
std::vector<help_indicators::triangle> transformed_help_indicators;
bool transformer1_in_use;
bool transformer2_in_use;
bool transformer3_in_use;
bool transformer4_in_use;
int count = 0;
ros::Publisher pub_head;
ros::Publisher pub_transformer;
ros::Publisher pub_transformer1;
ros::Publisher pub_transformer2;
ros::Publisher pub_transformer3;
ros::Publisher pub_transformer4;
ros::Publisher notify_new_tri_pub;
tf::TransformListener *tfListener; 

ros::Publisher marker_pub;
visualization_msgs::Marker marker;

// transform a pose from one frame to another (used for transforming from kinect pose to base_link pose)
geometry_msgs::PoseStamped transformPose(std::string frameTo, geometry_msgs::PoseStamped poseFrom) {
	geometry_msgs::PoseStamped poseTo;
	try {
		ros::Time current_time = ros::Time::now();
		poseFrom.header.stamp = current_time;
		poseTo.header.stamp = current_time;
		while( !(tfListener->waitForTransform(frameTo, poseFrom.header.frame_id, current_time, ros::Duration(1.0))) ){}
		tfListener->transformPose(frameTo, poseFrom, poseTo);
	} catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	return poseTo;
}


// convert new_help_indicators from remote_kinect frame to base_link frame before adding it to the help_indicators list
void transform_help_indicator(help_indicators::triangle &help_indicator) {
	geometry_msgs::PoseStamped kinect_frame_XYZ;
	geometry_msgs::PoseStamped base_link_frame_XYZ;
	kinect_frame_XYZ.header.frame_id = "remote_kinect";
	kinect_frame_XYZ.pose.position.x = help_indicator.x;
	kinect_frame_XYZ.pose.position.y = help_indicator.y;
	kinect_frame_XYZ.pose.position.z = help_indicator.z;
	kinect_frame_XYZ.pose.orientation.x = 0;
	kinect_frame_XYZ.pose.orientation.y = 0;
	kinect_frame_XYZ.pose.orientation.z = 0;
	kinect_frame_XYZ.pose.orientation.w = 1;
	kinect_frame_XYZ.header.stamp = ros::Time::now();
	base_link_frame_XYZ = transformPose("map", kinect_frame_XYZ);

	help_indicator.x = base_link_frame_XYZ.pose.position.x;
	help_indicator.y = base_link_frame_XYZ.pose.position.y;
	help_indicator.z = base_link_frame_XYZ.pose.position.z;
}


// if someone asks for the help_indicators give it to them and empty the list
bool get_help_indicators_cb(help_indicators::get_help_indicators::Request &req, help_indicators::get_help_indicators::Response &res) {	
        res.help_indicators = transformed_help_indicators;
        return true;
}

// clear transformed_help_indicator after they've been helped
void clear_transformed_help_indicators_cb(const std_msgs::StringConstPtr& name) {

	// search through transformed_help_indicators until we find specified name
	for(int i = 0; i < transformed_help_indicators.size(); i++) {
		if(transformed_help_indicators.at(i).name == name->data) {
		ROS_INFO("Deleting transformed_help_indicator: (%f,%f,%f)", transformed_help_indicators.at(i).x, transformed_help_indicators.at(i).y, transformed_help_indicators.at(i).z);
		    transformed_help_indicators.erase(transformed_help_indicators.begin() + i);
			break;
		}
	}
    std_msgs::String notify_new_tri_msg;
    notify_new_tri_msg.data = "new triangle deleted";
    notify_new_tri_pub.publish(notify_new_tri_msg);
}

void acknowledge_help(float x, float y) {
    const float PI = 3.14159265359;
  
    drrobot_h20_player::HeadCmd cmdhead_;

    int neck_x_rotation_ = 3800;
    int neck_z_rotation_= 3450;
    int mouth_= 3500;
    int upper_head_ = 3500;
    int left_eye_ = 3600;
    int right_eye_ = 3350;
    cmdhead_.neck_x_rotation = neck_x_rotation_; //Head Tilt
    cmdhead_.neck_z_rotation = neck_z_rotation_; //Head Pan
    cmdhead_.mouth = mouth_;
    cmdhead_.upper_head = upper_head_;
    cmdhead_.left_eye = left_eye_;
    cmdhead_.right_eye = right_eye_;
    cmdhead_.flag = 1000;

    double ang = atan ( y/x );
    //Assuming that the head pan servo has max/rest/min servo values (4550/3450/2350) corresponding to (+90deg/0deg/-90deg)
    //Can get that 1 deg ~= 12.222 servo increments; use 18 to exaggerate difference
    for(int i =0; i<10; i++){
        cmdhead_.neck_z_rotation= int((-ang*180/PI)/10*i*13)+3450;
        pub_head.publish(cmdhead_);
        usleep(150000);
    }
    pub_head.publish(cmdhead_);
    usleep(500000);
    //Nod action
    //Assuming that the head tilt servo has rest/min servo values (3800/3050) corresponding to (0deg/-45deg)
    //Can get that 1 deg ~= 23.333 servo increments
    for(int i=0;i<15;i++) {
      cmdhead_.neck_x_rotation= 3800 - int(2*i*23.333);
      
      pub_head.publish(cmdhead_);
      usleep(60000);
    }
    pub_head.publish(cmdhead_);
    for(int i=0;i<15;i++) {
      cmdhead_.neck_x_rotation= 3100 + int(2*i*23.333);
      
      pub_head.publish(cmdhead_);
      usleep(60000);
    }
    pub_head.publish(cmdhead_);
    cmdhead_.neck_x_rotation=3800;
    usleep(500000);
    double curr_ang=cmdhead_.neck_z_rotation;
    for(int i =0; i<10; i++) {
        cmdhead_.neck_z_rotation= curr_ang-int((ang*180/PI)/10*i*12.222);
        
        pub_head.publish(cmdhead_);
        usleep(150000);
    }
    pub_head.publish(cmdhead_);
  
}

// listens to head_ready topic, and if true head will acknowledge, if false it won't
void head_ready_cb(const std_msgs::BoolConstPtr& ready) {
        head_ready = ready->data;
}	

// transform callback
void transformer_cb(help_indicators::triangle new_help_indicator) {
	help_indicators::transformRequest tfReq;
	tfReq.triangle = new_help_indicator;
	bool done = false;
	while(!done) {
		done = true;
		if(!transformer1_in_use) {
			transformer1_in_use = true;
			tfReq.transformer = 1;
			ROS_INFO("Publishing to transformer1");
			pub_transformer1.publish(tfReq);
		} else if(!transformer2_in_use) {
			transformer2_in_use = true;
			tfReq.transformer = 2;
			ROS_INFO("Publishing to transformer2");
			pub_transformer2.publish(tfReq);
		} else if(!transformer3_in_use) {
			transformer3_in_use = true;
			tfReq.transformer = 3;
			ROS_INFO("Publishing to transformer3");
			pub_transformer3.publish(tfReq);
		} else if(!transformer4_in_use) {
			transformer4_in_use = true;
			tfReq.transformer = 4;
			ROS_INFO("Publishing to transformer4");
			pub_transformer4.publish(tfReq);
		} else {
			ROS_ERROR("No transformers available for transformation!");
			done = false;
		} 
	}
    std_msgs::String notify_new_tri_msg;
    notify_new_tri_msg.data = "new triangle detected";
    notify_new_tri_pub.publish(notify_new_tri_msg);
}


// transform callback
void transformerN_cb(help_indicators::transformRequest tfReq) {
	help_indicators::triangle new_help_indicator = tfReq.triangle;

	ROS_INFO("Before Transform: (%f,%f,%f)", new_help_indicator.x, new_help_indicator.y, new_help_indicator.z);
	transform_help_indicator(new_help_indicator);
	ROS_INFO("Adding: (%f,%f,%f)", new_help_indicator.x, new_help_indicator.y, new_help_indicator.z);
	transformed_help_indicators.push_back(new_help_indicator);

    //For visualization in RVIZ
    marker.header.frame_id = "/map";
    marker.pose.position.x = new_help_indicator.x;
    marker.pose.position.y = new_help_indicator.y;
    marker.pose.position.z = new_help_indicator.z;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.header.stamp = ros::Time::now();
    marker_pub.publish(marker);

	// acknowledge by nodding head
	if(head_ready) {
		acknowledge_help(new_help_indicator.x, new_help_indicator.y);
	}

	// UNLOCK
        if(tfReq.transformer == 1) {
		transformer1_in_use = false;
	} else if(tfReq.transformer == 2) {
		transformer2_in_use = false;
	} else if(tfReq.transformer == 3) {
		transformer3_in_use = false;
	} else if(tfReq.transformer == 4) {
		transformer4_in_use = false;
	} else {
		ROS_ERROR("%d does not exist", tfReq.transformer);
	} 

}

void add_new_triangle_cb(const help_indicators::triangle::ConstPtr& msg){
    ROS_INFO("Adding triangle that has been manually generated");
    help_indicators::triangle triangle;
    count++;
    std::string name = "Saved_Help_Indicator";
    triangle.name = name + boost::lexical_cast<std::string>(count);
    triangle.seen = msg->seen;
    triangle.x = msg->x;
    triangle.y = msg->y;
    triangle.z = msg->z;
    triangle.added = msg->added;
    transformed_help_indicators.push_back(triangle);

    std_msgs::String notify_new_tri_msg;
    notify_new_tri_msg.data = "new triangle detected";
    notify_new_tri_pub.publish(notify_new_tri_msg);

    ROS_INFO("There are now [%d] triangles", transformed_help_indicators.size());
}

int main(int argc, char** argv) {
	ros::init( argc, argv, "help_transformers" );
	ros::NodeHandle n;

	tfListener = new tf::TransformListener; 
	pub_head= n.advertise<drrobot_h20_player::HeadCmd>("/cmd_pose_head", 1);
	pub_transformer= n.advertise<help_indicators::triangle>("/transformer", 1);
	pub_transformer1= n.advertise<help_indicators::transformRequest>("/transformer1", 1);
	pub_transformer2= n.advertise<help_indicators::transformRequest>("/transformer2", 1);
	pub_transformer3= n.advertise<help_indicators::transformRequest>("/transformer3", 1);
	pub_transformer4= n.advertise<help_indicators::transformRequest>("/transformer4", 1);
    notify_new_tri_pub = n.advertise<std_msgs::String>("/help_indicators/notify_new_triangle", 1);

    //set up RVIZ marker stuff
    marker_pub = n.advertise<visualization_msgs::Marker>("help_pos_markers", 1000);
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "test_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.6;
    marker.color.r = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(20.0);

	transformer1_in_use = false;
	transformer2_in_use = false;
	transformer3_in_use = false;
	transformer4_in_use = false;

	ros::ServiceServer service = n.advertiseService("get_help_indicators", get_help_indicators_cb);
	ros::Subscriber clear_sub = n.subscribe("clear_transformed_help_indicator", 1, clear_transformed_help_indicators_cb);
	ros::Subscriber head_ready_sub = n.subscribe("head_ready", 1,head_ready_cb);
    ros::Subscriber add_new_triangles_sub = n.subscribe("help_indicators/add_new_triangle", 1000, add_new_triangle_cb);

	ros::Subscriber transformer_sub = n.subscribe("transformer", 1, transformer_cb);
	ros::Subscriber transformer1_sub = n.subscribe("transformer1", 1, transformerN_cb);
	ros::Subscriber transformer2_sub = n.subscribe("transformer2", 1, transformerN_cb);
	ros::Subscriber transformer3_sub = n.subscribe("transformer3", 1, transformerN_cb);
	ros::Subscriber transformer4_sub = n.subscribe("transformer4", 1, transformerN_cb);

	// One thread for service, four threads for the four buttons transformations and one thread to spin them all
	ros::MultiThreadedSpinner spinner(6);
	spinner.spin();

	delete tfListener;
	return 0;
}
