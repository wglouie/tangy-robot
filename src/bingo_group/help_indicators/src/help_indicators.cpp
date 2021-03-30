#include <help_indicators/help_indicators.h>

#define USE_IR
using namespace cv;

Help::Help(ros::NodeHandle n) :
    it_(n)
    , ir_image_sub_()
    , depth_image_sub_(){

	this->n = n;
    //head_ready = false;
	go = false;
	name = "Help_Indicator";
	count = 0;

	kinect_min_dist = 0;
	kinect_max_dist = 3;
	store_depth = false;
	n.getParam("help_indicators/kinect_min_dist", kinect_min_dist);
	n.getParam("help_indicators/kinect_max_dist", kinect_max_dist);
	n.getParam("help_indicators/store_depth", store_depth);

    //pub_head= n.advertise<drrobot_h20_player::HeadCmd>("/cmd_pose_head", 1);
    pub_filteredIR = n.advertise<sensor_msgs::Image>("/filtered_IR", 1);
    pub_transformer= n.advertise<help_indicators::triangle>("/transformer", 1);
    //pub_transformer1= n.advertise<help_indicators::transformRequest>("/transformer1", 1);
    //pub_transformer2= n.advertise<help_indicators::transformRequest>("/transformer2", 1);
    //pub_transformer3= n.advertise<help_indicators::transformRequest>("/transformer3", 1);
    //pub_transformer4= n.advertise<help_indicators::transformRequest>("/transformer4", 1);
     pub_clear_tf_indicator= n.advertise<std_msgs::String>("clear_transformed_help_indicator", 1);
    pub_detection_image = it_.advertise("help_indicators/detected_triangles", 1);
    //add_new_triangles_sub = n.subscribe("help_indicators/add_new_triangle", 1000, &Help::add_new_triangle_cb, this);
    rgb_image_sub = it_.subscribe("/bottom_camera/rgb/image_raw", 1, &Help::update_rgb_image, this);
    rgb_image_count = 0;
/*
		ir_image_sub_.subscribe(n, "/bottom_camera/ir/image", 1 );
		depth_image_sub_.subscribe(n, "/bottom_camera/depth/points", 1 );
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy( 100 ), ir_image_sub_, depth_image_sub_);
  sync->registerCallback( boost::bind( &Help::find_help_indicators, this, _1, _2 ) );
*/
	setup = false;

    //transformer1_in_use = false;
    //transformer2_in_use = false;
    //transformer3_in_use = false;
    //transformer4_in_use = false;

	ROS_INFO("Help Class Initiated.");
}

void Help::find_help_indicators(const sensor_msgs::ImageConstPtr& ir_msg, const sensor_msgs::PointCloud2ConstPtr& cloud) {
	if(!go) {
	    ROS_WARN("Help Indicators has not been started Publish to the help_indicators_go topic to start it.");
    	    return;
    	}
	else {
		if(store_depth == false) {
		    pcl::fromROSMsg(*cloud, cloud_filtered);
		}
		else if(!setup) {
			// Real World Distances
		    ROS_INFO("Storing Depth Image");
			pcl::fromROSMsg(*cloud, cloud_filtered);
		    setup = true;		
		}

			// Help Indicators
			#ifdef USE_IR
			std::vector<help_indicators::triangle> new_triangles = find_triangles(ir_msg);
            #else
			std::vector<help_indicators::triangle> new_triangles = find_color_triangles(ir_msg);
	   		#endif
            //ROS_INFO("New triangles size: [%d]", new_triangles.size());
			// Add triangles as a  new indicator (if its already included add as a time seen)
			update_new_help_indicators(new_triangles);
            //ROS_INFO("New help indicators size: [%d]", new_help_indicators.size());
			// Add indicators with 15 times seen and 40cm away from already added indicators
			update_help_indicators(cloud_filtered);

			draw_help_indicators();

	}
}

Help::~Help(){
    delete sync;
}

// clear help_indicator after they've been helped

void Help::clear_help_indicators_cb(const std_msgs::StringConstPtr& name) {

	// search through help_indicators until we find specified name
	for(int i = 0; i < help_indicators.size(); i++) {
		if(help_indicators.at(i).name == name->data) {
		    ROS_INFO("Deleting help_indicator: (%f,%f,%f)", help_indicators.at(i).x, help_indicators.at(i).y, help_indicators.at(i).z);
		    help_indicators.erase(help_indicators.begin() + i);
			break;
		}
	}
	pub_clear_tf_indicator.publish(name);
}

void Help::go_cb(const std_msgs::StringConstPtr& str) {
        if(str->data == "stop") {
                go = false;
        } else {
                go = true;
        }
}

void Help::update_rgb_image(const sensor_msgs::ImageConstPtr& rgb_msg){
    //Allow rgb camera to focus before stopping it
    if(rgb_image_count >= 50){
        ROS_INFO("Shutdown rgb_image_subscriber and initialize IR and Depth");
        rgb_image_sub.shutdown();
        ir_image_sub_.subscribe(n, "/bottom_camera/ir/image", 1 );
        depth_image_sub_.subscribe(n, "/bottom_camera/depth/points", 1 );
        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy( 100 ), ir_image_sub_, depth_image_sub_);
        sync->registerCallback( boost::bind( &Help::find_help_indicators, this, _1, _2 ) );
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    rgb_image = cv_ptr->image;
    //imshow("RGB IMAGE",rgb_image);
    //waitKey(30);
    rgb_image_count++;

}
// clear transformed_help_indicator after they've been helped
/*
void Help::clear_transformed_help_indicators_cb(const std_msgs::StringConstPtr& name) {

	// search through transformed_help_indicators until we find specified name
	for(int i = 0; i < transformed_help_indicators.size(); i++) {
		if(transformed_help_indicators.at(i).name == name->data) {
		ROS_INFO("Deleting transformed_help_indicator: (%f,%f,%f)", transformed_help_indicators.at(i).x, transformed_help_indicators.at(i).y, transformed_help_indicators.at(i).z);
		    transformed_help_indicators.erase(transformed_help_indicators.begin() + i);
			break;
		}
	}
}
*/

// if someone asks for the help_indicators give it to them and empty the list
/*
bool Help::get_help_indicators_cb(help_indicators::get_help_indicators::Request &req, help_indicators::get_help_indicators::Response &res) {	
        res.help_indicators = transformed_help_indicators;
        return true;
}
*/

// listens to help_indicators_go topic, and if stop is published it stops getting new_triangles from the kinect

// listens to head_ready topic, and if true head will acknowledge, if false it won't
/*
void Help::head_ready_cb(const std_msgs::BoolConstPtr& ready) {
        head_ready = ready->data;
}
*/
// transform callback
/*
void Help::transformer_cb(help_indicators::triangle new_help_indicator) {
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
}
*/
// transform callback
/*
void Help::transformerN_cb(help_indicators::transformRequest tfReq) {
	help_indicators::triangle new_help_indicator = tfReq.triangle;
	ROS_INFO("Before Transform: (%f,%f,%f)", new_help_indicator.x, new_help_indicator.y, new_help_indicator.z);
	transform_help_indicator(new_help_indicator);
	ROS_INFO("Adding: (%f,%f,%f)", new_help_indicator.x, new_help_indicator.y, new_help_indicator.z);
	transformed_help_indicators.push_back(new_help_indicator);

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
*/


void Help::draw_help_indicators(){
    cv::Mat result = rgb_image.clone();

    for(int i=0; i<help_indicators.size(); i++){
        cv::circle(result,cv::Point(help_indicators[i].image_x,help_indicators[i].image_y),5,cv::Scalar(0,255,0),10);
        ROS_INFO("X: %d         Y: %d            ",help_indicators[i].image_x, help_indicators[i].image_y);
    }

    //Mat circled_detection = circle_detected_triangle(new_triangle.x,new_triangle.y);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg();
    pub_detection_image.publish(msg);
    return;
}




			
