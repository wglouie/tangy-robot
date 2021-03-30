#ifndef HELP_INDICATORS_H
#define HELP_INDICATORS_H

//#define DEBUG_HELP_INDICATORS

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/photo/photo.hpp"
#include <sstream>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <iostream>
#include <help_indicators/squareFinder.h>

#include <ros/ros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/video/tracking.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <help_indicators/get_help_indicators.h>
#include <help_indicators/triangle.h>
#include <help_indicators/transformRequest.h>
#include <vector>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <drrobot_h20_player/HeadCmd.h>

class Help {

public:

	Help(ros::NodeHandle nh_);
    ~Help();
    //geometry_msgs::PoseStamped transformPose(std::string frameTo, geometry_msgs::PoseStamped poseFrom);
    void find_help_indicators(const sensor_msgs::ImageConstPtr& ir_msg, const sensor_msgs::PointCloud2ConstPtr& cloud);
	void update_help_indicators(pcl::PointCloud<pcl::PointXYZ> cloud_filtered);
	void update_new_help_indicators(std::vector<help_indicators::triangle> new_triangles);
	void remove_duplicates(std::vector<help_indicators::triangle> &new_triangles);
	double get_distance(double x1, double y1, double x2, double y2);
	std::vector<help_indicators::triangle> find_triangles(const sensor_msgs::ImageConstPtr& ir_msg);
	vector<help_indicators::triangle> find_color_triangles(const sensor_msgs::ImageConstPtr& ir_msg);
    void update_rgb_image(const sensor_msgs::ImageConstPtr& rgb_msg);
	void go_cb(const std_msgs::StringConstPtr& str);
    //void head_ready_cb(const std_msgs::BoolConstPtr& ready);
    //bool  get_help_indicators_cb(help_indicators::get_help_indicators::Request  &req, help_indicators::get_help_indicators::Response &res);
	void clear_help_indicators_cb(const std_msgs::StringConstPtr& str);
    //void clear_transformed_help_indicators_cb(const std_msgs::StringConstPtr& str);
    //void transform_help_indicator(help_indicators::triangle &help_indicator);
	int times_seen(help_indicators::triangle new_help_indicator);
    //void acknowledge_help(float x, float y);
    //void transformer_cb(help_indicators::triangle new_help_indicator);
    //void transformerN_cb(help_indicators::transformRequest tfReq);
    //void add_new_triangle_cb(const help_indicators::triangle::ConstPtr& msg);
    //void rgb_image_cb(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat overlay_detection_image(cv::Mat &triangle_image);
    void draw_help_indicators();

private:
    //bool transformer1_in_use;
    //bool transformer2_in_use;
    //bool transformer3_in_use;
    //bool transformer4_in_use;

	double kinect_min_dist;
	double kinect_max_dist;
	bool store_depth;
    //bool head_ready;

    //ros::Publisher pub_head;
    ros::Publisher pub_filteredIR;
    ros::Publisher pub_transformer;
    //ros::Publisher pub_transformer1;
    //ros::Publisher pub_transformer2;
    //ros::Publisher pub_transformer3;
    //ros::Publisher pub_transformer4;
	ros::Publisher pub_clear_tf_indicator;
    image_transport::Publisher pub_detection_image;
    //ros::Subscriber add_new_triangles_sub;

    image_transport::Subscriber rgb_image_sub;

	pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
	bool setup;
	bool go;
        std::vector<help_indicators::triangle> new_help_indicators;
	std::vector<help_indicators::triangle> help_indicators;
    //std::vector<help_indicators::triangle> transformed_help_indicators;
    //tf::TransformListener tfListener;

	ros::NodeHandle n;
	image_transport::ImageTransport it_;
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef image_transport::SubscriberFilter ImageSubscriber;

	typedef message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
	typedef message_filters::Subscriber<sensor_msgs::Image> image_sub;

	image_sub ir_image_sub_;
	cloud_sub depth_image_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy; //Changed

    message_filters::Synchronizer<MySyncPolicy> *sync;

    cv::Mat rgb_image;
    int rgb_image_count;
    //std::string action_name_;
    //bool detectTriangleGoal;
	std::string name;	
	int count;
};

#endif //HELP_INDICATORS_H
