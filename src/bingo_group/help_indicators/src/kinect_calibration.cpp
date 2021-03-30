#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>

#include <help_indicators/get_help_indicators.h>
#include <help_indicators/triangle.h>
#include <tangy_move.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>

using namespace std;
using namespace Eigen;

tf::TransformListener *tfListener;

// Parameters
std::string fixed_frame = "map";
std::string camera_frame = "camera_link";
pcl::PointCloud<pcl::PointXYZ> physical_points_;
pcl::PointCloud<pcl::PointXYZ> image_points_;


ros::ServiceClient client;
ros::Publisher go_pub;
ros::Publisher clear_pub;



tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
  tf::Matrix3x3 btm;
  btm.setValue(trans(0, 0), trans(0, 1), trans(0, 2),
               trans(1, 0), trans(1, 1), trans(1, 2),
               trans(2, 0), trans(2, 1), trans(2, 2));
  tf::Transform ret;
  ret.setOrigin(tf::Vector3(trans(0, 3), trans(1, 3), trans(2, 3)));
  ret.setBasis(btm);
  return ret;
}
Eigen::Matrix4f EigenFromTF(tf::Transform trans)
{
  Eigen::Matrix4f out;
  tf::Quaternion quat = trans.getRotation();
  tf::Vector3 origin = trans.getOrigin();

  Eigen::Quaternionf quat_out(quat.w(), quat.x(), quat.y(), quat.z());
  Eigen::Vector3f origin_out(origin.x(), origin.y(), origin.z());

  out.topLeftCorner<3, 3>() = quat_out.toRotationMatrix();
  out.topRightCorner<3, 1>() = origin_out;
  out(3, 3) = 1;

  return out;
}
void addPhysicalPoint() {
	geometry_msgs::PoseStamped poseTo;
	geometry_msgs::PoseStamped poseFrom;

	poseFrom.header.frame_id = "base_link";
	poseFrom.pose.position.x = 0;
	poseFrom.pose.position.y = 0;
	poseFrom.pose.position.z = 0;
	poseFrom.pose.orientation.w = 1;
	
	try {
		ros::Time current_time = ros::Time::now();
		poseFrom.header.stamp = current_time;
		poseTo.header.stamp = current_time;
		while( !(tfListener->waitForTransform(fixed_frame, poseFrom.header.frame_id, current_time, ros::Duration(1.0))) ){}
		tfListener->transformPose(fixed_frame, poseFrom, poseTo);
	} catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	ROS_INFO("The triangle is at: (%f, %f, %f)", poseTo.pose.position.x-0.48,poseTo.pose.position.y, 1.2);
	
	//ADDING 23 BECAUSE THE TRIANGLE WILL BE ON THE BACK OF THE ROBOT BUT THE POINT IS FROM BASELINK
	physical_points_.push_back(pcl::PointXYZ(poseTo.pose.position.x-0.48,poseTo.pose.position.y, 1.2));
	return;
}
bool addTrianglePoint()
{
	std_msgs::String stop_msg;
	stop_msg.data = "stop";
	
	std_msgs::String go_msg;
	go_msg.data = "go";

	// tell the kinect to start looking for triangles
	go_pub.publish(go_msg);
	
	// check if theres any help_indicators
	std::vector<help_indicators::triangle> help_indicators;
	
	while(help_indicators.size() < 1) {
		help_indicators::get_help_indicators srv;
		srv.request.str = "Anything";
		if(client.call(srv)) {
			help_indicators = srv.response.help_indicators;
		} else {
			ROS_ERROR("Failed to call service get_help_indicators");
			return 1;
		}
	}
	// if you ever need to stop
	go_pub.publish(stop_msg);
	sleep(3);	
	go_pub.publish(stop_msg);
	while(help_indicators.empty() == false) {
		ROS_INFO("There is a help_indicator at: (%f, %f, %f)", help_indicators.at(0).x, help_indicators.at(0).y, help_indicators.at(0).z);
		image_points_.push_back(pcl::PointXYZ(help_indicators.at(0).x, help_indicators.at(0).y, help_indicators.at(0).z));	
		// once you check the indicators clear them
		std_msgs::String clear_msg;
		clear_msg.data = help_indicators[0].name;
		clear_pub.publish(clear_msg);
		
		help_indicators::get_help_indicators srv;
		srv.request.str = "Anything";
		if(client.call(srv)) {
			help_indicators = srv.response.help_indicators;
		} else {
			ROS_ERROR("Failed to call service get_help_indicators");
		}
	}
	return 0;
}
void printStaticTransform(Eigen::Matrix4f& transform, const std::string frame1, const std::string frame2)
{
    Eigen::Quaternionf quat(transform.topLeftCorner<3, 3>());
    Eigen::Vector3f translation(transform.topRightCorner<3, 1>());

    cout << "Static transform publisher (use for external kinect): " << endl;
    cout << "x y z qx qy qz qw frame_id child_frame_id period_in_ms" << endl;
    cout << translation.x() << " " << translation.y() << " "
         << translation.z() << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
         << " " << "map" << " " << "remote_kinect" << " 100" << endl << endl;
	Eigen::Matrix4f EigenFromTF(tf::Transform trans);

	return;
}
int main(int argc, char** argv) {
	ros::init( argc, argv, "help_indicator_client" );
	ros::NodeHandle n;
	
	// initialize move_client class
	navigationClient navClient;
	navClient.initialize(n);
	cout << "Here" << endl;

	tfListener = new tf::TransformListener;
	
	client = n.serviceClient<help_indicators::get_help_indicators>("get_help_indicators");
	go_pub = n.advertise<std_msgs::String>("help_indicators_go", 1000);
	clear_pub = n.advertise<std_msgs::String>("clear_help_indicator", 1000);
	sleep(1);

	// pick 4 points on the square defined by (x0,y0), (x1,y0), (x1,y1), (x0,y1)
	double x[4];
	double y[4];
	x[0] = 0.6;
	y[0] = 0.0;
	x[1] = 1.6;
	y[1] = 0.6;
	x[2] = 1.6;
	y[2] = 0.0;
	x[3] = 1.6;
	y[3] = -1;
	for(int i = 0; i < 4; i++){
		if(i>1) {
			navClient.move_straight(-0.6);
		}
		physical_points_.header.frame_id = fixed_frame;
		
		// move robot to location for current itteration
		// use move_client object
		cout << "Moving to Position" << endl;

		navClient.move("map", x[i], y[i]);		
		navClient.rotate("map", 0);


		cout << "Preparing to grab points" << endl;

		// wait for robot to localize itself
		sleep(3);

		// get actual point the robot is currently at
		addPhysicalPoint();

		// add to list of points kinect is going to use for localizing itself
		addTrianglePoint();
		cout << "Finished grabbing points" << endl;
	}
	// use points stored for localization to determine kinect transformation	
	Eigen::Matrix4f t;
	pcl::registration::TransformationEstimationSVD < pcl::PointXYZ, pcl::PointXYZ > svd_estimator;
	svd_estimator.estimateRigidTransformation(image_points_,physical_points_, t);
	printStaticTransform(t, fixed_frame, camera_frame);

    return 0;
  }

