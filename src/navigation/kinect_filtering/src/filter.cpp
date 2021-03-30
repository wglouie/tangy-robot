#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;

void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& input)
{
  pcl::PCLPointCloud2::Ptr cloud_downsized (new pcl::PCLPointCloud2 ());
  
  //pcl_conversions::toPCL(*input,cloud_downsized)

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (input);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (*cloud_downsized);

  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  
  pcl::PassThrough<pcl::PCLPointCloud2> sort;
  sort.setInputCloud (cloud_downsized);
 // sort.input_frame("base_link");
  sort.setFilterFieldName ("z");
  sort.setFilterLimits (-0.50,0.10);
  sort.setFilterLimitsNegative (true);
  sort.filter(*cloud_filtered);

  // Publish the dataSize 
  pub.publish (*cloud_filtered);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud_throttled", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("kfiltering", 1);

  // Spin
  ros::spin ();
} 
