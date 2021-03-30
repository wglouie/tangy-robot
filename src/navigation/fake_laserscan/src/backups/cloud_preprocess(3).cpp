#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace laserscan
{
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> PointCloud;

    class CloudPreprocess : public nodelet::Nodelet
    {
        public:
            //Constructor
            CloudPreprocess():
              max_update_rate_(0),
              leaf_size_(0.01f),
              mean_k_(30),
              std_dev_mult_(1.0f)
          { 
          };

        private:

            ros::Time last_update_;
            double max_update_rate_;
            double leaf_size_;

            int mean_k_;
            double std_dev_mult_;

            virtual void onInit()
            {
                ros::NodeHandle& nh = getNodeHandle();
                ros::NodeHandle& private_nh = getPrivateNodeHandle();

                private_nh.getParam("max_rate", max_update_rate_);
                private_nh.getParam("voxel_filter_size", leaf_size_);
                private_nh.getParam("mean_k", mean_k_);
                private_nh.getParam("std_dev_mult", std_dev_mult_);

        	vox.setLeafSize (leaf_size_, leaf_size_, leaf_size_);

        	sor.setMeanK (mean_k_);
        	sor.setStddevMulThresh (std_dev_mult_);

		pub_ = nh.advertise<PointCloud>("cloud_out", 10);
                sub_ = nh.subscribe<PointCloud>("cloud_in", 10, &CloudPreprocess::callback, this);

            };

            void callback(const PointCloud::ConstPtr& cloud)
            {
                //Throttle point cloud to reduce data rate
                if (max_update_rate_ > 0.0)
                {
                    NODELET_DEBUG("update set to %f", max_update_rate_);
                    if ( last_update_ + ros::Duration(1.0/max_update_rate_) > ros::Time::now())
                    {
                        NODELET_DEBUG("throttle last update at %f skipping", last_update_.toSec());
                        return;
                    }
                }else{
                    NODELET_DEBUG("update_rate unset continuing");
                }
                last_update_ = ros::Time::now();

		PointCloud::Ptr cloud_alt(new PointCloud);
              	PointCloud::Ptr cloud_out(new PointCloud);

        	//Filter Outliers to reduce false positives in pointcloud
        	sor.setInputCloud (cloud);
        	sor.filter (*cloud_alt);

        	if(cloud_alt->points.size() < 1000){
          	  NODELET_ERROR("StatisticalOutlierRemoval Filter failed, discarding results");
          	  cloud_alt = (*cloud).makeShared();
        	}

        	//Voxel Filter, downsample pointcloud
        	vox.setInputCloud (cloud_alt);
        	vox.filter (*cloud_out);

       		if(cloud_out->points.size() < 1000){
          	  NODELET_ERROR("VoxelGrid Filter failed, discarding results");
          	  cloud_out = (*cloud_alt).makeShared();
		}

            	pub_.publish(cloud_out);
       	    }

            ros::Publisher pub_;
            ros::Subscriber sub_;

	    pcl::VoxelGrid<PointType> vox;
      	    pcl::StatisticalOutlierRemoval<PointType> sor;

    };


    PLUGINLIB_DECLARE_CLASS(fake_laserscan, CloudPreprocess, laserscan::CloudPreprocess, nodelet::Nodelet);
}
