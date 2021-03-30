#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
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
              std_dev_mult_(1.0f),
               to_frame_("")
          {
          };


        private:

            tf::TransformListener tf_;
            std::string to_frame_;
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
                private_nh.getParam("tf_to_frame_id", to_frame_);

               private_nh.getParam("mean_k", mean_k_);
               private_nh.getParam("std_dev_mult", std_dev_mult_);

        vox.setLeafSize (leaf_size_, leaf_size_, leaf_size_);


        sor.setMeanK (mean_k_);
        sor.setStddevMulThresh (std_dev_mult_);


                sub_ = nh.subscribe<PointCloud>("cloud_in", 10, &CloudPreprocess::callback, this);
                pub_ = nh.advertise<PointCloud>("cloud_out", 10);


            };

            void callback(const PointCloud::ConstPtr& cloud_in)
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
        sor.setInputCloud (cloud_in);
        sor.filter (*cloud_alt);
        if(cloud_alt->points.size() < 1000){
          NODELET_ERROR("StatisticalOutlierRemoval Filter failed, discarding results");
          cloud_alt = (*cloud_in).makeShared();
        }

        //Voxel Filter, downsample pointcloud
        vox.setInputCloud (cloud_alt);
        vox.filter (*cloud_out);
        if(cloud_out->points.size() < 1000){
          NODELET_ERROR("VoxelGrid Filter failed, discarding results");
          cloud_out = (*cloud_alt).makeShared();
        }

        //Transform point cloud into target frame_id
                if (to_frame_!="") {
                    NODELET_DEBUG("cloud received in frame %s", cloud_in->header.frame_id.c_str());
                    bool found_transform = tf_.waitForTransform(cloud_in->header.frame_id, to_frame_,cloud_in->header.stamp, ros::Duration(10.0));
                    if (found_transform)
                    {
                        pcl_ros::transformPointCloud(to_frame_, *cloud_out, *cloud_out,tf_);
                        NODELET_DEBUG("cloud transformed to frame in frame %s", cloud_out->header.frame_id.c_str());
                        pub_.publish (cloud_out);
                    }
                    else {
                        NODELET_WARN("No transform found between %s and %s", cloud_in->header.frame_id.c_str(),to_frame_.c_str());
                    }
                }
                else {
                    NODELET_WARN("No tf_to_frame");
                    pub_.publish(cloud_out);
                }

            }

            ros::Publisher pub_;
            ros::Subscriber sub_;

      pcl::VoxelGrid<PointType> vox;
      pcl::StatisticalOutlierRemoval<PointType> sor;

    };


    PLUGINLIB_DECLARE_CLASS(fake_laserscan, CloudPreprocess, laserscan::CloudPreprocess, nodelet::Nodelet);
}
