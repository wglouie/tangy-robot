#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

namespace laserscan
{
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> PointCloud;

    class CloudPreprocess : public nodelet::Nodelet
    {
        public:
            //Constructor
            CloudPreprocess(): max_update_rate_(0)
          { 
          };

        private:

            ros::Time last_update_;
            double max_update_rate_;
            double leaf_size_;

            virtual void onInit()
            {
                ros::NodeHandle& nh = getNodeHandle();
                ros::NodeHandle& private_nh = getPrivateNodeHandle();

                private_nh.getParam("max_rate", max_update_rate_);

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
            	pub_.publish(cloud);
       	    }

            ros::Publisher pub_;
            ros::Subscriber sub_;

    };


    PLUGINLIB_DECLARE_CLASS(fake_laserscan, CloudPreprocess, laserscan::CloudPreprocess, nodelet::Nodelet);
}
