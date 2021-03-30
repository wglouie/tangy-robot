#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <boost/thread/mutex.hpp>


namespace laserscan
{
    class CloudToScan : public nodelet::Nodelet
    {
        public:
            CloudToScan(): min_height_(0.10),
              max_height_(0.15),
              angle_min_(-M_PI/2),
              angle_max_(M_PI/2),
              angle_increment_(M_PI/180.0/2.0),
              scan_time_(1.0/30.0),
              range_min_(0.45),
              range_max_(10.0)
          { };

            ~CloudToScan()
            { }

        private:
            typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

            boost::mutex connect_mutex_;


            virtual void onInit()
            {
                nh_ = getNodeHandle();
                ros::NodeHandle& private_nh = getPrivateNodeHandle();

                private_nh.getParam("min_height", min_height_);
                private_nh.getParam("max_height", max_height_);

                private_nh.getParam("angle_min", angle_min_);
                private_nh.getParam("angle_max", angle_max_);
                private_nh.getParam("angle_increment", angle_increment_);
                private_nh.getParam("scan_time", scan_time_);
                private_nh.getParam("range_min", range_min_);
                private_nh.getParam("range_max", range_max_);

                //Efficient subscription, only runs if there are subscribers to scan
                ros::AdvertiseOptions scan_ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
                        "scan", 10,
                        boost::bind( &CloudToScan::connectCB, this),
                        boost::bind( &CloudToScan::disconnectCB, this), ros::VoidPtr(), nh_.getCallbackQueue());

                boost::lock_guard<boost::mutex> lock(connect_mutex_);
                pub_ = nh_.advertise(scan_ao);
            };

            void connectCB() {
                boost::lock_guard<boost::mutex> lock(connect_mutex_);
                if (pub_.getNumSubscribers() > 0) {
                    NODELET_DEBUG("Connecting to point cloud topic.");
                    sub_ = nh_.subscribe<PointCloud>("cloud_in", 10, &CloudToScan::callback, this);
                }
            }

            void disconnectCB() {
                boost::lock_guard<boost::mutex> lock(connect_mutex_);
                if (pub_.getNumSubscribers() == 0) {
                    NODELET_DEBUG("Unsubscribing from point cloud topic.");
                    sub_.shutdown();
                }
            }


            void callback(const PointCloud::ConstPtr& cloud)
            {
                sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
                output->header = cloud->header;
                output->header.frame_id = cloud->header.frame_id.c_str();
                output->angle_min = angle_min_;
                output->angle_max = angle_max_;
                output->angle_increment = angle_increment_;
                output->time_increment = 0.0;
                output->scan_time = scan_time_;
                output->range_min = range_min_;
                output->range_max = range_max_;

                uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
                output->ranges.assign(ranges_size, output->range_max + 1.0);

                for (PointCloud::const_iterator it = cloud->begin(); it != cloud->end(); ++it)
                {
                    const float &x = it->x;
                    const float &y = it->y;
                    const float &z = it->z;

                    if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
                    {
                        NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
                        continue;
                    }

                    if (z > max_height_ || z < min_height_)
                    {
                        NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", z, min_height_, max_height_);
                        continue;
                    }

                    double range = sqrt(x*x+y*y);
                    if (range < range_min_) {
                        NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, x, y, z);
                        continue;
                    }

                    double angle = atan2(y, x);
                    if (angle < output->angle_min || angle > output->angle_max)
                    {
                        NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
                        continue;
                    }
                    int index = (angle - output->angle_min) / output->angle_increment;


                    if (range < output->ranges[index]){
                        output->ranges[index] = range;
                    }
                }

                pub_.publish(output);
            }


            double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;

            ros::NodeHandle nh_;
            ros::Publisher pub_;
            ros::Subscriber sub_;

    };

    PLUGINLIB_DECLARE_CLASS(fake_laserscan, CloudToScan, laserscan::CloudToScan, nodelet::Nodelet);
}
