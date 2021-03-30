#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/ros/conversions.h> //Groovy
#include <pcl_conversions/pcl_conversions.h> //Hydro

namespace laserscan
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    class CloudToScan : public nodelet::Nodelet
    {
        public:
            CloudToScan() { };

            ~CloudToScan() { }

        private:

            virtual void onInit()
            {
                ros::NodeHandle nh = getNodeHandle();
                ros::NodeHandle& private_nh = getPrivateNodeHandle();


		private_nh.param<double>("min_height", min_height_, 0.10);
		private_nh.param<double>("max_height", max_height_, 0.15);
		private_nh.param<double>("angle_min", angle_min_, -M_PI/2);
		private_nh.param<double>("angle_max", angle_max_, M_PI/2);
		private_nh.param<double>("angle_increment", angle_increment_, M_PI/180.0/2.0);
		private_nh.param<double>("scan_time", scan_time_, 1.0/30.0);
		private_nh.param<double>("range_min", range_min_, 0.45);
		private_nh.param<double>("range_max", range_max_, 10.0);

		pub_ = nh.advertise<sensor_msgs::LaserScan>("fake_laserscan", 10);
   	        sub_ = nh.subscribe<PointCloud>("cloud", 10, &CloudToScan::callback, this);//first argument was "cloud"
            };

            void callback(const PointCloud::ConstPtr& cloud)
            {
		/*
                sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
		NODELET_DEBUG("Got cloud");
		//Copy Header		
		//output->header = cloud->header; //Groovy
                output->header = pcl_conversions::fromPCL(cloud->header); //Hydro
                output->header.frame_id = cloud->header.frame_id.c_str();
                output->angle_min = -M_PI/2;
                output->angle_max = M_PI/2;
                output->angle_increment = M_PI/180.0/2.0;
                output->time_increment = 0.0;
                output->scan_time = 1.0/30.0;
                output->range_min = 0.45;
                output->range_max = 5.0;
		*/

		sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
                NODELET_DEBUG("Got cloud");
		std_msgs::Header cloud_in_header = pcl_conversions::fromPCL(cloud->header);
		output->header = cloud_in_header;
		output->header.frame_id = cloud_in_header.frame_id.c_str();
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
				//NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
				continue;
			}

			if (y > max_height_ || y < min_height_)
			{
				//NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", z, min_height_, max_height_);
				continue;
			}

			//if (z > max_height_ || z < min_height_)
			//{
			//	NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", z, min_height_, max_height_);
			//	continue;
			//}

			//double range = sqrt(x*x+y*y);
			//if (range < range_min_) {
				//NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, x, y, z);
			//	continue;
			//}


			//double angle = atan2(y, x);
			//if (angle < output->angle_min || angle > output->angle_max)
			//{
				//NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
			//	continue;
			//}
			//int index = (angle - output->angle_min) / output->angle_increment;

			double angle = -atan2(x, z);
			if (angle < output->angle_min || angle > output->angle_max)
			{
				NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
				continue;
			}
			int index = (angle - output->angle_min) / output->angle_increment;

			double range_sq = z*z+x*x;
			if (output->ranges[index] * output->ranges[index] > range_sq){
				output->ranges[index] = sqrt(range_sq);
			}
                }

                pub_.publish(output);
            }


            double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;

            ros::Publisher pub_;
            ros::Subscriber sub_;

    };

    PLUGINLIB_DECLARE_CLASS(fake_laserscan, CloudToScan, laserscan::CloudToScan, nodelet::Nodelet);
}
