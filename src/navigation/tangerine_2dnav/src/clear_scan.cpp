#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "clear_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("clear_scan", 50);

  unsigned int num_readings = 100;
  double laser_frequency = 40;

  ros::Rate r(1.0);
  while(n.ok()){
    //generate some fake data for our laser scan
    ros::Time clear_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan clear;
    clear.header.stamp = clear_time;
    clear.header.frame_id = "map";
    clear.angle_min = -4.71/2;
    clear.angle_max = 4.71/2;
    clear.angle_increment = 4.71/ num_readings;
    clear.time_increment = (1 / laser_frequency) / (num_readings);
    clear.range_min = 0.0;
    clear.range_max = 100.0;

    clear.ranges.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      clear.ranges[i] = 10;
    }

    scan_pub.publish(clear);
    r.sleep();
  }
}
