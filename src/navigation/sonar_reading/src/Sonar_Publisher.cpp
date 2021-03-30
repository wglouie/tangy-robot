#include <sonar_reading/Sonar_Publisher.h>
#include <ros/ros.h>
#include <string>
#include <sstream>
#include <drrobot_h20_player/RangeArray.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

Sonar_Publisher::Sonar_Publisher () {
  for(int i = 0; i < 5; i++) {
      std::string S_Number;
      std::ostringstream convert;
      convert << i;
      S_Number = convert.str();
      std::string name = "Sonar_Data" +  S_Number;
      S_Pub[i] = n.advertise<sensor_msgs::PointCloud2>(name, 1); //create publisher
     // ClearIR_Pub = n.advertise<sensor_msgs::PointCloud2>("clear_ir_data_pcl", 1); //create publisher
     // std::cout << name;
  }
//previous_is_true = false;
}

void Sonar_Publisher::S_CB (const drrobot_h20_player::RangeArray& Given_Data) {
  sensor_msgs::PointCloud S3_PC[4];
  sensor_msgs::PointCloud2 S3_PC2[4];

	S3_PC[0].header.frame_id="drrobot_sonar_0";
	S3_PC[1].header.frame_id="drrobot_sonar_1";
	S3_PC[2].header.frame_id="drrobot_sonar_2";
	S3_PC[3].header.frame_id="drrobot_sonar_3";


	for (int i = 0; i < 4; i++){
		S3_PC[i].channels.resize(1);
		S3_PC[i].points.resize(1);
		S3_PC[i].header.stamp=ros::Time::now();

		S3_PC[i].channels[0].name="Raw Data";
		S3_PC[i].channels[0].values.push_back(Given_Data.ranges[i].range);
		S3_PC[i].points[0].x = Given_Data.ranges[i].range;
		S3_PC[i].points[0].y = 0;
		S3_PC[i].points[0].z = 0;

		sensor_msgs::convertPointCloudToPointCloud2(S3_PC[i], S3_PC2[i]);

		S_Pub[i].publish(S3_PC2[i]);
	}
  //for(int i = 0; i < 5; i++){
   // S_Pub[i].publish(Given_Data.ranges[i]);
  //}
 
}



void Sonar_Publisher::Subscribe(){  
ros::Subscriber sub = n.subscribe("/drrobot_sonar", 100, &Sonar_Publisher::S_CB, this);
ros::spin();
}


