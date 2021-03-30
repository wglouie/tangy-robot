#include <ir_reading/IR_Publisher.h>
#include <ros/ros.h>
#include <string>
#include <sstream>
#include <drrobot_h20_player/RangeArray.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

IR_Publisher::IR_Publisher () {
  /*for(int i = 0; i < 10; i++) {
      std::string IR_Number;
      std::ostringstream convert;
      convert << i;
      IR_Number = convert.str();
      std::string name = "IR_Data" +  IR_Number;*/
      IR_Pub = n.advertise<sensor_msgs::PointCloud2>("ir_data_pcl", 1); //create publisher
      ClearIR_Pub = n.advertise<sensor_msgs::PointCloud2>("clear_ir_data_pcl", 1); //create publisher
     // std::cout << name;
  //}
      previous_is_set = false;
      previous_time = ros::Time::now();
}

void IR_Publisher::IR_CB (const drrobot_h20_player::RangeArray& Given_Data) {
  
  sensor_msgs::PointCloud IR_PointCloud;
  sensor_msgs::PointCloud2 IR_PointCloud2;
 
  sensor_msgs::PointCloud ClearIR_PointCloud;
  sensor_msgs::PointCloud2 ClearIR_PointCloud2;


//float MAX_RANGE=0.600000000000;
//Original Data bieng converted

  

  IR_PointCloud.channels.resize(1); //10
  IR_PointCloud.points.resize(10);
  IR_PointCloud.header.stamp=ros::Time::now();
  IR_PointCloud.header.frame_id="infrared_sensors";

  IR_PointCloud.channels[0].name="Raw Data";
  
  ClearIR_PointCloud.channels.resize(1); //10
  ClearIR_PointCloud.points.resize(100);
  ClearIR_PointCloud.header.stamp=ros::Time::now();
  ClearIR_PointCloud.header.frame_id="infrared_sensors";

  ClearIR_PointCloud.channels[0].name="Raw Data";

  int j = 0;

  for(int i=0; i < 10; i++){
          
     geometry_msgs::PointStamped IR_Input;
     geometry_msgs::PointStamped IR_Transformed;

     geometry_msgs::PointStamped ClearIR_Input;
     geometry_msgs::PointStamped ClearIR_Transformed;     

     IR_Input.point.y = 0;
     IR_Input.point.z = 0;
     
     ClearIR_Input.point.y = 0;
     ClearIR_Input.point.z = 0;

     IR_Input.header = Given_Data.ranges[i].header;
     IR_Input.header.stamp = ros::Time();

     if (Given_Data.ranges[i].range > 0.4){
	IR_Input.point.x = 10; 
	IR_PointCloud.channels[0].values.push_back(10);
     }
     else {
	IR_Input.point.x = Given_Data.ranges[i].range; 
	IR_PointCloud.channels[0].values.push_back(Given_Data.ranges[i].range);
     }
     
     ClearIR_Input.header = Given_Data.ranges[i].header;
     ClearIR_Input.header.stamp = ros::Time();
     ClearIR_Input.point.x = 0;

     std::string IR_Number;
     std::ostringstream convert;
     convert << i;
     IR_Number = convert.str();
     std::string name = "/drrobot_ir_" +  IR_Number;  

     listener.waitForTransform(name,"/base_link_robot", ros::Time(0), ros::Duration(3));

     listener.transformPoint("base_link_robot", IR_Input, IR_Transformed);
     IR_PointCloud.points[i].x=IR_Transformed.point.x;
     IR_PointCloud.points[i].y=IR_Transformed.point.y;
     IR_PointCloud.points[i].z=IR_Transformed.point.z;
		float k;
	 	for(int j = 0; j < 10; j++){
			k = 0.1*j;
			ClearIR_Input.point.x = k;
			ClearIR_PointCloud.channels[0].values.push_back(k);
			listener.transformPoint("base_link_robot", ClearIR_Input, ClearIR_Transformed);
			ClearIR_PointCloud.points[i*10 + j].x = ClearIR_Transformed.point.x;
			ClearIR_PointCloud.points[i*10 +j].y = ClearIR_Transformed.point.y;
			ClearIR_PointCloud.points[i*10 +j].z = ClearIR_Transformed.point.z;
		}
  }

  
   
  sensor_msgs::convertPointCloudToPointCloud2(IR_PointCloud, IR_PointCloud2);
  IR_Pub.publish(IR_PointCloud2);

  sensor_msgs::convertPointCloudToPointCloud2(ClearIR_PointCloud, ClearIR_PointCloud2);
  ClearIR_Pub.publish(ClearIR_PointCloud2);
 
/*  if(previous_is_set) {
		ClearIR_Pub.publish(Previous);//ClearIR_PointCloud2);
  }
 
  if((ros::Time::now()-previous_time)>ros::Duration(0.2)){
		Previous=ClearIR_PointCloud2;
		previous_is_set = true;
		previous_time=ros::Time::now();
  }*/
}



void IR_Publisher::Subscribe(){  
ros::Subscriber sub = n.subscribe("/drrobot_ir", 100, &IR_Publisher::IR_CB, this);
ros::spin();
}


