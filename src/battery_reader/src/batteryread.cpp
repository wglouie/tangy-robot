
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <drrobot_h20_player/PowerInfo.h>
#include <battery_reader/batterylevel.h>
#include <battery_reader/powerstatus.h>
ros::Publisher status_pub, battery_pub;
void chatterCallback(const drrobot_h20_player::PowerInfo::ConstPtr& powerInfo)
{

int pstatus;
pstatus = powerInfo->power_status;
float bat1_vol, bat2_vol;
bat1_vol = powerInfo->bat1_vol;
bat2_vol = powerInfo->bat2_vol;

float batavg_vol = (bat1_vol + bat2_vol)/2;

//  ROS_INFO("Power Status: [%d]", pstatus);
//  ROS_INFO("BatteryAVG: [%f]", batavg_vol);
  //ROS_INFO("Battery_1: [%f]", bat1_vol);
  //ROS_INFO("Battery_2: [%f]", bat2_vol);

// if (pstatus == 16){
//std::cout << "Charging";
//}
//else {
//std::cout << "On Battery";
//}

//create the message
	battery_reader::batterylevel current;
	battery_reader::powerstatus status;
	current.battery_level = batavg_vol;
	status.power = pstatus;
  battery_pub.publish(current);
  status_pub.publish(status);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "batread");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("drrobot_powerinfo", 1000, chatterCallback);
  battery_pub = n.advertise<battery_reader::batterylevel>("batterylevel", 1000);
  status_pub = n.advertise<battery_reader::powerstatus>("powerstatus", 1000);
  ros::spin();

  return 0;
}

