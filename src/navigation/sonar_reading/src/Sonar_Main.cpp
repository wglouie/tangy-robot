#include <ros/ros.h>
#include <sonar_reading/Sonar_Publisher.h>

int main (int argc, char** argv){

ros::init (argc, argv, "Sonar_Main");
Sonar_Publisher S_Sensor;
S_Sensor.Subscribe();
ros::NodeHandle node;
//ros::Subscriber sub = node.subscribe("/drrobot_ir", 100, &IR_Publisher::IR_CB, &IR_Sensor);
ros::spin();

return 0;
}
