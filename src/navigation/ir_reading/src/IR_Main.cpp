#include <ros/ros.h>
#include <ir_reading/IR_Publisher.h>

int main (int argc, char** argv){

ros::init (argc, argv, "IR_Main");
IR_Publisher IR_Sensor;
IR_Sensor.Subscribe();
ros::NodeHandle node;
//ros::Subscriber sub = node.subscribe("/drrobot_ir", 100, &IR_Publisher::IR_CB, &IR_Sensor);
ros::spin();

return 0;
}
