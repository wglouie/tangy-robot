#ifndef SONAR_PUBLISHER
#define SONAR_PUBLISHER
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <drrobot_h20_player/RangeArray.h>
#include <tf/transform_listener.h>


class Sonar_Publisher {
public:
Sonar_Publisher ();
void Subscribe();
void S_CB (const drrobot_h20_player::RangeArray& Given_Data);
ros::NodeHandle n;
private:
ros::Publisher S_Pub[5];
ros::Publisher ClearIR_Pub;
tf::TransformListener listener;
};

#endif //SONAR_PUBLISHER
