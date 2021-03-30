#ifndef IR_PUBLISHER
#define IR_PUBLISHER
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <drrobot_h20_player/RangeArray.h>
#include <tf/transform_listener.h>


class IR_Publisher {
	public:
		IR_Publisher ();
		void Subscribe();
		void IR_CB (const drrobot_h20_player::RangeArray& Given_Data);
		ros::NodeHandle n;
	private:
		ros::Publisher IR_Pub;
		ros::Publisher ClearIR_Pub;
		tf::TransformListener listener;
		sensor_msgs::PointCloud2 Previous;
		bool previous_is_set;
		ros::Time previous_time;
};

#endif //IR_PUBLISHER
