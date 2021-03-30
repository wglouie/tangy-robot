#include <ros/ros.h>
#include <drrobot_h20_player/HeadCmd.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>



/*
class NeckMovement{
public:
  
	ros::NodeHandle nh_;
	ros::Subscriber head_pos_sub_;
	ros::Subscriber marker_coordinates_sub_;
	
	ros::NodeHandle n_;
	ros::Publisher pub_line_marker;
	ros::Publisher pub_head;


	actionlib::SimpleActionClient<face_detection::identifyAction> FaceDetectionClient;
	
	
	int neck_x_rotation_;
	int neck_z_rotation_;



	
	int neck_pos_x;
	int neck_pos_z;


	drrobot_h20_player::HeadCmd cmdhead_;


	head_pos_sub_ = nh_.subscribe<drrobot_h20_player::HeadCmd>("head_pose", 1, boost::bind(&UserTrackingServer::neckPositionCB, this, _1));

	pub_head = n_.advertise<drrobot_h20_player::HeadCmd>("cmd_head", 1);


	



	
};
*/

int main (int argc, char **argv)
{

	ros::init(argc, argv, "neck_movement");


	ros::NodeHandle n_;
	ros::Publisher pub_head;

	//pub_head = n_.advertise<drrobot_h20_player::HeadCmd>("cmd_head", 1);
	pub_head = n_.advertise<drrobot_h20_player::HeadCmd>("cmd_pose_head", 1);
	//topic is cmd_head
	//pub_head is publisher

	drrobot_h20_player::HeadCmd cmdhead_;
	//cmdhead_ is a message 

	

	//needs to be between 3000 and 4500 - is for head up-down movement
	int neck_x_rotation_ = 3000;
	//needs to be between 1400 and 5500 - is for left-right movement	
	int neck_z_rotation_= 3450; //3450 is center - use 3000 and 3900 for side views
	//needs to be between 2950 and 3600 
	int mouth_= 3600;
	//needs to be between 2500 and 5200 
	int upper_head_ = 3500;
	//needs to be between 2000 and 4600 
	int left_eye_ = 3500;
	//needs to be between 2350 and 4950 
	int right_eye_ = 3500;
	//not sure what this is for, so I kept it at 0	
	int flag_ = 0;
	
	printf("\nstart\n");


	int i = 0;
	bool check = false;	


	ros::Rate loop_rate(2);

	
	//Send commands to HEAD servos
	
	
	cmdhead_.neck_x_rotation = neck_x_rotation_;  
	cmdhead_.neck_z_rotation = neck_z_rotation_; 
        cmdhead_.mouth = mouth_;
        cmdhead_.upper_head = upper_head_;
        cmdhead_.left_eye = left_eye_;
        cmdhead_.right_eye = right_eye_;
	cmdhead_.flag = flag_;
	
	


	while(!check)
	{
		printf("\npublish %d\n", i);

		pub_head.publish(cmdhead_);

		printf("\nsleep");
		loop_rate.sleep();
	


		ros::spinOnce();

	
		i++;

		if (i == 2)
		{
			check = true;

			//i = 3;
			//cmdhead_.neck_x_rotation *=-1;
			//cmdhead_.neck_z_rotation *=-1;

			
		}
/*
		if (i == 2)
		{
			cmdhead_.neck_x_rotation = 0; 
			cmdhead_.neck_z_rotation = -100;
		}



		if (check)
		{
			cmdhead_.neck_x_rotation = 0; //neck_x_rotation_;
			cmdhead_.neck_z_rotation = 0; //neck_z_rotation_;
			//neck_x_rotation_ = 0;
			//neck_z_rotation_= 0;

		}
*/

	}
	printf("\nfinished\n");

	return 0;


}
