#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <drrobot_h20_player/MotorInfoArray.h>


ros::Time current_time, last_time;

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.1;
double vy = -0.1;
double vth = 0.1;

/*! \brief
 *      This call back function is for MotorInfoArray message.
 *  @param[in] msg  is a pointer of MotorInfoArray message, the array size should be 6
 *  @return null
 */
void motorSensorCallback(const drrobot_h20_player::MotorInfoArray::ConstPtr& msg)
{
  int msgSize = msg->motorInfos.capacity();

  if (msgSize == 6)
  {
	current_time = ros::Time::now();

	/*
	double ctime = current_time.toSec();
	double ltime = last_time.toSec();
	ROS_INFO("\n \n >>>>>> HERE WE GO: \n");
	ROS_INFO("\n \n >>>>>> Second now: [%d]\n", ctime);
	ROS_INFO("\n \n >>>>>> Second last: [%d]\n", ltime);
	ROS_INFO("VELOCITY : [%d] \n", msg->motorInfos[1].encoder_vel);
	ROS_INFO("POSITION : [%d] \n", msg->motorInfos[1].encoder_pos);
	ROS_INFO("DIRECTION : [%d] \n", msg->motorInfos[1].encoder_dir);
	ROS_INFO("\n >>>>>> HERE WE GO: \n \n");
	ros::Rate r(1.0);




	//compute odometry in a typical way given the velocities of the robot
	double dt = (current_time - last_time).toSec();
	double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
	double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
	double delta_th = vth * dt;

	x += delta_x;
	y += delta_y;
	th += delta_th;

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vth;

	//publish the message
	odom_pub.publish(odom);

	last_time = current_time;


	r.sleep();
	*/


	/*
	ROS_INFO("Motor Encoder Pos: [%d, %d, %d, %d, %d, %d]", msg->motorInfos[0].encoder_pos, msg->motorInfos[1].encoder_pos, msg->motorInfos[2].encoder_pos
		   , msg->motorInfos[3].encoder_pos, msg->motorInfos[4].encoder_pos, msg->motorInfos[5].encoder_pos);
	ROS_INFO("Motor Encoder Vel: [%d, %d, %d, %d, %d, %d]", msg->motorInfos[0].encoder_vel, msg->motorInfos[1].encoder_vel, msg->motorInfos[2].encoder_vel
			 , msg->motorInfos[3].encoder_vel, msg->motorInfos[4].encoder_vel, msg->motorInfos[5].encoder_vel);
	ROS_INFO("Motor Encoder Dir: [%d, %d, %d, %d, %d, %d]", msg->motorInfos[0].encoder_dir, msg->motorInfos[1].encoder_dir, msg->motorInfos[2].encoder_dir
			 , msg->motorInfos[3].encoder_dir, msg->motorInfos[4].encoder_dir, msg->motorInfos[5].encoder_dir);
	ROS_INFO("Motor Motor Current: [%2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f]", msg->motorInfos[0].motor_current, msg->motorInfos[1].motor_current, msg->motorInfos[2].motor_current
			   , msg->motorInfos[3].motor_current, msg->motorInfos[4].motor_current, msg->motorInfos[5].motor_current);
	ROS_INFO("Motor Motor_PWM: [%d, %d, %d, %d, %d, %d]", msg->motorInfos[0].motor_pwm, msg->motorInfos[1].motor_pwm, msg->motorInfos[2].motor_pwm
			 , msg->motorInfos[3].motor_pwm, msg->motorInfos[4].motor_pwm, msg->motorInfos[5].motor_pwm);
	*/
  }
}



int main(int argc, char **argv)
{
  /*!
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "odometry_publisher");

  /*!
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  
  ROS_INFO("\n \n >>>>>> WWWWWWWWWWW \n");
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  /*!
   *  here will subscribe the sensor messages which are published by drrobot_player
   *  The second parameter to the subscribe() function is set as '1' because we are only
   *  interested at latest sensor information
   */
  ros::Subscriber motorSensorSub = n.subscribe("drrobot_motor", 1, motorSensorCallback);

  /*!
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}


/*
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  //::Subscriber cmd_vel_sub_;
  //cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("drrobot_cmd_vel", 1, boost::bind(&DrRobotPlayerNode::cmdVelReceived, this, _1));
  
  
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  

  ros::Rate r(1.0);
  
  
  while(n.ok()){


    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    
    
    r.sleep();
  }
}
*/
