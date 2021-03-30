/*!
 *  drrobot_h20_player
 *  Copyright (c) 2011, Dr Robot Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*!

@mainpage
  drrobot_h20_player is a driver for motion control system on I90/Sentinel3/Hawk/H20/X80SV/Jaguar series mobile robot, available from
<a href="http://www.drrobot.com">Dr Robot </a>.
<hr>

@section usage Usage
@par     After start roscore, you need load robot configuration file to parameter server first.
          For example, I90 robot, you need load drrobotplayer_I90.yaml use command "rosparam load drrobotplayer_I90.yaml"
@verbatim
$ drrobot_h20_player
@endverbatim

<hr>
@section topic ROS topics

Subscribes to (name/type):
- @b "cmd_vel"/Twist : velocity commands to differentially drive the robot.
- @b will develop other command subscribles in future, such as servo control.

Publishes to (name / type):
-@b drrobot_motor: will publish MotionInfoArray Message. Please referee the message file.
-@b drrobot_powerinfo: will publish PowerInfo Message. Please referee the message file.
-@b drrobot_ir: will publish RangeArray Message for IR sensor, and transform AD value from DrRobotMotionSensorDriver to distance value in unit meter. Please referee the message file.
-@b drrobot_sonar: will publish RangeArray Message for ultrasonic sensor, and transform value from DrRobotMotionSensorDriver to distance value in unit meter. Please referee the message file.
-@b drrobot_standardsensor: will publish StandardardSensor Message. Please referee the message file.
-@b drrobot_customsensor: will publish CustomSensor Message. Please referee the message file. Not available for standard I90/Sentinel3/Hawk/H20/X80SV robot

<hr>

@section parameters ROS parameters, please read yaml file

- @b RobotCommMethod (string) : Robot communication method, normally is "Network".
- @b RobotID (string) : specify the robot ID
- @b RobotBaseIP (string) : robot main WiFi module IP address in dot format, default is "192.168.0.201".
- @b RobotPortNum (string) : socket port number first serial port, and as default the value increased by one will be second port number.
- @b RobotSerialPort (int) : specify the serial port name if you choose serial communication in RobotCommMethod, default /dev/ttyS0"
- @b RobotType (string) : specify the robot type, now should in list: I90, Sentinel3, Hawk_H20, Jaguar, X80SV
- @b MotorDir (int) : specify the motor control direction
- @b WheelRadius (double) : wheel radius
- @b WheelDistance (double) : the distance between two driving wheels
- @b EncoderCircleCnt (int) : one circle encoder count
- @b MinSpeed (double) : minimum speed, unit is m/s.
- @b MaxSpeed (double) : maximum speed, unit is m/s.
- @b enable_ir (bool)  : Whether to enable sonar range sensors. Default: true.
- @b enable_sonar (bool)  : Whether to enable IR range sensors. Default: true.
 */

#include <math.h>
#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <drrobot_h20_player/HeadCmd.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <drrobot_h20_player/MotorInfo.h>
#include <drrobot_h20_player/MotorInfoArray.h>
#include <drrobot_h20_player/RangeArray.h>
#include <drrobot_h20_player/Range.h>
#include <drrobot_h20_player/PowerInfo.h>
#include <drrobot_h20_player/StandardSensor.h>
#include <drrobot_h20_player/CustomSensor.h>
#include "DrRobotMotionSensorDriver.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "odometry.h"									//Geoff	Odom


#define MOTOR_NUM       6
#define IR_NUM          10
#define US_NUM          6
using namespace std;
using namespace DrRobot_MotionSensorDriver;

struct RobotOdomState{
	int encoder_left;		// Encoder left ticks	(zero is left in array)
	int encoder_right;		// Encoder right ticks	(one is right in array)
	double ticks_metre;		// Encoder ticks per a metre
	double wheel_distance;		// Robot wheel distances
	double wheel_radius;	// Robot wheel radii
	double x;			// x-position real world
	double y;			// y-position real world
	double theta;			// theta position real world
	double linearX;			// linear velocity X
	double linearY;			// linear velocity Y
	double angularZ;		// Angular velocity
	double time;			// Time of variable in seconds
};



class DrRobotPlayerNode
{
public:

    ros::NodeHandle node_;

    tf::TransformBroadcaster tf_;

    ros::Publisher motorInfo_pub_;
    ros::Publisher powerInfo_pub_;
    ros::Publisher ir_pub_;
    ros::Publisher sonar_pub_;
    ros::Publisher standardSensor_pub_;
    ros::Publisher customSensor_pub_;
    ros::Publisher HeadPose_pub_;

    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber cmd_head_sub_;
    ros::Subscriber cmd_pose_head_sub_;

    std::string robot_prefix_;

    ros::Time current_time, last_time;			 	//tangy
    double testTime;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Publisher odom_pub_;

    RobotOdomState tangyOdom;

    //Head variables
    int neck_x_rotation_;
    int neck_z_rotation_;
    int mouth_;
    int upper_head_;
    int left_eye_;
    int right_eye_;

    Odometry odometry_;				// Geoff Odom
    double wheel_left_total;
    double wheel_right_total;
    bool initialized;
				
    DrRobotPlayerNode()
    {
        ros::NodeHandle private_nh("~");

        //Head stuffs
        neck_x_rotation_ = 3500;
	neck_z_rotation_ = 3450;
	mouth_ = 3500;
	upper_head_ = 3600;
	left_eye_ = 3600;
	right_eye_ = 3350;
	
	HeadPose_pub_ = node_.advertise<drrobot_h20_player::HeadCmd>("head_pose", 1);

        robotID_ = "drobot1";
        private_nh.getParam("RobotID",robotID_);
        ROS_INFO("I get ROBOT_ID: [%s]", robotID_.c_str());

        //robotType_ = "Jaguar";
        robotType_ = "Hawk_H20";
        private_nh.getParam("RobotType",robotType_);
        ROS_INFO("I get ROBOT_Type: [%s]", robotType_.c_str());

        robotCommMethod_ = "Network";
        private_nh.getParam("RobotCommMethod",robotCommMethod_);
        ROS_INFO("I get ROBOT_CommMethod: [%s]", robotCommMethod_.c_str());

        robotIP_ = "192.168.0.95";
        private_nh.getParam("RobotBaseIP",robotIP_);
        ROS_INFO("I get ROBOT_IP: [%s]", robotIP_.c_str());

        commPortNum_ = 10001;
        private_nh.getParam("RobotPortNum",commPortNum_);
        ROS_INFO("I get ROBOT_PortNum: [%d]", commPortNum_);

        robotSerialPort_ = "/dev/ttyS0";
        private_nh.getParam("RobotSerialPort",robotSerialPort_);
        ROS_INFO("I get ROBOT_SerialPort: [%s]", robotSerialPort_.c_str());

        enable_ir_ = true;
        private_nh.getParam("Enable_IR", enable_ir_);
        if (enable_ir_)
          ROS_INFO("I get Enable_IR: true");
        else
          ROS_INFO("I get Enable_IR: false");


        enable_sonar_ = true;
        private_nh.getParam("Enable_US", enable_sonar_);
        if (enable_sonar_)
          ROS_INFO("I get Enable_US: true");
        else
          ROS_INFO("I get Enable_US: false");

        motorDir_ = 1;
        private_nh.getParam("MotorDir", motorDir_);
        ROS_INFO("I get MotorDir: [%d]", motorDir_);

        wheelRadius_ = 0.0835;
        private_nh.getParam("WheelRadius", wheelRadius_);
        ROS_INFO("I get Wheel Radius: [%f]", wheelRadius_);

        wheelDis_ = 0.4001;
        private_nh.getParam("WheelDistance", wheelDis_);
        ROS_INFO("I get Wheel Distance: [%f]", wheelDis_);

        minSpeed_ = 0.1;
        private_nh.getParam("MinSpeed", minSpeed_);
        ROS_INFO("I get Min Speed: [%f]", minSpeed_);

        maxSpeed_ = 1.0;
        private_nh.getParam("MaxSpeed", maxSpeed_);
        ROS_INFO("I get Max Speed: [%f]", maxSpeed_);

        encoderOneCircleCnt_ = 800;
        private_nh.getParam("EncoderCircleCnt", encoderOneCircleCnt_);
        ROS_INFO("I get Encoder One Circle Count: [%d]", encoderOneCircleCnt_);

        if (robotCommMethod_ == "Network")
        {
          robotConfig1_.commMethod = Network;
          robotConfig2_.commMethod = Network;
        }
        else
        {
          robotConfig1_.commMethod = Serial;
          robotConfig2_.commMethod = Serial;
        }

        if (robotType_ == "Jaguar")
        {
          robotConfig1_.boardType = Jaguar;
        }
        else if(robotType_ == "I90")
        {
          robotConfig1_.boardType = I90_Power;
          robotConfig2_.boardType = I90_Motion;
        }
        else if (robotType_ == "Sentinel3")
        {
          robotConfig1_.boardType = Sentinel3_Power;
          robotConfig2_.boardType = Sentinel3_Motion;
        }
        else if (robotType_ == "Hawk_H20")
        {
          robotConfig1_.boardType = Hawk_H20_Power;
          robotConfig2_.boardType = Hawk_H20_Motion;
        }
        else if(robotType_ == "X80SV")
        {
          robotConfig1_.boardType = X80SV;
        }

        robotConfig1_.portNum = commPortNum_;
        robotConfig2_.portNum = commPortNum_ + 1;

	  //  strcat(robotConfig1_.robotIP,robotIP_.c_str());
	  strcpy(robotConfig1_.robotIP,robotIP_.c_str());
	  //  strcat(robotConfig2_.robotIP,robotIP_.c_str());
	  strcpy(robotConfig2_.robotIP,robotIP_.c_str());

	  //  strcat(robotConfig1_.serialPortName,robotSerialPort_.c_str());
	  strcpy(robotConfig1_.serialPortName,robotSerialPort_.c_str());
	  //  strcat(robotConfig2_.serialPortName,robotSerialPort_.c_str());
	  strcpy(robotConfig2_.serialPortName,robotSerialPort_.c_str());
        //create publishers for sensor data information
        motorInfo_pub_ = node_.advertise<drrobot_h20_player::MotorInfoArray>("drrobot_motor", 1);
        powerInfo_pub_ = node_.advertise<drrobot_h20_player::PowerInfo>("drrobot_powerinfo", 1);
        if (enable_ir_) { ir_pub_ = node_.advertise<drrobot_h20_player::RangeArray>("drrobot_ir", 1); }
        if (enable_sonar_) { sonar_pub_ = node_.advertise<drrobot_h20_player::RangeArray>("drrobot_sonar",1); }
        standardSensor_pub_ = node_.advertise<drrobot_h20_player::StandardSensor>("drrobot_standardsensor", 1);
        customSensor_pub_ = node_.advertise<drrobot_h20_player::CustomSensor>("drrobot_customsensor", 1);

        odom_pub_ = node_.advertise<nav_msgs::Odometry>("odom", 5); //Tangy


        drrobotPowerDriver_ = new DrRobotMotionSensorDriver();
        drrobotMotionDriver_ = new DrRobotMotionSensorDriver();
        if (  (robotType_ == "Jaguar") )
        {
          drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
        }
        else
        {
          drrobotPowerDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
          drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig2_);
        }
        cntNum_ = 0;

	//INITIALIZE ALL ODOMETRY INFORMATION
	
	tangyOdom.encoder_left = 0;
	tangyOdom.encoder_right = 0;
	tangyOdom.ticks_metre = (377)/(0.5225);
	tangyOdom.wheel_distance = 0.4001;
	tangyOdom.wheel_radius = 0.083158458;
	tangyOdom.x = 0;
	tangyOdom.y = 0;
	tangyOdom.theta = 0;
	tangyOdom.linearX = 0;
	tangyOdom.linearY = 0;
	tangyOdom.angularZ = 0;
	tangyOdom.time = -1.0;

	odometry_.setWheelParams(tangyOdom.wheel_distance, tangyOdom.wheel_radius);			//Geoff Odom

    }

    ~DrRobotPlayerNode()
    {
    }

    int start()
    {

      int res = -1;
      if (  (robotType_ == "Jaguar"))
      {
        res = drrobotMotionDriver_->openNetwork(robotConfig1_.robotIP,robotConfig1_.portNum);
	if (res == 0)
	{
		ROS_INFO("open port number at: [%d]", robotConfig1_.portNum);
	}
	else
	{
		ROS_INFO("could not open network connection to [%s,%d]",  robotConfig1_.robotIP,robotConfig1_.portNum);
		//ROS_INFO("error code [%d]",  res);
	}

      }
      else
      {
        drrobotMotionDriver_->openNetwork(robotConfig2_.robotIP,robotConfig2_.portNum);
        drrobotPowerDriver_->openNetwork(robotConfig1_.robotIP,robotConfig1_.portNum);
      // odom_pub
      }

      //cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("drrobot_cmd_vel", 1, boost::bind(&DrRobotPlayerNode::cmdVelReceived, this, _1));
      cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&DrRobotPlayerNode::cmdVelReceived, this, _1));

      //head subscriber
      cmd_head_sub_ = node_.subscribe<drrobot_h20_player::HeadCmd>("cmd_head", 1, boost::bind(&DrRobotPlayerNode::cmdHeadReceived, this, _1));
      cmd_pose_head_sub_ = node_.subscribe<drrobot_h20_player::HeadCmd>("cmd_pose_head", 1, boost::bind(&DrRobotPlayerNode::cmdPoseHeadReceived, this, _1));

 	odometry_.init(ros::Time::now());			//Geoff Odom
	wheel_left_total = 0;
	wheel_right_total = 0;
	odometry_.update(wheel_left_total, wheel_right_total, current_time);
     	initialized = false;
        return(0);
    }

    int stop()
    {
        int status = 0;
        drrobotMotionDriver_->close();
        drrobotPowerDriver_->close();				//Changed July 25, 2014
        usleep(1000000);
        return(status);
    }

    void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
      double g_vel = 1.5*cmd_vel->linear.x;
      double t_vel = 3*cmd_vel->angular.z;

      if(g_vel > 4) {
            g_vel = 4;
      }
      if(g_vel < -3) {
            g_vel = -3;
      }
      if(t_vel > 1.5) {
            t_vel = 1.5;
      }
      if(t_vel < -1.5) {
            t_vel = -1.5;
      }
      if (robotConfig1_.boardType != Jaguar)
      {
        double leftWheel = (2 * g_vel - t_vel* wheelDis_) / (2 * wheelRadius_);
        double rightWheel = (t_vel* wheelDis_ + 2 * g_vel) / (2 * wheelRadius_);

        int leftWheelCmd = motorDir_ * leftWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
        int rightWheelCmd = - motorDir_ * rightWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
        // ROS_INFO("Received control command: [%d, %d]", leftWheelCmd,rightWheelCmd);
        drrobotMotionDriver_->sendMotorCtrlAllCmd(Velocity,leftWheelCmd, rightWheelCmd,NOCONTROL,NOCONTROL, NOCONTROL,NOCONTROL);
      }
      else
      {
         int forwardPWM = -motorDir_ * g_vel * 16384 + 16384;
         int turnPWM = -motorDir_ * t_vel * 16384 + 16384;
         if (forwardPWM > 32767) forwardPWM = 32767;
         if (forwardPWM < 0) forwardPWM = 0;
         if (turnPWM > 32767) turnPWM = 32767;
         if (turnPWM < 0) turnPWM = 0;
         drrobotMotionDriver_->sendMotorCtrlAllCmd(PWM,NOCONTROL,NOCONTROL,NOCONTROL,forwardPWM,turnPWM, NOCONTROL);
      }

    }

    //Head callback
    void cmdHeadReceived(const drrobot_h20_player::HeadCmd::ConstPtr& cmd_head)
    {
	neck_x_rotation_ += cmd_head->neck_x_rotation;
      neck_z_rotation_ += cmd_head->neck_z_rotation;
      mouth_ += cmd_head->mouth;
      upper_head_ += cmd_head->upper_head;
      left_eye_ += cmd_head->left_eye;
      right_eye_ += cmd_head->right_eye;

			if (neck_x_rotation_ > 4200) neck_x_rotation_ = 4200;
			if (neck_x_rotation_ < 2900) neck_x_rotation_ = 2900;
			
			if (neck_z_rotation_ > 5500) neck_z_rotation_ = 4000;
			if (neck_z_rotation_ < 1400) neck_z_rotation_ = 2000;
			
			if (mouth_ > 3600) mouth_ = 3600;
			if (mouth_ < 2950) mouth_ = 2950;
			
			if (upper_head_ > 5200) upper_head_ = 5200;
			if (upper_head_ < 2500) upper_head_ = 2500;
			
			if (left_eye_ > 4600) left_eye_ = 4600;
			if (left_eye_ < 2000) left_eye_ = 2000;
			
			if (right_eye_ > 4950) right_eye_ = 4950;
			if (right_eye_ < 2350) right_eye_ = 2350;

			if (cmd_head->flag == 1)
			{
	  		   neck_x_rotation_ = 0;
			   neck_z_rotation_ = 0;
		 	   mouth_ = NOCONTROL;
			   upper_head_ = 0;
			   left_eye_ = 0;
			   right_eye_ = 0;
			}
			
			if (cmd_head->flag == 2)
			{
	  			neck_x_rotation_ = 3550;
				neck_z_rotation_ = 3450;
				mouth_ = 3500;
				upper_head_ = 3600;
				left_eye_ = 3600;
				right_eye_ = 3350;
			}
			
			drrobotMotionDriver_->sendServoCtrlAllCmd(neck_x_rotation_, neck_z_rotation_,mouth_,upper_head_, left_eye_,right_eye_,500);
		}
		
	void cmdPoseHeadReceived(const drrobot_h20_player::HeadCmd::ConstPtr& cmd_head)
    {
    	neck_x_rotation_ = cmd_head->neck_x_rotation;
      neck_z_rotation_ = cmd_head->neck_z_rotation;
      mouth_ = cmd_head->mouth;
      upper_head_ = cmd_head->upper_head;
      left_eye_ = cmd_head->left_eye;
      right_eye_ = cmd_head->right_eye;

			
			if (neck_x_rotation_ > 4500) neck_x_rotation_ = 4500;
			if (neck_x_rotation_ < 3000) neck_x_rotation_ = 3000;
			
			if (neck_z_rotation_ > 5500) neck_z_rotation_ = 5500;
			if (neck_z_rotation_ < 1400) neck_z_rotation_ = 1400;
			
			if (mouth_ > 3600) mouth_ = 3600;
			if (mouth_ < 2950) mouth_ = 2950;
			
			if (upper_head_ > 5200) upper_head_ = 5200;
			if (upper_head_ < 2500) upper_head_ = 2500;
			
			if (left_eye_ > 4600) left_eye_ = 4600;
			if (left_eye_ < 2000) left_eye_ = 2000;
			
			if (right_eye_ > 4950) right_eye_ = 4950;
			if (right_eye_ < 2350) right_eye_ = 2350;

			
			drrobotMotionDriver_->sendServoCtrlAllCmd(neck_x_rotation_, neck_z_rotation_,mouth_,upper_head_, left_eye_,right_eye_,cmd_head->flag);
		}

    void doUpdate()
    {
	drrobotPowerDriver_->sendAck();				//This should keep the robot on

      if ( (robotConfig1_.boardType == I90_Power) || (robotConfig1_.boardType == Sentinel3_Power)
          || (robotConfig1_.boardType == Hawk_H20_Power) )
      {
        if (drrobotPowerDriver_->portOpen())
        {
          //sendPowerCtrlCmd
          //drrobotPowerDriver_->sendPowerCtrlCmd(0x0000ca); //it cannot be on when using the arms
	  //rrobotPowerDriver_->sendPowerCtrlCmd(0x000037);
	  drrobotPowerDriver_->sendPowerCtrlCmd(0xff);		//Turn on the robot

          drrobotPowerDriver_->readPowerSensorData(&powerSensorData_);
          drrobot_h20_player::PowerInfo powerInfo;
          powerInfo.ref_vol = 1.5 * 4095 /(double)powerSensorData_.refVol;

          powerInfo.bat1_vol = (double)powerSensorData_.battery1Vol  * 8 / 4095 * powerInfo.ref_vol;
          powerInfo.bat2_vol = (double) powerSensorData_.battery2Vol * 8 / 4095 * powerInfo.ref_vol;
          //odom_pub
          powerInfo.bat1_temp = powerSensorData_.battery1Thermo;
          powerInfo.bat2_temp = powerSensorData_.battery2Thermo;

          powerInfo.dcin_vol = (double)powerSensorData_.dcINVol * 8 / 4095 * powerInfo.ref_vol;
          powerInfo.charge_path = powerSensorData_.powerChargePath;
          powerInfo.power_path = powerSensorData_.powerPath;
          powerInfo.power_status = powerSensorData_.powerStatus;

          powerInfo_pub_.publish(powerInfo);
        }
      }
      if (drrobotMotionDriver_->portOpen())
      {
        drrobotMotionDriver_->readMotorSensorData(&motorSensorData_);
        drrobotMotionDriver_->readRangeSensorData(&rangeSensorData_);
        drrobotMotionDriver_->readStandardSensorData(&standardSensorData_);

        drrobotMotionDriver_->readCustomSensorData(&customSensorData_);
              // Translate from driver data to ROS data
            cntNum_++;
              drrobot_h20_player::MotorInfoArray motorInfoArray;
              motorInfoArray.motorInfos.resize(MOTOR_NUM);
              for (uint32_t i = 0 ; i < MOTOR_NUM; ++i)
              {
                  motorInfoArray.motorInfos[i].header.stamp = ros::Time::now();
                  motorInfoArray.motorInfos[i].header.frame_id = string("drrobot_motor_");
                  motorInfoArray.motorInfos[i].header.frame_id += boost::lexical_cast<std::string>(i);
                  motorInfoArray.motorInfos[i].robot_type = robotConfig1_.boardType;
                  motorInfoArray.motorInfos[i].encoder_pos = motorSensorData_.motorSensorEncoderPos[i];
                  motorInfoArray.motorInfos[i].encoder_vel = motorSensorData_.motorSensorEncoderVel[i];
                  motorInfoArray.motorInfos[i].encoder_dir = motorSensorData_.motorSensorEncoderDir[i];
                  if (robotConfig1_.boardType == Hawk_H20_Motion)
                  {
                    motorInfoArray.motorInfos[i].motor_current = (float)motorSensorData_.motorSensorCurrent[i] * 3 /4096;;
                  }
                  else if(robotConfig1_.boardType != Jaguar)
                  {
                    motorInfoArray.motorInfos[i].motor_current = (float)motorSensorData_.motorSensorCurrent[i] / 728;
                  }
                  else
                  {
                    motorInfoArray.motorInfos[i].motor_current = 0.0;
                  }
                  motorInfoArray.motorInfos[i].motor_pwm = motorSensorData_.motorSensorPWM[i];
              }

              //ROS_INFO("publish motor info array");
              motorInfo_pub_.publish(motorInfoArray);



	      
	      //Geoff Odom Starts
	      
              

	// old == tangyOdom.encoder_left
	// new == motorSensorData_.motorSensorEncoderPos[0]
	     if (!initialized && motorSensorData_.motorSensorEncoderPos[0] != 0 && motorSensorData_.motorSensorEncoderPos[1] != 0) {
		ROS_INFO("Initializing");
		tangyOdom.encoder_left = motorSensorData_.motorSensorEncoderPos[0];
		tangyOdom.encoder_right = motorSensorData_.motorSensorEncoderPos[1];
		initialized = true;
	     }

             const int UPDATE_TICKS = 5;
	     // ROS_INFO("Left Encoder = %d", motorSensorData_.motorSensorEncoderPos[0]);
	     // ROS_INFO("Right Encoder = %d", motorSensorData_.motorSensorEncoderPos[1]);
	     if(fabs(tangyOdom.encoder_left - motorSensorData_.motorSensorEncoderPos[0]) > UPDATE_TICKS ||
		fabs(tangyOdom.encoder_right - motorSensorData_.motorSensorEncoderPos[1]) > UPDATE_TICKS){
		      
		current_time = ros::Time::now();
		float left_pos;

		// no rollover (encoder doesnt go from 0 to 32767 or from 32767 to 0
		if( fabs(motorSensorData_.motorSensorEncoderPos[0]-tangyOdom.encoder_left) < (32767/2) ) {
			left_pos = (motorSensorData_.motorSensorEncoderPos[0] - tangyOdom.encoder_left)/tangyOdom.ticks_metre;
			//ROS_INFO("No rollover, left_pos: %f", left_pos);
		}
		// 32767 to 0 old bigger than new (moved forward)
		else if(tangyOdom.encoder_left > motorSensorData_.motorSensorEncoderPos[0]) {
			left_pos = (32767 - tangyOdom.encoder_left + motorSensorData_.motorSensorEncoderPos[0])/tangyOdom.ticks_metre;
			//ROS_INFO("Forward Rollover, left_pos: %f", left_pos);
		}
		// 0 to 32767 new bigger than old (moved backward)
		else {
			left_pos = -(32767 - motorSensorData_.motorSensorEncoderPos[0] + tangyOdom.encoder_left )/tangyOdom.ticks_metre;
			//ROS_INFO("Backward Rollover, left_pos: %f", left_pos);
		}
		left_pos = left_pos/tangyOdom.wheel_radius;
		wheel_left_total += left_pos;

		float right_pos;
		// no rollover (encoder doesnt go from 0 to 32767 or from 32767 to 0
		if( fabs(motorSensorData_.motorSensorEncoderPos[1]-tangyOdom.encoder_right) < (32767/2) ) {
			right_pos = (motorSensorData_.motorSensorEncoderPos[1] - tangyOdom.encoder_right)/tangyOdom.ticks_metre;
			//ROS_INFO("No Rollover, right_pos: %f", left_pos);
		}
		// 0 to 32767 new bigger than the old (moved forward) -- opposite of the left wheel
		else if(motorSensorData_.motorSensorEncoderPos[1] > tangyOdom.encoder_right) {
			right_pos = (32767 - motorSensorData_.motorSensorEncoderPos[1] + tangyOdom.encoder_right)/tangyOdom.ticks_metre;
			//ROS_INFO("Forward Rollover, right_pos: %f", left_pos);
		}
		// 32767 to 0 old bigger than the new (moved backward) -- opposite of the left wheel
		else {
			right_pos = -(32767 - tangyOdom.encoder_right + motorSensorData_.motorSensorEncoderPos[1])/tangyOdom.ticks_metre;
			//ROS_INFO("Backward Rollover, right_pos: %f", left_pos);
		}

		right_pos = -right_pos/tangyOdom.wheel_radius;
		wheel_right_total += right_pos;

		odometry_.update(wheel_left_total , wheel_right_total , current_time);
		//ROS_INFO("X: [%f]\n", odometry_.getX());
		//ROS_INFO("Y: [%f]\n", odometry_.getY());
		//ROS_INFO("ROTATION: [%f}\n",odometry_.getHeading());

		tangyOdom.encoder_left = motorSensorData_.motorSensorEncoderPos[0];
		tangyOdom.encoder_right = motorSensorData_.motorSensorEncoderPos[1];
	      }
			//get quaternion form of yaw
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odometry_.getHeading());

			//publish tranform over tf
			geometry_msgs::TransformStamped odom_trans;

			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_link";
			odom_trans.transform.translation.x = odometry_.getX();
			odom_trans.transform.translation.y = odometry_.getY();
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;
			odom_broadcaster.sendTransform(odom_trans);

			//publish odometry over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_link";

			odom.pose.pose.position.x = odometry_.getX();
			odom.pose.pose.position.y = odometry_.getY();
			odom.pose.pose.orientation = odom_quat;

			odom.twist.twist.linear.x = odometry_.getLinear();
			odom.twist.twist.angular.z = odometry_.getAngular();

			odom_pub_.publish(odom);
				

		     	//TANGY STUFF ENDS HERE



              drrobot_h20_player::RangeArray rangerArray;
              rangerArray.ranges.resize(US_NUM);
	      if(enable_sonar_)
	      {
		      for (uint32_t i = 0 ; i < US_NUM; ++i)
		      {

		          rangerArray.ranges[i].header.stamp = ros::Time::now();
		          rangerArray.ranges[i].header.frame_id = string("drrobot_sonar_");
		          rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
		          rangerArray.ranges[i].range = (float)rangeSensorData_.usRangeSensor[i]/100;     //to meters

		          // around 30 degrees
		          rangerArray.ranges[i].field_of_view = 0.5236085;
		          rangerArray.ranges[i].max_range = 2.55;
		          rangerArray.ranges[i].min_range = 0;
		          rangerArray.ranges[i].radiation_type = drrobot_h20_player::Range::ULTRASOUND;
		      }

		      sonar_pub_.publish(rangerArray);
		}


	      if(enable_ir_)
	      {
		      rangerArray.ranges.resize(IR_NUM);
		      for (uint32_t i = 0 ; i < IR_NUM; ++i)
		      {
		          rangerArray.ranges[i].header.stamp = ros::Time::now();
		          rangerArray.ranges[i].header.frame_id = string("drrobot_ir_");
		          rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
		          rangerArray.ranges[i].range = ad2Dis(rangeSensorData_.irRangeSensor[i]);
		          rangerArray.ranges[i].radiation_type = drrobot_h20_player::Range::INFRARED;
		      }

		      ir_pub_.publish(rangerArray);
	     }

              drrobot_h20_player::StandardSensor standardSensor;
              standardSensor.humanSensorData.resize(4);
              standardSensor.tiltingSensorData.resize(2);
              standardSensor.overHeatSensorData.resize(2);
              standardSensor.header.stamp = ros::Time::now();
              standardSensor.header.frame_id = string("drrobot_standardsensor");
              for (uint32_t i = 0; i < 4; i++)
                standardSensor.humanSensorData[i] = standardSensorData_.humanSensorData[i];
              for (uint32_t i = 0; i < 2; i++)
                standardSensor.tiltingSensorData[i] = standardSensorData_.tiltingSensorData[i];
              for (uint32_t i = 0; i < 2; i++)
                standardSensor.overHeatSensorData[i] = standardSensorData_.overHeatSensorData[i];

              standardSensor.thermoSensorData = standardSensorData_.thermoSensorData;

              standardSensor.boardPowerVol = (double)standardSensorData_.boardPowerVol * 9 /4095;
              standardSensor.servoPowerVol = (double)standardSensorData_.servoPowerVol * 9 /4095;

              if (robotConfig1_.boardType != Jaguar)
              {
                standardSensor.motorPowerVol = (double)standardSensorData_.motorPowerVol * 24 /4095;
              }
              else
              {
                standardSensor.motorPowerVol = (double)standardSensorData_.motorPowerVol * 34.498 /4095;
              }
              standardSensor.refVol = (double)standardSensorData_.refVol / 4095 * 6;
              standardSensor.potVol = (double)standardSensorData_.potVol / 4095 * 6;
              standardSensor_pub_.publish(standardSensor);

              drrobot_h20_player::CustomSensor customSensor;
              customSensor.customADData.resize(8);
              customSensor.header.stamp = ros::Time::now();
              customSensor.header.frame_id = string("drrobot_customsensor");

              for (uint32_t i = 0; i < 8; i ++)
              {
                customSensor.customADData[i] = customSensorData_.customADData[i];
              }
              customSensor.customIO = (uint8_t)(customSensorData_.customIO & 0xff);
              customSensor_pub_.publish(customSensor);
      }
      
      //Head pose publish
      drrobot_h20_player::HeadCmd HeadPose;
      HeadPose.neck_x_rotation = neck_x_rotation_;
      HeadPose.neck_z_rotation = neck_z_rotation_;
      HeadPose.mouth = mouth_;
      HeadPose.upper_head = upper_head_;
      HeadPose.left_eye = left_eye_;
      HeadPose.right_eye = right_eye_;
      
      HeadPose_pub_.publish(HeadPose);
      
      
      
      
    }

private:

    DrRobotMotionSensorDriver* drrobotMotionDriver_;
    DrRobotMotionSensorDriver* drrobotPowerDriver_;
    struct DrRobotMotionConfig robotConfig1_;
    struct DrRobotMotionConfig robotConfig2_;

    std::string odom_frame_id_;
    struct MotorSensorData motorSensorData_;
    struct RangeSensorData rangeSensorData_;
    struct PowerSensorData powerSensorData_;
    struct StandardSensorData standardSensorData_;
    struct CustomSensorData customSensorData_;


    std::string robotType_;
    std::string robotID_;
    std::string robotIP_;
    std::string robotCommMethod_;
    std::string robotSerialPort_;
    bool enable_ir_;
    bool enable_sonar_;
    int  commPortNum_;
    int  encoderOneCircleCnt_;
    double wheelDis_;
    double wheelRadius_;
    int motorDir_;
    double minSpeed_;
    double maxSpeed_;

    int cntNum_;
    double ad2Dis(int adValue)
    {
      double temp = 0;
      double irad2Dis = 0;

      if (adValue <= 0)
        temp = -1;
      else
        temp = 21.6 /((double)adValue * 3 /4096 - 0.17);

      if ( (temp > 80) || (temp < 0))
      {
        irad2Dis = 0.81;
      }
      else if( (temp < 10) && (temp > 0))
      {
        irad2Dis = 0.09;
      }
      else
        irad2Dis = temp /100;
      return irad2Dis;
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "drrobot_h20_player");

    DrRobotPlayerNode drrobotPlayer;
    ros::NodeHandle n;
    // Start up the robot

	// pose publisher
/*
	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
	geometry_msgs::PoseWithCovarianceStamped origin;
	origin.header.frame_id = "map";
	for(int i = 0; i < 36; i++) {
		origin.pose.covariance[i] = 0;
	}
	origin.pose.pose.position.x = 0;
	origin.pose.pose.position.y = 0;
	origin.pose.pose.position.z = 0;
	origin.pose.pose.orientation.x = 0;
	origin.pose.pose.orientation.y = 0;
	origin.pose.pose.orientation.z = 0;
	origin.pose.pose.orientation.w = 1;
	pose_pub.publish(origin);
*/
	// set robot pose to (0,0,0,0,0,0)

    if (drrobotPlayer.start() != 0)
    {
        exit(-1);
    }
    /////////////////////////////////////////////////////////////////

       drrobotPlayer.current_time = ros::Time::now();
       drrobotPlayer.last_time = ros::Time::now();
       ros::Rate r(10);


    drrobotPlayer.current_time = ros::Time::now(); //tangy
    while (n.ok())
    {
      ros::spinOnce(); 	//tangy - moved this up here
      drrobotPlayer.current_time = ros::Time::now();	 //tangy

      drrobotPlayer.doUpdate();
      //ros::spinOnce();
      r.sleep();
    }
    /////////////////////////////////////////////////////////////////

    // Stop the robot
    drrobotPlayer.stop();

    return(0);
}

