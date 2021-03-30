//============================================================================
// Name        : DrRobot_Arm_Broadcaster.cpp
// Author      : Ananias Paiva
// Copyright   :
// Description : Broadcast tangy model frames and make tangy model move
//============================================================================

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <drrobot_h20_arm_player/ArmCmd.h>
#include <sensor_msgs/JointState.h>

#define pi 3.14159265

using namespace std;

class drrobotarmbroadcaster
{
private:
	ros::NodeHandle node;
	ros::Subscriber cmd_arm_sub;
	ros::Publisher pos_msg;
	tf::TransformBroadcaster br;
  	tf::Transform transform;
  	//TODO:change this variables to arrays
  	float ang1, ang2, ang3, ang4, ang5, ang6, ang7, ang8, ang9, ang10, ang11, ang12, ang13, ang14, ang15, ang16;
  	float c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16;
	
public:
	
	//set variables to defaut values
	drrobotarmbroadcaster()
	{
		ang1 = -pi/2;
		ang2 = -pi/2;
		ang3 = 0;
		ang4 = pi/2;
		ang5 = 0;
		ang6 = -pi/2;
		ang7 = -pi/2;
		ang8 = 0;
		ang9 = -pi/2;
		ang10 = -pi/2;
		ang11 = 0;
		ang12 = pi/2;
		ang13 = 0;
		ang14 = -pi/2;
		ang15 = -pi/2;
		ang16 = 0;
		
		cmd_arm_sub = node.subscribe<drrobot_h20_arm_player::ArmCmd>("cmd_arm", 1, boost::bind(&drrobotarmbroadcaster::cmdArmReceived, this, _1));
		pos_msg = node.advertise<sensor_msgs::JointState>("joint_states", 1);
	}	
	
	//This function is not necessary in here, but it is very usefull =)
	void createFrame(std::string parent, std::string child, double origin_x, double origin_y, double origin_z, double rotation_x, double rotation_y, double rotation_z)
	{
		transform.setOrigin( tf::Vector3(origin_x, origin_y, origin_z) );
    	transform.setRotation( tf::Quaternion(rotation_x, rotation_y, rotation_z) );
    	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, child));
    }
    
    double degreeToRad(float degree)
    {
    	double rad;
    	
    	rad = (pi*degree)/180;
    	
    	return rad;
    }
    
    //Create a jointstate msg and inicialize it with all joint angles that other functions will get from "cmd_arm"
    //This function make tangy model move like the real
    void sendPosition()
    {
    	ros::Rate r(10);
    	
    	sensor_msgs::JointState msg;
    	
    	std::string names[] = {"right_shoulder_x_revolute", "right_shoulder_y_revolute", "upper_right_arm_z_revolute", "upper_right_arm_x_revolute", "wrist_right_arm_z_revolute", "wrist_right_arm_x_revolute", "left_shoulder_x_revolute", "left_shoulder_y_revolute", "upper_left_arm_z_revolute", "upper_left_arm_x_revolute", "wrist_left_arm_z_revolute", "wrist_left_arm_x_revolute"};
    	
    	//Publish to "joint_state" all joint angles, so tangy model keep tracking of real tangy movements
    	while(node.ok())
    	{
    		msg.header.stamp = ros::Time::now();
    		msg.name.resize(12);
       	 	msg.position.resize(12);
    		
    		for(int i = 0;i < 12;i++)
    		{
    			msg.name[i] = names[i];
    		}
    		
    		msg.position[0] = ang8;
    		msg.position[1] = ang7;
    		msg.position[2] = ang6;
    		msg.position[3] = ang5;
    		msg.position[4] = ang4;
    		msg.position[5] = ang3;
    		
    		msg.position[6] = ang16;
    		msg.position[7] = ang15;
    		msg.position[8] = ang14;
    		msg.position[9] = ang13;
    		msg.position[10] = ang12;
    		msg.position[11] = ang11;
    	
    		pos_msg.publish(msg);
    		
    		ros::spinOnce();
    		r.sleep();
    	}
    	
    }
    
    //"arm_cmd" callback
    //send commands to another funciton to turn them into angles
    void cmdArmReceived(const drrobot_h20_arm_player::ArmCmd::ConstPtr& cmd_arm)
	{	
		angCheck(cmd_arm->right_arm[0],cmd_arm->right_arm[1],cmd_arm->right_arm[2],cmd_arm->right_arm[3],cmd_arm->right_arm[4],cmd_arm->right_arm[5],cmd_arm->right_arm[6],cmd_arm->right_arm[7],
				 cmd_arm->left_arm[0],cmd_arm->left_arm[1],cmd_arm->left_arm[2],cmd_arm->left_arm[3],cmd_arm->left_arm[4],cmd_arm->left_arm[5],cmd_arm->left_arm[6],cmd_arm->left_arm[7], 
				 cmd_arm->vel);	

	}	
	
	//This function get arm commands and divides them into small movements so when this commands go to the model it won't "jump" to a position to another
	//TODO: change all this varibles to arrays!!
	void angCheck(int a1, int a2, int a3, int a4, int a5, int a6, int a7, int a8, int a9, int a10, int a11, int a12, int a13, int a14, int a15, int a16, int vel)
	{
		float sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0, sum5 = 0, sum6 = 0, sum7 = 0, sum8 = 0, sum9 = 0, sum10 = 0, sum11 = 0, sum12 = 0, sum13 = 0, sum14 = 0, sum15 = 0, sum16 = 0,
		      dif1 = 0, dif2 = 0, dif3 = 0, dif4 = 0, dif5 = 0, dif6 = 0, dif7 = 0, dif8 = 0, dif9 = 0, dif10 = 0, dif11 = 0, dif12 = 0, dif13 = 0, dif14 = 0, dif15 = 0, dif16 = 0;
		c1 = 1700;
		c2 = 1269;
		c3 = 1625;
		c4 = 1545;
		c5 = 1710;
		c6 = 2000;
		c7 = 2300;
		c8 = 760;
		c9 = 1290;
		c10 = 1720;
		c11 = 1570;
		c12 = 1520;
		c13 = 1940;
		c14 = 2010;
		c15 = 1240;
		c16 = 2200;        
		
		if(c1 != a1 || c2 != a2 || c3 != a3 || c4 != a4 || c5 != a5 || c6 != a6 || c7 != a7 || c8 != a8 || c9 != a9 || c10 != a10 || c11 != a11 || c12 != a12 || 
		   c13 != a13 || c14 != a14 || c15 != a15 || c16 != a16)
		{
			dif1 = ((float)(a1 - c1)/(float)vel);
			dif2 = ((float)(a2 - c2)/(float)vel);
			dif3 = ((float)(a3 - c3)/(float)vel);
			dif4 = ((float)(a4 - c4)/(float)vel);
			dif5 = ((float)(a5 - c5)/(float)vel);
			dif6 = ((float)(a6 - c6)/(float)vel);
			dif7 = ((float)(a7 - c7)/(float)vel);
			dif8 = ((float)(a8 - c8)/(float)vel);
			dif9 = ((float)(a9 - c9)/(float)vel);
			dif10 = ((float)(a10 - c10)/(float)vel);
			dif11 = ((float)(a11 - c11)/(float)vel);
			dif12 = ((float)(a12 - c12)/(float)vel);
			dif13 = ((float)(a13 - c13)/(float)vel);
			dif14 = ((float)(a14 - c14)/(float)vel);
			dif15 = ((float)(a15 - c15)/(float)vel);
			dif16 = ((float)(a16 - c16)/(float)vel);

			for (int i = 1; i <= vel; i++)
			{
				sum1 = i*dif1;
				sum2 = i*dif2;
				sum3 = i*dif3;
				sum4 = i*dif4;
				sum5 = i*dif5;
				sum6 = i*dif6;
				sum7 = i*dif7;
				sum8 = i*dif8;
				sum9 = i*dif9;
				sum10 = i*dif10;
				sum11 = i*dif11;
				sum12 = i*dif12;
				sum13 = i*dif13;
				sum14 = i*dif14;
				sum15 = i*dif15;
				sum16 = i*dif16;
				 
				c1 = 1700 + sum1;
				c2 = 1269 + sum2;
				c3 = 1625 + sum3;
				c4 = 1545 + sum4;
				c5 = 1710 + sum5;
				c6 = 2000 + sum6;
				c7 = 2300 + sum7;
				c8 = 760 + sum8;
				c9 = 1290 + sum9;
				c10 = 1720 + sum10;
				c11 = 1570 + sum11;
				c12 = 1520 + sum12;
				c13 = 1940 + sum13;
				c14 = 2010 + sum14;
				c15 = 1240 + sum15;
				c16 = 2200 + sum16;

				cmdToAng(c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16);
			}
		}
	}
    
	//Tranform servo commands into angles and prepare them to be send to tangy model  
    void cmdToAng(int cmd1, int cmd2, int cmd3, int cmd4, int cmd5, int cmd6, int cmd7, int cmd8, int cmd9, int cmd10, int cmd11, int cmd12, int cmd13, int cmd14, int cmd15, int cmd16)
	{
		int i;
		float temp1,temp2,temp3,temp4,temp5,temp6,temp7,temp8,temp9,temp10,temp11,temp12,temp13,temp14,temp15,temp16;

		for (i = 0; i < 16; i++)
		{
			if (i == 0)
			{
				if (cmd1 < 1700)
				{
					temp1 = (-1700 + cmd1)/((1700.0 - 590.0)/(-90.0 - (30.0))) + (-90);
				}
				else
				{
					temp1 = (-1700 + cmd1)/((2110.0 - 1700.0)/(-150.0 - (-90.0))) + (-90);
				}
			}
			if (i == 1)
			{
				if (cmd2 < 1269)
				{
					temp2 = (-1269 + cmd2)/((1269.0 - 950.0)/(-90.0 - (-60.0))) + (-90);
				}
				else
				{
					temp2 = (-1269 + cmd2)/((2400.0 - 1269.0)/(-210.0 - (-90.0))) + (-90);
				}
			}
			if (i == 2)
			{
				if (cmd3 > 1625)
				{
					temp3 = (-1625 + cmd3)/((2250.0 - 1625.0)/(-60.0 - (0.0))) + (0);
				}
				else
				{
					temp3 = (-1625 + cmd3)/((1625.0 - 680.0)/(0 - (85.0))) + (0);
				}
			}
			if (i == 3)
			{
				if (cmd4 > 1545)
				{
					temp4 = (-1545 + cmd4)/((2350.0 - 1545.0)/(0.0 - (90.0))) + (90);
				}
				else
				{
					temp4 = (-1545 + cmd4)/((1545.0 - 580.0)/(90.0 - (180.0))) + (90);
				}
			}
			if (i == 4)
			{
				if (cmd5 > 1710)
				{
					temp5 = (-1710 + cmd5)/((2110.0 - 1710.0)/(-129.0 - (0.0))) + (0);
				}
				else
				{
					temp5 = (-1710 + cmd5)/((1710.0 - 1400.0)/(0.0 - (90.0))) + (0);
				}
			}
			if (i == 5)
			{
				if (cmd6 < 2000)
				{
					temp6 = (-2000 + cmd6)/((2000.0 - 1670.0)/(-90.0 - (-195.0))) + (-90);
				}
				else
				{
					temp6 = (-2000 + cmd6)/((2350.0 - 2000.0)/(15.0 - (-90.0))) + (-90);
				}
			}
			if (i == 6)
			{
				if (cmd7 < 2300)
				{
					temp7 = (-2300 + cmd7)/((2300.0 - 1460.0)/(-90.0 - (-200.0))) + (-90);
				}
				else
				{
					temp7 = (-2300 + cmd7)/((2400.0 - 2300.0)/(-85.0 - (-90.0))) + (-90);
				}
			}
			if (i == 7)
			{
				if (cmd8 < 760)
				{
					temp8 = (-760 + cmd8)/((760.0 - 525.0)/(0 - (-20.0))) + (0);
				}
				else
				{
					temp8 = (-760 + cmd8)/((2450.0 - 760.0)/(192.0 - (0))) + (0);
				}
			}
			if (i == 8)
			{
				if (cmd9 < 1290)
				{
					temp9 = (-1290 + cmd9)/((1290.0 - 960.0)/(-90.0 - (-30.0))) + (-90);
				}
				else
				{
					temp9 = (-1290 + cmd9)/((2380.0 - 1290.0)/(-210.0 - (-90.0))) + (-90);
				}
			}
			if (i == 9)
			{
				if (cmd10 < 1720)
				{
					temp10 = (-1720 + cmd10)/((1720.0 - 565.0)/(-90.0 - (30.0))) + (-90);
				}
				else
				{
					temp10 = (-1720 + cmd10)/((2015.0 - 1720.0)/(-120.0 - (-90.0))) + (-90);
				}
			}
			if (i == 10)
			{
				if (cmd11 < 1570)
				{
					temp11 = (-1570 + cmd11)/((1570.0 - 650.0)/(0 - (-60.0))) + (0);
				}
				else
				{
					temp11 = (-1570 + cmd11)/((2250.0 - 1625.0)/(85.0 - (0))) + (0);
				}
			}
			if (i == 11)
			{
				if (cmd12 > 1520)
				{
					temp12 = (-1520 + cmd12)/((2430.0 - 1520.0)/(180.0 - (90.0))) + (90);
				}
				else
				{
					temp12 = (-1520 + cmd12)/((1520.0 - 580.0)/(90.0 - (0.0))) + (90);
				}
			}
			if (i == 12)
			{
				if (cmd13 > 1940)
				{
					temp13 = (-1940 + cmd13)/((2255.0 - 1940.0)/(90.0 - (0.0))) + (0);
				}
				else
				{
					temp13 = (-1940 + cmd13)/((1940.0 - 1510.0)/(0.0 - (-129.0))) + (0);
				}
			}
			if (i == 13)
			{
				if (cmd14 < 2010)
				{
					temp14 = (-2010 + cmd14)/((2010.0 - 1670.0)/(-90.0 - (15.0))) + (-90);
				}
				else
				{
					temp14 = (-2010 + cmd14)/((2350.0 - 2010.0)/(-195.0 - (-90.0))) + (-90);
				}
			}
			if (i == 14)
			{
				if (cmd15 < 1240)
				{
					temp15 = (-1240 + cmd15)/((1240.0 - 1170.0)/(-90.0 - (-85.0))) + (-90);
				}
				else
				{
					temp15 = (-1240 + cmd15)/((2015.0 - 1240.0)/(-200.0 - (-90))) + (-90);
				}
			}
			if (i == 15)
			{
				if (cmd16 < 2200)
				{
					temp16 = (-2200 + cmd16)/((2200.0 - 560.0)/(0 - (192.0))) + (0);
				}
				else
				{
					temp16 = (-2200 + cmd16)/((2450.0 - 2200.0)/(-20.0 - (0))) + (0);
				}
			}
		}
		
		ang1 = degreeToRad(temp1);
		ang2 = degreeToRad(temp2);
		ang3 = degreeToRad(temp3);
		ang4 = degreeToRad(temp4);
		ang5 = degreeToRad(temp5);
		ang6 = degreeToRad(temp6);
		ang7 = degreeToRad(temp7);
		ang8 = degreeToRad(temp8);
		ang9 = degreeToRad(temp9);
		ang10 = degreeToRad(temp10);
		ang11 = degreeToRad(temp11);
		ang12 = degreeToRad(temp12);
		ang13 = degreeToRad(temp13);
		ang14 = degreeToRad(temp14);
		ang15 = degreeToRad(temp15);
		ang16 = degreeToRad(temp16);
	}
	
};		

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");

  
  drrobotarmbroadcaster _drrobotarmbroadcaster;

  _drrobotarmbroadcaster.sendPosition();
  
  return 0;
}
