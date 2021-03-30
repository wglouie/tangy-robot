//============================================================================
// Name        : sample_arm_movements.cpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Using code from DrRobotMotionArmDriver to drive random arm movements
//============================================================================
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "DrRobotMotionArmDriver.hpp"

	DrRobotMotionArmDriver::DrRobotMotionArmDriver()
	{
		_robotConfig = new DrRobotArmConfig;
		_robotConfig->robotID = "DrRobot";
		_robotConfig->robotIP = "127.0.0.1";
		_robotConfig->portNum = DEFAULT_PORT;

		client.sin_family = AF_INET;
		client.sin_port = htons(_robotConfig->portNum);
		client.sin_addr.s_addr = inet_addr(_robotConfig->robotIP.c_str());

		sockCheck = socket(AF_INET, SOCK_STREAM, 0);
		connectCheck = 0;
		msgCheck = 0;
	}

	DrRobotMotionArmDriver::~DrRobotMotionArmDriver()
	{

	}

	void DrRobotMotionArmDriver::close()
	{
		if (sockCheck > 0)
		    {
			  shutdown(sockCheck,SHUT_RDWR);
		      sockCheck = -1;
		    }
	}

	int DrRobotMotionArmDriver::vali_ip(const char*  ip_str)
	{
	  unsigned int n1,n2,n3,n4;
	  if ( sscanf(ip_str, "%u.%u.%u.%u", &n1,&n2,&n3,&n4) != 4 ) return 1;
	  if ((n1 != 0) && (n1 <= 255) && (n2 <= 255) && (n3 <= 255) && (n4 <= 255) )
	  {
	    char buf[64];
	    sprintf(buf,"%u.%u.%u.%u", n1,n2,n3,n4);
	    if (strcmp(buf,ip_str)) return 1;
	    return 0;
	  }

	  return 1;
	}
	
	int DrRobotMotionArmDriver::openConnection(std::string robotIP, int portNum)
	{

		//check the parameter first
		sockCheck = socket(AF_INET, SOCK_STREAM, 0);

		if(sockCheck == -1)
		{
			printf("Could not create socket\n");
			return 0;
		}

		printf("Socket created!\n");

		if (portNum <= 0)
		{
		  return -1;
		}

		if (vali_ip(robotIP.c_str()) == 1)
		{
		  printf("DrRobot Motion/Sensor Driver Error Message: invalid IP address: %s\n", robotIP.c_str());
		  return -2;
		}

		_robotConfig->portNum = portNum;
		_robotConfig->robotIP = robotIP;
		client.sin_family = AF_INET;
		client.sin_port = htons(_robotConfig->portNum);

		if (inet_addr(_robotConfig->robotIP.c_str()) < 0)
		{
			printf("DrRobot Motion/Sensor Driver Error Message: invalid IP address: %s\n", _robotConfig->robotIP.c_str());
			return -3;
		}
		else
		{
			client.sin_addr.s_addr = inet_addr(_robotConfig->robotIP.c_str());
		}
		
		printf("Connecting to %s(%d)...\n",_robotConfig->robotIP.c_str(),_robotConfig->portNum);
		connectCheck = connect(sockCheck , (struct sockaddr *)&client , sizeof(client));

		if(connectCheck < 0)
		{
			printf("Connection failed.\n");
			return -4;
		}

		printf("Connected!\n");

		return 1;

	}

	void DrRobotMotionArmDriver::getDrRobotArmDriverConfig(DrRobotArmConfig *driverConfig)
	{
		driverConfig->portNum = _robotConfig->portNum;
		driverConfig->robotID = _robotConfig->robotID;
		driverConfig->robotIP = _robotConfig->robotIP;
	}

	void DrRobotMotionArmDriver::setDrRobotArmDriverConfig(DrRobotArmConfig *driverConfig)
	{
		_robotConfig->portNum = driverConfig->portNum;
		_robotConfig->robotID = driverConfig->robotID;
		_robotConfig->robotIP = driverConfig->robotIP;
	}
	
	// Formula for transform angles in servo commands:
	// cmd = (intial cmd value) + (angle - initialAng)*(ratio)
	int DrRobotMotionArmDriver::sendServoCtrlAllAng(int ang1, int ang2, int ang3, int ang4, int ang5, int ang6, int ang7, int ang8, int ang9, int ang10, int ang11, int ang12, int ang13, int ang14, int ang15, int ang16, int vel, bool speed)
	{
		int i;
		float cmd1, cmd2, cmd3, cmd4, cmd5, cmd6, cmd7, cmd8, cmd9, cmd10, cmd11, cmd12, cmd13, cmd14, cmd15, cmd16;


		for (i = 0; i < 18; i++)
		{
			if (i == 0)
			{
				if (ang1 < -90)
				{
					cmd1 = 1700 + (ang1 - (-90))*((1700.0 - 590.0)/(-90.0 - (30.0)));
				}
				else
				{
					cmd1 = 1700 + (ang1 - (-90))*((2110.0 - 1700.0)/(-150.0 - (-90.0)));
				}
			}
			if (i == 1)
			{
				if (ang2 < -90)
				{
					cmd2 = 1269 + (ang2 - (-90))*((1269.0 - 950.0)/(-90.0 - (-60.0)));
				}
				else
				{
					cmd2 = 1269 + (ang2 - (-90))*((2400.0 - 1269.0)/(-210.0 - (-90.0)));
				}
			}
			if (i == 2)
			{
				if (ang3 < 0)
				{
					cmd3 = 1625 + (ang3 - (0))*((1625.0 - 680.0)/(0 - (-60.0)));
				}
				else
				{
					cmd3 = 1625 + (ang3 - (0))*((2250.0 - 1625.0)/(85.0 - (0)));
				}
			}
			if (i == 3)
			{
				if (ang4 < 90)
				{
					cmd4 = 1545 + (ang4 - (90))*((1545.0 - 580.0)/(90.0 - (0.0)));
				}
				else
				{
					cmd4 = 1545 + (ang4 - (90))*((2350.0 - 1545.0)/(180.0 - (90.0)));
				}
			}
			if (i == 4)
			{
				if (ang5 < 0)
				{
					cmd5 = 1710 + (ang5 - (0))*((1710.0 - 1400.0)/(0 - (-129.0)));
				}
				else
				{
					cmd5 = 1710 + (ang5 - (0))*((2110.0 - 1710.0)/(90.0 - (0)));
				}
			}
			if (i == 5)
			{
				if (ang6 < -90)
				{
					cmd6 = 2000 + (ang6 - (-90))*((2000.0 - 1670.0)/(-90.0 - (-195.0)));
				}
				else
				{
					cmd6 = 2000 + (ang6 - (-90))*((2350.0 - 2000.0)/(15.0 - (90.0)));
				}
			}
			if (i == 6)
			{
				if (ang7 < -90)
				{
					cmd7 = 2300 + (ang7 - (-90))*((2300.0 - 1460.0)/(-90.0 - (-200.0)));
				}
				else
				{
					cmd7 = 2300 + (ang7 - (-90))*((2400.0 - 2300.0)/(-85.0 - (-90.0)));
				}
			}
			if (i == 7)
			{
				if (ang8 < 0)
				{
					cmd8 = 760 + (ang8 - (0))*((760.0 - 525.0)/(0 - (-20.0)));
				}
				else
				{
					cmd8 = 760 + (ang8 - (0))*((2450.0 - 760.0)/(192.0 - (0)));
				}
			}
			if (i == 8)
			{
				if (ang9 > -90)
				{
					cmd9 = 1290 + (ang9 - (-90))*((1290.0 - 960.0)/(-90.0 - (-30.0)));
				}
				else
				{
					cmd9 = 1290 + (ang9 - (-90))*((2380.0 - 1290.0)/(-210.0 - (-90.0)));
				}
			}
			if (i == 9)
			{
				if (ang10 > -90)
				{
					cmd10 = 1720 + (ang10 - (-90))*((1720.0 - 565.0)/(-90.0 - (30.0)));
				}
				else
				{
					cmd10 = 1720 + (ang10 - (-90))*((2015.0 - 1720.0)/(-120.0 - (-90.0)));
				}
			}
			if (i == 10)
			{
				if (ang11 > 0)
				{
					cmd11 = 1570 + (ang11 - (0))*((1570.0 - 650.0)/(0 - (85.0)));
				}
				else
				{
					cmd11 = 1570 + (ang11 - (0))*((2250.0 - 1625.0)/(85.0 - (0)));
				}
			}
			if (i == 11)
			{
				if (ang12 > 90)
				{
					cmd12 = 1520 + (ang12 - (90))*((1520.0 - 580.0)/(90.0 - (180.0)));
				}
				else
				{
					cmd12 = 1520 + (ang12 - (90))*((2430.0 - 1520.0)/(0.0 - (90.0)));
				}
			}
			if (i == 12)
			{
				if (ang13 > 0)
				{
					cmd13 = 1940 + (ang13 - (0))*((1940.0 - 1510.0)/(0 - (90.0)));
				}
				else
				{
					cmd13 = 1940 + (ang13 - (0))*((2255.0 - 1940.0)/(-129.0 - (0)));
				}
			}
			if (i == 13)
			{
				if (ang14 > -90)
				{
					cmd14 = 2010 + (ang14 - (-90))*((2010.0 - 1670.0)/(-90.0 - (15.0)));
				}
				else
				{
					cmd14 = 2010 + (ang14 - (-90))*((2350.0 - 2010.0)/(-195.0 - (-90.0)));
				}
			}
			if (i == 14)
			{
				if (ang15 > -90)
				{
					cmd15 = 1240 + (ang15 - (-90))*((1240.0 - 1170.0)/(-90.0 - (-85.0)));
				}
				else
				{
					cmd15 = 1240 + (ang15 - (-90))*((2015.0 - 1240.0)/(-200.0 - (-90)));
				}
			}
			if (i == 15)
			{
				if (ang16 > 0)
				{
					cmd16 = 2200 + (ang16 - (0))*((2200.0 - 560.0)/(0 - (192.0)));
				}
				else
				{
					cmd16 = 2200 + (ang16 - (0))*((2450.0 - 2200.0)/(-20.0 - (0)));
				}
			}
		}

				return sendServoCtrlAllCmd(cmd1, cmd2, cmd3, cmd4, cmd5, cmd6, cmd7, cmd8, cmd9, cmd10, cmd11, cmd12, cmd13, cmd14, cmd15, cmd16, vel, speed);

	}

	int DrRobotMotionArmDriver::sendServoCtrlAllAng(int ang1, int ang2, int ang3, int ang4, int ang5, int ang6, int ang7, int ang8, int ang9, int ang10, int ang11, int ang12, int ang13, int ang14, int ang15, int ang16)
	{
		int i;
		float cmd1, cmd2, cmd3, cmd4, cmd5, cmd6, cmd7, cmd8, cmd9, cmd10, cmd11, cmd12, cmd13, cmd14, cmd15, cmd16;

		for (i = 0; i < 18; i++)
		{
			if (i == 0)
			{
				if (ang1 < -90)
				{
					cmd1 = 1700 + (ang1 - (-90))*((1700.0 - 590.0)/(-90.0 - (30.0)));
				}
				else
				{
					cmd1 = 1700 + (ang1 - (-90))*((2110.0 - 1700.0)/(-150.0 - (-90.0)));
				}
			}
			if (i == 1)
			{
				if (ang2 < -90)
				{
					cmd2 = 1269 + (ang2 - (-90))*((1269.0 - 950.0)/(-90.0 - (-60.0)));
				}
				else
				{
					cmd2 = 1269 + (ang2 - (-90))*((2400.0 - 1269.0)/(-210.0 - (-90.0)));
				}
			}
			if (i == 2)
			{
				if (ang3 < 0)
				{
					cmd3 = 1625 + (ang3 - (0))*((1625.0 - 680.0)/(0 - (-60.0)));
				}
				else
				{
					cmd3 = 1625 + (ang3 - (0))*((2250.0 - 1625.0)/(85.0 - (0)));
				}
			}
			if (i == 3)
			{
				if (ang4 < 90)
				{
					cmd4 = 1545 + (ang4 - (90))*((1545.0 - 580.0)/(90.0 - (0.0)));
				}
				else
				{
					cmd4 = 1545 + (ang4 - (90))*((2350.0 - 1545.0)/(180.0 - (90.0)));
				}
			}
			if (i == 4)
			{
				if (ang5 < 0)
				{
					cmd5 = 1710 + (ang5 - (0))*((1710.0 - 1400.0)/(0 - (-129.0)));
				}
				else
				{
					cmd5 = 1710 + (ang5 - (0))*((2110.0 - 1710.0)/(90.0 - (0)));
				}
			}
			if (i == 5)
			{
				if (ang6 < -90)
				{
					cmd6 = 2000 + (ang6 - (-90))*((2000.0 - 1670.0)/(-90.0 - (-195.0)));
				}
				else
				{
					cmd6 = 2000 + (ang6 - (-90))*((2350.0 - 2000.0)/(15.0 - (90.0)));
				}
			}
			if (i == 6)
			{
				if (ang7 < -90)
				{
					cmd7 = 2300 + (ang7 - (-90))*((2300.0 - 1460.0)/(-90.0 - (-200.0)));
				}
				else
				{
					cmd7 = 2300 + (ang7 - (-90))*((2400.0 - 2300.0)/(-85.0 - (-90.0)));
				}
			}
			if (i == 7)
			{
				if (ang8 < 0)
				{
					cmd8 = 760 + (ang8 - (0))*((760.0 - 525.0)/(0 - (-20.0)));
				}
				else
				{
					cmd8 = 760 + (ang8 - (0))*((2450.0 - 760.0)/(192.0 - (0)));
				}
			}
			if (i == 8)
			{
				if (ang9 > -90)
				{
					cmd9 = 1290 + (ang9 - (-90))*((1290.0 - 960.0)/(-90.0 - (-30.0)));
				}
				else
				{
					cmd9 = 1290 + (ang9 - (-90))*((2380.0 - 1290.0)/(-210.0 - (-90.0)));
				}
			}
			if (i == 9)
			{
				if (ang10 > -90)
				{
					cmd10 = 1720 + (ang10 - (-90))*((1720.0 - 565.0)/(-90.0 - (30.0)));
				}
				else
				{
					cmd10 = 1720 + (ang10 - (-90))*((2015.0 - 1720.0)/(-120.0 - (-90.0)));
				}
			}
			if (i == 10)
			{
				if (ang11 > 0)
				{
					cmd11 = 1570 + (ang11 - (0))*((1570.0 - 650.0)/(0 - (85.0)));
				}
				else
				{
					cmd11 = 1570 + (ang11 - (0))*((2250.0 - 1625.0)/(85.0 - (0)));
				}
			}
			if (i == 11)
			{
				if (ang12 > 90)
				{
					cmd12 = 1520 + (ang12 - (90))*((1520.0 - 580.0)/(90.0 - (180.0)));
				}
				else
				{
					cmd12 = 1520 + (ang12 - (90))*((2430.0 - 1520.0)/(0.0 - (90.0)));
				}
			}
			if (i == 12)
			{
				if (ang13 > 0)
				{
					cmd13 = 1940 + (ang13 - (0))*((1940.0 - 1510.0)/(0 - (90.0)));
				}
				else
				{
					cmd13 = 1940 + (ang13 - (0))*((2255.0 - 1940.0)/(-129.0 - (0)));
				}
			}
			if (i == 13)
			{
				if (ang14 > -90)
				{
					cmd14 = 2010 + (ang14 - (-90))*((2010.0 - 1670.0)/(-90.0 - (15.0)));
				}
				else
				{
					cmd14 = 2010 + (ang14 - (-90))*((2350.0 - 2010.0)/(-195.0 - (-90.0)));
				}
			}
			if (i == 14)
			{
				if (ang15 > -90)
				{
					cmd15 = 1240 + (ang15 - (-90))*((1240.0 - 1170.0)/(-90.0 - (-85.0)));
				}
				else
				{
					cmd15 = 1240 + (ang15 - (-90))*((2015.0 - 1240.0)/(-200.0 - (-90)));
				}
			}
			if (i == 15)
			{
				if (ang16 > 0)
				{
					cmd16 = 2200 + (ang16 - (0))*((2200.0 - 560.0)/(0 - (192.0)));
				}
				else
				{
					cmd16 = 2200 + (ang16 - (0))*((2450.0 - 2200.0)/(-20.0 - (0)));
				}
			}
		}

		return sendServoCtrlAllCmd(cmd1, cmd2, cmd3, cmd4, cmd5, cmd6, cmd7, cmd8, cmd9, cmd10, cmd11, cmd12, cmd13, cmd14, cmd15, cmd16);

	}


	//Servos channel starts with 0 and goes to 15, representing all 16 servos.
	//Each servo has it own angle limitation.
	int DrRobotMotionArmDriver::sendServoCtrlAllAng(int channel, int ang, int vel, bool speed)
	{
		float cmd;

		if (channel == 0)
		{
			if (ang < -90)
			{
				cmd = 1700 + (ang - (-90))*((1700.0 - 590.0)/(-90.0 - (30.0)));
			}
			else
			{
				cmd = 1700 + (ang - (-90))*((2110.0 - 1700.0)/(-150.0 - (-90.0)));
			}
		}
		if (channel == 1)
		{
			if (ang < -90)
			{
				cmd = 1269 + (ang - (-90))*((1269.0 - 950.0)/(-90.0 - (-60.0)));
			}
			else
			{
				cmd = 1269 + (ang - (-90))*((2400.0 - 1269.0)/(-210.0 - (-90.0)));
			}
		}
		if (channel == 2)
		{
			if (ang < 0)
			{
				cmd = 1625 + (ang - (0))*((1625.0 - 680.0)/(0 - (-60.0)));
			}
			else
			{
				cmd = 1625 + (ang - (0))*((2250.0 - 1625.0)/(85.0 - (0)));
			}
		}
		if (channel == 3)
		{
			if (ang < 90)
			{
				cmd = 1545 + (ang - (90))*((1545.0 - 580.0)/(90.0 - (0.0)));
			}
			else
			{
				cmd = 1545 + (ang - (90))*((2350.0 - 1545.0)/(180.0 - (90.0)));
			}
		}
		if (channel == 4)
		{
			if (ang < 0)
			{
				cmd = 1710 + (ang - (0))*((1710.0 - 1400.0)/(0 - (-129.0)));
			}
			else
			{
				cmd = 1710 + (ang - (0))*((2110.0 - 1710.0)/(90.0 - (0)));
			}
		}
		if (channel == 5)
		{
			if (ang < -90)
			{
				cmd = 2000 + (ang - (-90))*((2000.0 - 1670.0)/(-90.0 - (-195.0)));
			}
			else
			{
				cmd = 2000 + (ang - (-90))*((2350.0 - 2000.0)/(15.0 - (90.0)));
			}
		}
		if (channel == 6)
		{
			if (ang < -90)
			{
				cmd = 2300 + (ang - (-90))*((2300.0 - 1460.0)/(-90.0 - (-200.0)));
			}
			else
			{
				cmd = 2300 + (ang - (-90))*((2400.0 - 2300.0)/(-85.0 - (-90.0)));
			}
		}
		if (channel == 7)
		{
			if (ang < 0)
			{
				cmd = 760 + (ang - (0))*((760.0 - 525.0)/(0 - (-20.0)));
			}
			else
			{
				cmd = 760 + (ang - (0))*((2450.0 - 760.0)/(192.0 - (0)));
			}
		}
		if (channel == 8)
		{
			if (ang > -90)
			{
				cmd = 1290 + (ang - (-90))*((1290.0 - 960.0)/(-90.0 - (-30.0)));
			}
			else
			{
				cmd = 1290 + (ang - (-90))*((2380.0 - 1290.0)/(-210.0 - (-90.0)));
			}
		}
		if (channel == 9)
		{
			if (ang > -90)
			{
				cmd = 1720 + (ang - (-90))*((1720.0 - 565.0)/(-90.0 - (30.0)));
			}
			else
			{
				cmd = 1720 + (ang - (-90))*((2015.0 - 1720.0)/(-120.0 - (-90.0)));
			}
		}
		if (channel == 10)
		{
			if (ang > 0)
			{
				cmd = 1570 + (ang - (0))*((1570.0 - 650.0)/(0 - (85.0)));
			}
			else
			{
				cmd = 1570 + (ang - (0))*((2250.0 - 1625.0)/(85.0 - (0)));
			}
		}
		if (channel == 11)
		{
			if (ang > 90)
			{
				cmd = 1520 + (ang - (90))*((1520.0 - 580.0)/(90.0 - (180.0)));
			}
			else
			{
				cmd = 1520 + (ang - (90))*((2430.0 - 1520.0)/(0.0 - (90.0)));
			}
		}
		if (channel == 12)
		{
			if (ang > 0)
			{
				cmd = 1940 + (ang - (0))*((1940.0 - 1510.0)/(0 - (90.0)));
			}
    		else
	     	{
				cmd = 1940 + (ang - (0))*((2255.0 - 1940.0)/(-129.0 - (0)));
			}
		}
		if (channel == 13)
		{
			if (ang > -90)
			{
				cmd = 2010 + (ang - (-90))*((2010.0 - 1670.0)/(-90.0 - (15.0)));
			}
			else
			{
				cmd = 2010 + (ang - (-90))*((2350.0 - 2010.0)/(-195.0 - (-90.0)));
			}
		}
		if (channel == 14)
		{
			if (ang > -90)
			{
				cmd = 1240 + (ang - (-90))*((1240.0 - 1170.0)/(-90.0 - (-85.0)));
			}
			else
			{
				cmd = 1240 + (ang - (-90))*((2015.0 - 1240.0)/(-200.0 - (-90)));
			}
		}
		if (channel == 15)
		{
			if (ang > 0)
			{
				cmd = 2200 + (ang - (0))*((2200.0 - 560.0)/(0 - (192.0)));
			}
			else
			{
				cmd = 2200 + (ang - (0))*((2450.0 - 2200.0)/(-20.0 - (0)));
			}
		}

		return sendServoCtrlAllCmd(channel,cmd,vel, speed);
	}


	//Servos channel starts with 0 and goes to 15, representing all 16 servos.
	//Each servo has it own angle limitation.
	int DrRobotMotionArmDriver::sendServoCtrlAllAng(int channel, int ang)
	{
		float cmd;
		if (channel == 0)
		{
			if (ang < -90)
			{
				cmd = 1700 + (ang - (-90))*((1700.0 - 590.0)/(-90.0 - (30.0)));
			}
			else
			{
				cmd = 1700 + (ang - (-90))*((2110.0 - 1700.0)/(-150.0 - (-90.0)));
			}
		}
		if (channel == 1)
		{
			if (ang < -90)
			{
				cmd = 1269 + (ang - (-90))*((1269.0 - 950.0)/(-90.0 - (-60.0)));
			}
			else
			{
				cmd = 1269 + (ang - (-90))*((2400.0 - 1269.0)/(-210.0 - (-90.0)));
			}
		}
		if (channel == 2)
		{
			if (ang < 0)
			{
				cmd = 1625 + (ang - (0))*((1625.0 - 680.0)/(0 - (-60.0)));
			}
			else
			{
				cmd = 1625 + (ang - (0))*((2250.0 - 1625.0)/(85.0 - (0)));
			}
		}
		if (channel == 3)
		{
			if (ang < 90)
			{
				cmd = 1545 + (ang - (90))*((1545.0 - 580.0)/(90.0 - (0.0)));
			}
			else
			{
				cmd = 1545 + (ang - (90))*((2350.0 - 1545.0)/(180.0 - (90.0)));
			}
		}
		if (channel == 4)
		{
			if (ang < 0)
			{
				cmd = 1710 + (ang - (0))*((1710.0 - 1400.0)/(0 - (-129.0)));
			}
			else
			{
				cmd = 1710 + (ang - (0))*((2110.0 - 1710.0)/(90.0 - (0)));
			}
		}
		if (channel == 5)
		{
			if (ang < -90)
			{
				cmd = 2000 + (ang - (-90))*((2000.0 - 1670.0)/(-90.0 - (-195.0)));
			}
			else
			{
				cmd = 2000 + (ang - (-90))*((2350.0 - 2000.0)/(15.0 - (90.0)));
			}
		}
		if (channel == 6)
		{
			if (ang < -90)
			{
				cmd = 2300 + (ang - (-90))*((2300.0 - 1460.0)/(-90.0 - (-200.0)));
			}
			else
			{
				cmd = 2300 + (ang - (-90))*((2400.0 - 2300.0)/(-85.0 - (-90.0)));
			}
		}
		if (channel == 7)
		{
			if (ang < 0)
			{
				cmd = 760 + (ang - (0))*((760.0 - 525.0)/(0 - (-20.0)));
			}
			else
			{
				cmd = 760 + (ang - (0))*((2450.0 - 760.0)/(192.0 - (0)));
			}
		}
		if (channel == 8)
		{
			if (ang > -90)
			{
				cmd = 1290 + (ang - (-90))*((1290.0 - 960.0)/(-90.0 - (-30.0)));
			}
			else
			{
				cmd = 1290 + (ang - (-90))*((2380.0 - 1290.0)/(-210.0 - (-90.0)));
			}
		}
		if (channel == 9)
		{
			if (ang > -90)
			{
				cmd = 1720 + (ang - (-90))*((1720.0 - 565.0)/(-90.0 - (30.0)));
			}
			else
			{
				cmd = 1720 + (ang - (-90))*((2015.0 - 1720.0)/(-120.0 - (-90.0)));
			}
		}
		if (channel == 10)
		{
			if (ang > 0)
			{
				cmd = 1570 + (ang - (0))*((1570.0 - 650.0)/(0 - (85.0)));
			}
			else
			{
				cmd = 1570 + (ang - (0))*((2250.0 - 1625.0)/(85.0 - (0)));
			}
		}
		if (channel == 11)
		{
			if (ang > 90)
			{
				cmd = 1520 + (ang - (90))*((1520.0 - 580.0)/(90.0 - (180.0)));
			}
			else
			{
				cmd = 1520 + (ang - (90))*((2430.0 - 1520.0)/(0.0 - (90.0)));
			}
		}
		if (channel == 12)
		{
			if (ang > 0)
			{
				cmd = 1940 + (ang - (0))*((1940.0 - 1510.0)/(0 - (90.0)));
			}
    		else
	     	{
				cmd = 1940 + (ang - (0))*((2255.0 - 1940.0)/(-129.0 - (0)));
			}
		}
		if (channel == 13)
		{
			if (ang > -90)
			{
				cmd = 2010 + (ang - (-90))*((2010.0 - 1670.0)/(-90.0 - (15.0)));
			}
			else
			{
				cmd = 2010 + (ang - (-90))*((2350.0 - 2010.0)/(-195.0 - (-90.0)));
			}
		}
		if (channel == 14)
		{
			if (ang > -90)
			{
				cmd = 1240 + (ang - (-90))*((1240.0 - 1170.0)/(-90.0 - (-85.0)));
			}
			else
			{
				cmd = 1240 + (ang - (-90))*((2015.0 - 1240.0)/(-200.0 - (-90)));
			}
		}
		if (channel == 15)
		{
			if (ang > 0)
			{
				cmd = 2200 + (ang - (0))*((2200.0 - 560.0)/(0 - (192.0)));
			}
			else
			{
				cmd = 2200 + (ang - (0))*((2450.0 - 2200.0)/(-20.0 - (0)));
			}
		}
		return sendServoCtrlAllCmd(channel,cmd);
	}

	int DrRobotMotionArmDriver::sendServoCtrlAllCmd(int cmd1, int cmd2, int cmd3, int cmd4, int cmd5, int cmd6, int cmd7, int cmd8, int cmd9, int cmd10, int cmd11, int cmd12, int cmd13, int cmd14, int cmd15, int cmd16, int vel, bool speed)
	{
		int poses[18];
		int i;
		
		if (cmd1 != NOCONTROL)
		{
			if (cmd1 > 2110) cmd1 = 2110;
			if (cmd1 < 590) cmd1 = 590;
		}
		if (cmd2 != NOCONTROL)
		{
			if (cmd2 > 2400) cmd2 = 2400;
			if (cmd2 < 950) cmd2 = 950;
		}
		if (cmd3 != NOCONTROL)
		{
			if (cmd3 > 2250) cmd3 = 2250;
			if (cmd3 < 680) cmd3 = 680;
		}
		if (cmd4 != NOCONTROL)
		{
			if (cmd4 > 2350) cmd4 = 2350;
			if (cmd4 < 580) cmd4 = 580;
		}
		if (cmd5 != NOCONTROL)
		{
			if (cmd5 > 2110) cmd5 = 2110;
			if (cmd5 < 1400) cmd5 = 1400;
		}
		if (cmd6 != NOCONTROL)
		{
			if (cmd6 > 2350) cmd6 = 2350;
			if (cmd6 < 1670) cmd6 = 1670;
		}
		if (cmd7 != NOCONTROL)
		{
			if (cmd7 > 2400) cmd7 = 2400;
			if (cmd7 < 1460) cmd7 = 1460;
		}
		if (cmd8 != NOCONTROL)
		{
			if (cmd8 > 2450) cmd8 = 2450;
			if (cmd8 < 525) cmd8 = 525;
		}
		if (cmd9 != NOCONTROL)
		{
			if (cmd9 > 2380) cmd9 = 2380;
			if (cmd9 < 960) cmd9 = 960;
		}
		if (cmd10 != NOCONTROL)
		{
			if (cmd10 > 2015) cmd10 = 2015;
			if (cmd10 < 565) cmd10 = 565;
		}
		if (cmd11 != NOCONTROL)
		{	
			if (cmd11 > 2200) cmd11 = 2200;
			if (cmd11 < 650) cmd11 = 650;
		}
		if (cmd12 != NOCONTROL)
		{
			if (cmd12 > 2430) cmd12 = 2430;
			if (cmd12 < 580) cmd12 = 580;
		}
		if (cmd13 != NOCONTROL)
		{
			if (cmd13 > 2255) cmd13 = 2255;
			if (cmd13 < 1510) cmd13 = 1510;
		}
		if (cmd14 != NOCONTROL)
		{
			if (cmd14 > 2350) cmd14 = 2350;
			if (cmd14 < 1670) cmd14 = 1670;
		}
		if (cmd15 != NOCONTROL)
		{
			if (cmd15 > 2015) cmd15 = 2015;
			if (cmd15 < 1170) cmd15 = 1170;
		}
		if (cmd16 != NOCONTROL)
		{
			if (cmd16 > 2450) cmd16 = 2450;
			if (cmd16 < 560) cmd16 = 560;
		}
		if (vel <= 0) vel = 1;
		

		for (i = 0; i < 18; i++)
		{
			if (i == 0)
			{
				poses[i] = cmd1;
			}
			if (i == 1)
			{
				poses[i] = cmd2;
			}
			if (i == 2)
			{
				poses[i] = cmd3;
			}
			if (i == 3)
			{
				poses[i] = cmd4;
			}
			if (i == 4)
			{
				poses[i] = cmd5;
			}
			if (i == 5)
			{
				poses[i] = cmd6;
			}
			if (i == 6)
			{
				poses[i] = cmd7;
			}
			if (i == 7)
			{
				poses[i] = cmd8;
			}
			if (i == 8)
			{
				poses[i] = cmd9;
			}
			if (i == 9)
			{
				poses[i] = cmd10;
			}
			if (i == 10)
			{
				poses[i] = cmd11;
			}
			if (i == 11)
			{
				poses[i] = cmd12;
			}
			if (i == 12)
			{
				poses[i] = cmd13;
			}
			if (i == 13)
			{
				poses[i] = cmd14;
			}
			if (i == 14)
			{
				poses[i] = cmd15;
			}
			if (i == 15)
			{
				poses[i] = cmd16;
			}
			if (i == 16)
			{
				poses[i] = vel;
			}
			if (i == 17)
			{
				if (speed == true) poses[i] = 1;
				else poses[i] = 0;
			}
		}

		return sendCommand(poses, i, 1);
	}

	int DrRobotMotionArmDriver::sendServoCtrlAllCmd(int cmd1, int cmd2, int cmd3, int cmd4, int cmd5, int cmd6, int cmd7, int cmd8, int cmd9, int cmd10, int cmd11, int cmd12, int cmd13, int cmd14, int cmd15, int cmd16)
	{
		int poses[16];
		int i;

		if (cmd1 != NOCONTROL)
		{
			if (cmd1 > 2110) cmd1 = 2110;
			if (cmd1 < 590) cmd1 = 590;
		}
		if (cmd2 != NOCONTROL)
		{
			if (cmd2 > 2400) cmd2 = 2400;
			if (cmd2 < 950) cmd2 = 950;
		}
		if (cmd3 != NOCONTROL)
		{
			if (cmd3 > 2250) cmd3 = 2250;
			if (cmd3 < 680) cmd3 = 680;
		}
		if (cmd4 != NOCONTROL)
		{
			if (cmd4 > 2350) cmd4 = 2350;
			if (cmd4 < 580) cmd4 = 580;
		}
		if (cmd5 != NOCONTROL)
		{
			if (cmd5 > 2110) cmd5 = 2110;
			if (cmd5 < 1400) cmd5 = 1400;
		}
		if (cmd6 != NOCONTROL)
		{
			if (cmd6 > 2350) cmd6 = 2350;
			if (cmd6 < 1670) cmd6 = 1670;
		}
		if (cmd7 != NOCONTROL)
		{
			if (cmd7 > 2400) cmd7 = 2400;
			if (cmd7 < 1460) cmd7 = 1460;
		}
		if (cmd8 != NOCONTROL)
		{
			if (cmd8 > 2450) cmd8 = 2450;
			if (cmd8 < 525) cmd8 = 525;
		}
		if (cmd9 != NOCONTROL)
		{
			if (cmd9 > 2380) cmd9 = 2380;
			if (cmd9 < 960) cmd9 = 960;
		}
		if (cmd10 != NOCONTROL)
		{
			if (cmd10 > 2015) cmd10 = 2015;
			if (cmd10 < 565) cmd10 = 565;
		}
		if (cmd11 != NOCONTROL)
		{	
			if (cmd11 > 2200) cmd11 = 2200;
			if (cmd11 < 650) cmd11 = 650;
		}
		if (cmd12 != NOCONTROL)
		{
			if (cmd12 > 2430) cmd12 = 2430;
			if (cmd12 < 580) cmd12 = 580;
		}
		if (cmd13 != NOCONTROL)
		{
			if (cmd13 > 2255) cmd13 = 2255;
			if (cmd13 < 1510) cmd13 = 1510;
		}
		if (cmd14 != NOCONTROL)
		{
			if (cmd14 > 2350) cmd14 = 2350;
			if (cmd14 < 1670) cmd14 = 1670;
		}
		if (cmd15 != NOCONTROL)
		{
			if (cmd15 > 2015) cmd15 = 2015;
			if (cmd15 < 1170) cmd15 = 1170;
		}
		if (cmd16 != NOCONTROL)
		{
			if (cmd16 > 2450) cmd16 = 2450;
			if (cmd16 < 560) cmd16 = 560;
		}

		for (i = 0; i < 16; i++)
		{
			if (i == 0)
			{
				poses[i] = cmd1;
			}
			if (i == 1)
			{
				poses[i] = cmd2;
			}
			if (i == 2)
			{
				poses[i] = cmd3;
			}
			if (i == 3)
			{
				poses[i] = cmd4;
			}
			if (i == 4)
			{
				poses[i] = cmd5;
			}
			if (i == 5)
			{
				poses[i] = cmd6;
			}
			if (i == 6)
			{
				poses[i] = cmd7;
			}
			if (i == 7)
			{
				poses[i] = cmd8;
			}
			if (i == 8)
			{
				poses[i] = cmd9;
			}
			if (i == 9)
			{
				poses[i] = cmd10;
			}
			if (i == 10)
			{
				poses[i] = cmd11;
			}
			if (i == 11)
			{
				poses[i] = cmd12;
			}
			if (i == 12)
			{
				poses[i] = cmd13;
			}
			if (i == 13)
			{
				poses[i] = cmd14;
			}
			if (i == 14)
			{
				poses[i] = cmd15;
			}
			if (i == 15)
			{
				poses[i] = cmd16;
			}
		}

		return sendCommand(poses, i, 2);
	}
	
	
	//Channel goes from 1 to 16 representing the sixteen servos
	int DrRobotMotionArmDriver::sendServoCtrlAllCmd(int channel, int cmd, int vel, bool speed)
	{
		int poses[18];
		int i;
		
		if (cmd != NOCONTROL)
		{
			if (channel == 1)
			{
				if (cmd > 2110) cmd = 2110;
				if (cmd < 590) cmd = 590;
			}
			if (channel == 2)
			{
				if (cmd > 2400) cmd = 2400;
				if (cmd < 950) cmd = 950;
			}
			if (channel == 3)
			{
				if (cmd > 2250) cmd = 2250;
				if (cmd < 680) cmd = 680;
			}
			if (channel == 4)
			{
				if (cmd > 2350) cmd = 2350;
				if (cmd < 580) cmd = 580;
			}
			if (channel == 5)
			{
				if (cmd > 2110) cmd = 2110;
				if (cmd < 1400) cmd = 1400;
			}
			if (channel == 6)
			{
				if (cmd > 2350) cmd = 2350;
				if (cmd < 1670) cmd = 1670;
			}
			if (channel == 7)
			{
				if (cmd > 2400) cmd = 2400;
				if (cmd < 1460) cmd = 1460;
			}
			if (channel == 8)
			{
				if (cmd > 2450) cmd = 2450;
				if (cmd < 525) cmd = 525;
			}
			if (channel == 9)
			{
				if (cmd > 2380) cmd = 2380;
				if (cmd < 960) cmd = 960;
			}
			if (channel == 10)
			{
				if (cmd > 2015) cmd = 2015;
				if (cmd < 565) cmd = 565;
			}
			if (channel == 11)
			{	
				if (cmd > 2200) cmd = 2200;
				if (cmd < 650) cmd = 650;
			}
			if (channel == 12)
			{
				if (cmd > 2430) cmd = 2430;
				if (cmd < 580) cmd = 580;
			}
			if (channel == 13)
			{
				if (cmd > 2255) cmd = 2255;
				if (cmd < 1510) cmd = 1510;
			}
			if (channel == 14)
			{
				if (cmd > 2350) cmd = 2350;
				if (cmd < 1670) cmd = 1670;
			}
			if (channel == 15)
			{
				if (cmd > 2015) cmd = 2015;
				if (cmd < 1170) cmd = 1170;
			}
			if (channel == 16)
			{
				if (cmd > 2450) cmd = 2450;
				if (cmd < 560) cmd = 560;
			}

		}
		if (vel <= 0) vel = 1;
		
		for (i = 0; i < 18; i++)
		{
			if (i == channel) poses[i] = cmd;
			else poses[i] = NOCONTROL;

			if (i == 16)
			{
				poses[i] = vel;
			}
			if (i == 17)
			{
				if (speed == true) poses[i] = 1;
				else poses[i] = 0;
			}
		}

		return sendCommand(poses, i, 1);
	}

	int DrRobotMotionArmDriver::sendServoCtrlAllCmd(int channel, int cmd)
	{
		int poses[16];
		int i;
		int homeposes[16]={1700, 1269, 1625, 1545, 1710, 2000, 2300, 760, 1290, 1720, 1570, 1520, 1940, 2010, 1240, 2200};

		if (cmd != NOCONTROL)
		{
			if (channel == 1)
			{
				if (cmd > 2110) cmd = 2110;
				if (cmd < 590) cmd = 590;
			}
			if (channel == 2)
			{
				if (cmd > 2400) cmd = 2400;
				if (cmd < 950) cmd = 950;
			}
			if (channel == 3)
			{
				if (cmd > 2250) cmd = 2250;
				if (cmd < 680) cmd = 680;
			}
			if (channel == 4)
			{
				if (cmd > 2350) cmd = 2350;
				if (cmd < 580) cmd = 580;
			}
			if (channel == 5)
			{
				if (cmd > 2110) cmd = 2110;
				if (cmd < 1400) cmd = 1400;
			}
			if (channel == 6)
			{
				if (cmd > 2350) cmd = 2350;
				if (cmd < 1670) cmd = 1670;
			}
			if (channel == 7)
			{
				if (cmd > 2400) cmd = 2400;
				if (cmd < 1460) cmd = 1460;
			}
			if (channel == 8)
			{
				if (cmd > 2450) cmd = 2450;
				if (cmd < 525) cmd = 525;
			}
			if (channel == 9)
			{
				if (cmd > 2380) cmd = 2380;
				if (cmd < 960) cmd = 960;
			}
			if (channel == 10)
			{
				if (cmd > 2015) cmd = 2015;
				if (cmd < 565) cmd = 565;
			}
			if (channel == 11)
			{	
				if (cmd > 2200) cmd = 2200;
				if (cmd < 650) cmd = 650;
			}
			if (channel == 12)
			{
				if (cmd > 2430) cmd = 2430;
				if (cmd < 580) cmd = 580;
			}
			if (channel == 13)
			{
				if (cmd > 2255) cmd = 2255;
				if (cmd < 1510) cmd = 1510;
			}
			if (channel == 14)
			{
				if (cmd > 2350) cmd = 2350;
				if (cmd < 1670) cmd = 1670;
			}
			if (channel == 15)
			{
				if (cmd > 2015) cmd = 2015;
				if (cmd < 1170) cmd = 1170;
			}
			if (channel == 16)
			{
				if (cmd > 2450) cmd = 2450;
				if (cmd < 560) cmd = 560;
			}

		}

		for (i = 0; i < 16; i++)
		{
			if (i == channel) poses[i] = cmd;
			else poses[i] = homeposes[i];
		}

		return sendCommand(poses, i, 2);
	}
	

	//This function send a char with the commands to the controller, it return -1 if fails.
	//It receive a vector of positions, the vector length and the mode, which is the type of control the user wants.
	//Could be:
	//mode1: Send commands with time or speed setup
	//mode2: Send commands with time default
	int DrRobotMotionArmDriver::sendCommand(int poses[], int vecLen, int mode)
	{
		std::string message;
		int servo_id, temp;

		if (mode == 1)
		{

			for (servo_id = 0; servo_id < (vecLen - 2); servo_id++)
			{
				if (poses[servo_id] != NOCONTROL)
				{
					temp = servo_id;
					
					if(servo_id > 7)
					{ 
						temp = temp + SERVO_CHANNEL_OFFSET;
						message += "#" + to_string(temp) + " P" + to_string(poses[servo_id]) + " ";
					}
					else
					{
						message += "#" + to_string(servo_id) + " P" + to_string(poses[servo_id]) + " ";
					}
					
				}
			}

			if(poses[vecLen-1] == 0) message += "T" + to_string(poses[vecLen-2]) + '\r';
			
			if(poses[vecLen-1] == 1) message += "S" + to_string(poses[vecLen-2]) + '\r';

		}

		if (mode == 2)
		{
			for (servo_id = 0; servo_id < (vecLen); servo_id++)
			{
				if (poses[servo_id] != NOCONTROL)
				{
					temp = servo_id;
					
					if(servo_id > 7)
					{ 
						temp = temp + SERVO_CHANNEL_OFFSET;
						message += "#" + to_string(temp) + " P" + to_string(poses[servo_id]) + " ";
					}
					else
					{
						message += "#" + to_string(servo_id) + " P" + to_string(poses[servo_id]) + " ";
					}
				}
			}
			message += "T1000\r";
		}
		
		ROS_INFO("Sending commands...");
		ROS_INFO("right_arm [%d %d %d %d %d %d %d %d]", poses[0], poses[1], poses[2], poses[3], poses[4], poses[5], poses[6], poses[7]);
		ROS_INFO("left_arm [%d %d %d %d %d %d %d %d]",poses[8], poses[9], poses[10], poses[11], poses[12], poses[13], poses[14], poses[15]);
		if (mode == 1) ROS_INFO("Time/Speed [%d]\n", poses[16]);
		else ROS_INFO("Time/Speed [1000]\n");
		
		msgCheck = send(sockCheck, message.c_str(), strlen(message.c_str()), 0);

		if (msgCheck < 0)
		{
			puts("Send failed");
			return -1;
		}
		
		//checkStatus();
		
		return 0;
	}
	
	int DrRobotMotionArmDriver::checkStatus()
	{
		char feedBack[10];
		int queue;

		client_len = sizeof(struct sockaddr_in);
		
		if (sockCheck < 0) 
		{
			puts("Create socket first.\n");
			return -1;
		}
		if (connectCheck < 0) 
		{
			puts("You are not connect to the server...");
			return -2;
		}
		
		puts("Waiting for movements...");
		
		while(1)
		{
			queue = send(sockCheck, "q", 1, 0);
			if (queue < 0)
			{
				puts("Can not send messages.");
				return -3;
			}
			puts("here");
					
			if( recv(sockCheck , feedBack , 10 , MSG_OOB) < 0) 
			{
				puts("Can not receive messages!");
				return -4;
			}
			puts("here too");
					
			if(feedBack == "+") puts("Action in progress, still moving...");
			else
			{
				puts("Action finished!");
				break;
			}
		}
		
		return 0;
	}
	

int main(int argc , char *argv[])
{
  ros::init(argc, argv, "Trajectory_Broadcaster");
  ros::NodeHandle n;
  ros::Publisher traj_broadcaster=n.advertise<trajectory_msgs::JointTrajectory>("joint_traj",1000);
  ros::Rate loop_rate(10);
  
	printf("STARTING SAMPLE MOVEMENTS\n");
	DrRobotMotionArmDriver client;
	std::string ip = "192.168.0.94";
	
	client.openConnection(ip,10001);
	
	trajectory_msgs::JointTrajectory goal;
	goal.header.stamp = ros::Time::now() + ros::Duration(1.0);
  goal.joint_names.push_back("r_arm_controller_5");
  goal.points.resize(3);
  goal.points[0].positions.resize(1);
  goal.points[0].velocities.resize(1);
  goal.points[1].positions.resize(1);
  goal.points[1].velocities.resize(1);
  goal.points[2].positions.resize(1);
  goal.points[2].velocities.resize(1);
  int i=0;
  while(ros::ok()){
  	client.sendServoCtrlAllCmd(6,1460);
  	goal.points[0].positions[0]=1670;
  	goal.points[0].velocities[0]=1;
  	goal.points[0].time_from_start=ros::Duration(1.0)+ros::Duration(i);
  	traj_broadcaster.publish(goal);
  	loop_rate.sleep();
  	sleep(2);
  	i=i+3;
  	client.sendServoCtrlAllCmd(6,1800);
  	goal.points[1].positions[0]=2000;
  	goal.points[1].velocities[0]=2;
  	goal.points[1].time_from_start=ros::Duration(1.0)+ros::Duration(i);
  	traj_broadcaster.publish(goal);
  	loop_rate.sleep();
    sleep(2);
    i=i+3;
  	client.sendServoCtrlAllCmd(6,2300);
  	goal.points[2].positions[0]=2300;
  	goal.points[2].velocities[0]=3;
  	goal.points[2].time_from_start=ros::Duration(1.0)+ros::Duration(i);
  	traj_broadcaster.publish(goal);
  	loop_rate.sleep();
  	sleep(2);
  	i=i+3;
  	
  }

	printf("FINISHED SAMPLE MOVEMENTS\n");
	return 0;
}
