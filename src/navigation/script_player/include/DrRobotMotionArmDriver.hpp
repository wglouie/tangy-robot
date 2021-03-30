//============================================================================
// Name        : DrRobotMotionArmDriver.hpp
// Author      : Ananias Paiva
// Version     : 1.0
// Copyright   :
// Description : Arm motion driver
//============================================================================

#ifndef DRROBOTMOTIONARMDRIVER_HPP_
#define DRROBOTMOTIONARMDRIVER_HPP_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>    //strlen
#include <sys/socket.h>    //socket
#include <arpa/inet.h> //inet_addr
#include <netinet/in.h>
#include <unistd.h>    //write
#include <sys/time.h>
#include <sys/types.h>
#include <string>
#include <iostream>
#include <sstream>
#include <ros/ros.h>

#define DEFAULT_PORT 8888
#define NOCONTROL -32768
#define SERVO_CHANNEL_OFFSET 8

using namespace std;

class DrRobotMotionArmDriver
{
public:
	struct DrRobotArmConfig
		{
			std::string robotID;
			std::string robotIP;
			int portNum;
		};
	
	DrRobotMotionArmDriver();
	~DrRobotMotionArmDriver();
	
	void close();
	
	int vali_ip(const char*  ip_str);
	
	int openConnection(std::string robotIP, int portNum);
	
	void getDrRobotArmDriverConfig(DrRobotArmConfig *driverConfig);
	
	void setDrRobotArmDriverConfig(DrRobotArmConfig *driverConfig);
	
	int sendServoCtrlAllAng(int ang1, int ang2, int ang3, int ang4, int ang5, int ang6, int ang7, int ang8, int ang9, int ang10, int ang11, int ang12, int ang13, int ang14, int ang15, int ang16, int vel, bool speed);
	int sendServoCtrlAllAng(int ang1, int ang2, int ang3, int ang4, int ang5, int ang6, int ang7, int ang8, int ang9, int ang10, int ang11, int ang12, int ang13, int ang14, int ang15, int ang16);
	int sendServoCtrlAllAng(int channel, int ang, int vel, bool speed);
	int sendServoCtrlAllAng(int channel, int ang);
	
	int sendServoCtrlAllCmd(int cmd1, int cmd2, int cmd3, int cmd4, int cmd5, int cmd6, int cmd7, int cmd8, int cmd9, int cmd10, int cmd11, int cmd12, int cmd13, int cmd14, int cmd15, int cmd16, int vel, bool speed);
	int sendServoCtrlAllCmd(int cmd1, int cmd2, int cmd3, int cmd4, int cmd5, int cmd6, int cmd7, int cmd8, int cmd9, int cmd10, int cmd11, int cmd12, int cmd13, int cmd14, int cmd15, int cmd16);
	int sendServoCtrlAllCmd(int channel, int cmd, int vel, bool speed);
	int sendServoCtrlAllCmd(int channel, int cmd);
	
	int sendCommand(int poses[], int vecLen, int mode);
	
	int checkStatus();
	
private:
	DrRobotArmConfig *_robotConfig;
	struct sockaddr_in client;
	int sockCheck;
	int connectCheck;
	int msgCheck;
	socklen_t client_len;
	
	std::string to_string(int value)
	{
		stringstream ss;
		ss << value;
		return ss.str();
	}
};

#endif
