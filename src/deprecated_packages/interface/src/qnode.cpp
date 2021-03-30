/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/interface/qnode.hpp"

//---- ANANIAS' LIBRARIES ---------------

//#include <termios.h>
//#include <signal.h>
//#include <math.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <sys/poll.h>


//#include <boost/thread/thread.hpp>
//#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <drrobot_h20_player/HeadCmd.h>

//#include "drrobot_script_teleop.hpp"
//#include "drrobot_head_script.hpp"

//---- /ANANIAS' LIBRARIES ---------------

#define key_W   1
#define key_A   2
#define key_S   3
#define key_D   4
#define key_I   5
#define key_J   6
#define key_K   7
#define key_L   8
#define key_C   9
#define key_V   10

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace interface {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"interface");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
        n.getParam("/script_folder", script_folder);


	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    pub_head = n.advertise<drrobot_h20_player::HeadCmd>("cmd_head", 1);
    pub_base = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //xmlFile_head = "/home/nikiminha/ros/tangy/src/navigation/drrobot_h20_arm_player/script/Pose_head.xml";
    //xmlFile_arm = "/home/nikiminha/ros/tangy/src/navigation/drrobot_h20_arm_player/script/Pose_arm.xml";

    double maxVel = 1.0;
    double maxTurn = 1.0;
    int neck_x_rotation_ = 0;
    int neck_z_rotation_= 0;
    int mouth_= 0;
    int upper_head_ = 0;
    int left_eye_ = 0;
    int right_eye_ = 0;
    int flag_ = 0;
    bool dirty = false;

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"interface");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    n.getParam("/script_folder", script_folder);


	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

    // ANANIAS CODE
    pub_head = n.advertise<drrobot_h20_player::HeadCmd>("cmd_head", 1);
    pub_base = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //xmlFile_head = "/home/nikiminha/ros/tangy/src/navigation/drrobot_h20_arm_player/script/Pose_head.xml";
    //xmlFile_arm = "/home/nikiminha/ros/tangy/src/navigation/drrobot_h20_arm_player/script/Pose_arm.xml";
    // /code ANANIAS CODE

    start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

        //std_msgs::String msg;
        //std::stringstream ss;
        //ss << "hello world " << count;
        //msg.data = ss.str();
        //chatter_publisher.publish(msg);
        //log(Info,std::string("I sent: ")+msg.data);
        //ros::spinOnce();
        //loop_rate.sleep();
        //++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::publish(int command){
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Key pressed: ";

    double maxVel = 1.0;
    double maxTurn = 1.0;
    int neck_x_rotation_ = 0;
    int neck_z_rotation_= 0;
    int mouth_= 0;
    int upper_head_ = 0;
    int left_eye_ = 0;
    int right_eye_ = 0;
    int flag_ = 0;
    bool dirty = false;


    switch(command){
        case key_W:
            ss << "W";

            ///*ANANIAS
                maxVel = 0.2;
                maxTurn = 0;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
            //*/

            break;
        case key_A:
            ss << "A";

            ///*ANANIAS
                maxVel = 0;
                maxTurn = 0.4;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
            //*/

            break;
        case key_S:
            ss << "S";

            ///*ANANIAS
                maxVel = -0.2;
                maxTurn = 0;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
            //*/

            break;
        case key_D:
            ss << "D";

            ///*ANANIAS
                maxVel = 0;
                maxTurn = -0.4;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
            //*/

            break;
        case key_I:
            ss << "I";

            neck_x_rotation_ = 20;
            neck_z_rotation_ = 0;
            mouth_ = 0;
            upper_head_ = 0;
            left_eye_ = 0;
            right_eye_ = 0;
            maxVel = 0;
            maxTurn = 0;
            dirty = true;

            break;
        case key_J:
            ss << "J";

            neck_x_rotation_ = 0;
            neck_z_rotation_ = 20;
            mouth_ = 0;
            upper_head_ = 0;
            left_eye_ = 0;
            right_eye_ = 0;
            maxVel = 0;
            maxTurn = 0;
            dirty = true;

            break;
        case key_K:
            ss << "K";

            neck_x_rotation_ = -20;
            neck_z_rotation_ = 0;
            mouth_ = 0;
            upper_head_ = 0;
            left_eye_ = 0;
            right_eye_ = 0;
            maxVel = 0;
            maxTurn = 0;
            dirty = true;

            break;
        case key_L:
            ss << "L";

            neck_x_rotation_ = 0;
            neck_z_rotation_ = -20;
            mouth_ = 0;
            upper_head_ = 0;
            left_eye_ = 0;
            right_eye_ = 0;
            maxVel = 0;
            maxTurn = 0;
            dirty = true;

            break;

        case key_C:
            ss << "C";

            flag_ = 1;
            maxVel = 0;
            maxTurn = 0;
            neck_x_rotation_ = 0;
            neck_z_rotation_ = 0;
            mouth_ = 0;
            upper_head_ = 0;
            left_eye_ = 0;
            right_eye_ = 0;
            dirty = true;
            break;

        case key_V:
            ss << "V";

            flag_ = 2;
            maxVel = 0;
            maxTurn = 0;
            neck_x_rotation_ = 0;
            neck_z_rotation_ = 0;
            mouth_ = 0;
            upper_head_ = 0;
            left_eye_ = 0;
            right_eye_ = 0;
            dirty = true;
            break;

        default:
            ss << "COMMAND NOT FOUND!";

            flag_ = 0;
            maxVel = 0;
            maxTurn = 0;
            neck_x_rotation_ = 0;
            neck_z_rotation_ = 0;
            mouth_ = 0;
            upper_head_ = 0;
            left_eye_ = 0;
            right_eye_ = 0;
            dirty = false;

            break;
    }

    //ss << "publishing " << count;
    msg.data = ss.str();
    chatter_publisher.publish(msg);

    if(dirty){
        cmdvel_.linear.x = maxVel;
        cmdvel_.angular.z = maxTurn;
        cmdhead_.neck_x_rotation = neck_x_rotation_;
        cmdhead_.neck_z_rotation = neck_z_rotation_;
        cmdhead_.mouth = mouth_;
        cmdhead_.upper_head = upper_head_;
        cmdhead_.left_eye = left_eye_;
        cmdhead_.right_eye = right_eye_;
        cmdhead_.flag = flag_;
        pub_head.publish(cmdhead_);
        pub_base.publish(cmdvel_);
    }
}

void QNode::publish_2(){
    int count = 9999;

    std_msgs::String msg;
    std::stringstream ss;
    ss << "publishing " << count;
    msg.data = ss.str();
    chatter_publisher.publish(msg);
}

}  // namespace interface
