/**
 * @file /include/interface/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef interface_QNODE_HPP_
#define interface_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

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

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace interface {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	void publish(int command);
	void publish_2();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
	std::string script_folder;

Q_SIGNALS:
	void loggingUpdated();
    	void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    	QStringListModel logging_model;
	
	//-------- Ananias' variables -----------------------------
	
	geometry_msgs::Twist cmdvel_;
        drrobot_h20_player::HeadCmd cmdhead_;
        //doesn't need //ros::NodeHandle n_;
        ros::Publisher pub_base, pub_head;
        
        //DrRobotHeadScript drrobotheadscript_;
        //DrRobotScriptTeleop drrobotscriptteleop_;
        
        //std::string xmlFile_head, xmlFile_arm;

	//-------- /Ananias' variables ----------------------------
        
};

}  // namespace interface

#endif /* interface_QNODE_HPP_ */
