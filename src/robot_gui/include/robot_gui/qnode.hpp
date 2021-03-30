/**
 * @file /include/robot_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robot_gui_QNODE_HPP_
#define robot_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include "robot_gui_server.hpp"
//#include "navigation_client.hpp"
#include <drrobot_h20_player/PowerInfo.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	RobotGuiServer *aS;

	bool showingTelepresence, showingDefaultTab, showingBingo, showingTrivia, waiting, onCall;
	bool callRejected;
	bool sessionCompleted;
	bool speak, showText;
	int  tabIndex;
	//bool clientInitialized;

	std::string command_speech_s;
	std::stringstream command_speech_ss, skypeUser, nameOfCaller;

	QString auxstr;
	QString formatedstring;
	int subtab;

	//power info variables
	double bat1_vol_;
	double bat2_vol_;
	double bat1_temp_;
	double bat2_temp_;
	double dcin_vol_;
	double ref_vol_;
	unsigned short int power_status_;
	unsigned short int power_path_;
	unsigned short int charge_path_;

	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
	void setGUIText();

	void powerInfoReceived(const drrobot_h20_player::PowerInfo::ConstPtr& powerInfo);
	
	void closeInterface();

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

	void ttsSpeak(QString text, QString lang, bool male, bool webbased);

    void playBingoDemo();
	void rigBingoDemo();

    void askToPlayBingo();

    void getTTSDefaults(QString *lang, bool *male, bool *webbased);

Q_SIGNALS:
	void loggingUpdated();
    	void rosShutdown();
	//void showTabTelepresence(int index);
	void changeTab(int index);
	void updateBatteryInfo();
	void trivia_activity();
	void bingo_activity();
	//void bingo_activity(QString text);
	void default_activity();
	//void default_activity(QString text);
	void reset_defaultTab();



private:
	int init_argc;
	char** init_argv;
	//ros::Publisher chatter_publisher;
    	QStringListModel logging_model;

	drrobot_h20_player::PowerInfo powerInfo_;
	ros::Subscriber powerInfo_sub;
	ros::Publisher 	powerInfo_pub;
	ros::Publisher 	chatter_publisher;
	ros::Subscriber chatter_subscriber;
	ros::ServiceClient tts_client;
};

}  // namespace robot_gui

#endif /* robot_gui_QNODE_HPP_ */
