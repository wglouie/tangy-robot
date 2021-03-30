/**
 * @file /include/robot_gui/main_window.hpp
 *
 * @brief Qt based gui for robot_gui.
 *
 * @date November 2010
 **/
#ifndef robot_gui_MAIN_WINDOW_H
#define robot_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*
#include <string>
#include <std_msgs/String.h>
#include <sstream>
*/

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robot_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

const static int TRIVIA_SUBTAB_0 = 0;
const static int TRIVIA_SUBTAB_1 = 1;
const static int TRIVIA_SUBTAB_2 = 2;

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
	
	QString last_string;

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

	void telepresenceSession();
	
  QString trivia_curr_screen_text;
  
  
public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	//void on_button_connect_clicked(bool check );
	//void on_checkbox_use_environment_stateChanged(int state);

	void on_button_Answer_clicked();
	void on_button_Reject_clicked();
	void changeTab(int index);
	void updatePowerInfo();
	//void bingo_activity(QString text);
	void bingo_activity();
  void trivia_activity();
	//void default_activity(QString text);
	void default_activity();
	void reset_defaultTab();
	// tts
  void sayWithGUISettings(QString text);
	void sayIt();
    //void playBingoDemo();
    //void rigButton();
    //void askToPlayBingo();
    //void resetButtons();
    //void faceDetection();
	void speak(QString text, QString lang = "en", bool male = true, bool webbased=true);
	//custom font sizing
	int maxFontSize(QString text, QLabel *label);
	//Split text into segments based on newline character
	std::vector< QString > splitText(QString text);
	/******************************************
	** Manual connections
	*******************************************/
	void updateLoggingView(); // no idea why this can't connect automatically

Q_SIGNALS:
	//void sendInfo(QString msg);
	void initSkypeCal();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	
};

}  // namespace robot_gui

#endif // robot_gui_MAIN_WINDOW_H
