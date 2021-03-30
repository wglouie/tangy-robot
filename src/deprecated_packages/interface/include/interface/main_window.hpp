/**
 * @file /include/interface/main_window.hpp
 *
 * @brief Qt based gui for interface.
 *
 * @date November 2010
 **/
#ifndef interface_MAIN_WINDOW_H
#define interface_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
//#include <QKeyEvent>

#include "script_player_client.hpp"
#include <string>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace interface {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
	void playscript(std::string folder);
	
	void keyPressEvent(QKeyEvent * event);
	ScriptClient *client;

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);


	void on_pushButton00_clicked();
	void on_pushButton01_clicked();
	void on_pushButton02_clicked();
	void on_pushButton03_clicked();
	void on_pushButton10_clicked();
	void on_pushButton11_clicked();
	void on_pushButton12_clicked();
	void on_pushButton13_clicked();
	void on_pushButton20_clicked();
	void on_pushButton21_clicked();
	void on_pushButton22_clicked();
	void on_pushButton23_clicked();
	void on_pushButton30_clicked();
	void on_pushButton31_clicked();
	void on_pushButton40_clicked();
	void on_pushButton41_clicked();
	void on_pushButton42_clicked();
	void on_pushButton50_clicked();
	void on_pushButton51_clicked();
	void on_pushButton52_clicked();
	void on_pushButton60_clicked();
	void on_pushButton61_clicked();
	void on_pushButton62_clicked();
	void on_pushButton70_clicked();
	void on_pushButton71_clicked();
	void on_pushButton72_clicked();
	void on_pushButton80_clicked();
	void on_pushButton81_clicked();
	void on_pushButton82_clicked();
	void on_pushButton90_clicked();
	void on_pushButton91_clicked();
	void on_pushButton92_clicked();
	void on_pushButton100_clicked();
	void on_pushButton101_clicked();
	void on_pushButton102_clicked();
	void on_pushButton110_clicked();
	void on_pushButton111_clicked();
	void on_pushButton112_clicked();
	void on_pushButton120_clicked();
	void on_pushButton121_clicked();
	void on_pushButton122_clicked();
	void on_pushButton130_clicked();
	void on_pushButton131_clicked();
	void on_pushButton132_clicked();
	void on_pushButton140_clicked();
	void on_pushButton141_clicked();
	void on_pushButton142_clicked();
	void on_pushButton150_clicked();
	void on_pushButton151_clicked();
	void on_pushButton152_clicked();
	void on_pushButton160_clicked();
	void on_pushButton161_clicked();
	void on_pushButton162_clicked();
	
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace interface

#endif // interface_MAIN_WINDOW_H
