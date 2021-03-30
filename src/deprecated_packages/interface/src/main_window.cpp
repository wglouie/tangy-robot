/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <QKeyEvent>
#include <iostream>
#include "../include/interface/main_window.hpp"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <script_player/ScriptPlayAction.h>
#include <string>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace interface {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(1); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    qnode.init();
    client = new ScriptClient("script_player_client");
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
    if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
    }
    else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "interface");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "interface");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::keyPressEvent(QKeyEvent * event)
{
    int cmd = 0;
    switch(event->key()){
        case(0x57):
            //ui.label_4->setText("Key pressed: W");
            cmd = 1;//qnode.publish(1);
            break;
        case(0x41):
            cmd = 2;//qnode.publish(2);
            //ui.label_4->setText("Key pressed: A");
            break;
        case(0x53):
            cmd = 3;//qnode.publish(3);
            //ui.label_4->setText("Key pressed: S");
            break;
        case(0x44):
            cmd = 4;//qnode.publish(4);
           // ui.label_4->setText("Key pressed: D");
            break;
        case(0x49):
            cmd = 5;//qnode.publish(5);
           //ui.label_4->setText("Key pressed: I");
            break;
        case(0x4a):
            cmd = 6;//qnode.publish(6);
           // ui.label_4->setText("Key pressed: J");
            break;
        case(0x4b):
            cmd = 7;//qnode.publish(7);
           // ui.label_4->setText("Key pressed: K");
            break;
        case(0x4c):
            cmd = 8;//qnode.publish(8);
           // ui.label_4->setText("Key pressed: L");
            break;
        case(0x43):
            cmd = 9;//qnode.publish(9);
           // ui.label_4->setText("Key pressed: C");
            break;
        case(0x56):
            cmd = 10;//qnode.publish(10);
           // ui.label_4->setText("Key pressed: V");
            break;
        default:
            cmd = 0;//qnode.publish(9);
           // ui.label_4->setText("UNKNOWN COMMAND");
            break;
    }
    qnode.publish(cmd);
}
void MainWindow::playscript(std::string folder)
{
    //client[0].ac.cancelAllGoals();
    client[0].send(qnode.script_folder + folder + "/");
}


void MainWindow::on_pushButton00_clicked(){
    playscript("Test");
}

void MainWindow::on_pushButton01_clicked(){
    playscript("Make_tea");
}

void MainWindow::on_pushButton02_clicked(){
    playscript("Follow_me");
}

void MainWindow::on_pushButton03_clicked(){
    playscript("Okay");
}

void MainWindow::on_pushButton10_clicked(){
    playscript("Well_done");
}

void MainWindow::on_pushButton11_clicked(){
    playscript("Great");
}

void MainWindow::on_pushButton12_clicked(){
    playscript("Great_job");
}

void MainWindow::on_pushButton13_clicked(){
    playscript("Doing_well");
}

void MainWindow::on_pushButton20_clicked(){

}

void MainWindow::on_pushButton21_clicked(){
    playscript("Dont_know");
}

void MainWindow::on_pushButton22_clicked(){

}

void MainWindow::on_pushButton23_clicked(){
    playscript("Finished");
}

void MainWindow::on_pushButton30_clicked(){

}

void MainWindow::on_pushButton31_clicked(){
    playscript("Sorry");
}

//-----------------Go to Kitchen ----------
void MainWindow::on_pushButton40_clicked(){
    playscript("Gokitch_min");
}

void MainWindow::on_pushButton41_clicked(){
    playscript("Gokitch_max");
}

void MainWindow::on_pushButton42_clicked(){
    playscript("Gokitch_vid");
}

//-----------------Tap on ----------------
void MainWindow::on_pushButton50_clicked(){
    playscript("Tapon_min");
}

void MainWindow::on_pushButton51_clicked(){
    playscript("Tapon_max");
}

void MainWindow::on_pushButton52_clicked(){
    playscript("Tapon_vid_tea");
}

//------------------Fill------------------
void MainWindow::on_pushButton60_clicked(){
    playscript("Fill_min");
}

void MainWindow::on_pushButton61_clicked(){
    playscript("Fill_max");
}

void MainWindow::on_pushButton62_clicked(){
    playscript("Fill_vid");
}

//---------------Tap off----------------
void MainWindow::on_pushButton70_clicked(){
    playscript("Tapoff_min");
}

void MainWindow::on_pushButton71_clicked(){
    playscript("Tapoff_max");
}

void MainWindow::on_pushButton72_clicked(){
    playscript("Tapoff_vid_tea");
}

//---------------Switch-----------------
void MainWindow::on_pushButton80_clicked(){
    playscript("Switch_min");
}

void MainWindow::on_pushButton81_clicked(){
    playscript("Switch_max");
}

void MainWindow::on_pushButton82_clicked(){
    playscript("Switch_vid");
}

//--------------Plug in Socket -----------
void MainWindow::on_pushButton90_clicked(){
    playscript("Pluginsocket_min");
}

void MainWindow::on_pushButton91_clicked(){
    playscript("Pluginsocket_max");
}

void MainWindow::on_pushButton92_clicked(){

}

//--------------Press Button --------------
void MainWindow::on_pushButton100_clicked(){
    playscript("Pressbutton_max");
}

void MainWindow::on_pushButton101_clicked(){
    playscript("Pressbutton_min");
}

void MainWindow::on_pushButton102_clicked(){

}

//---------------Bag in -------------------
void MainWindow::on_pushButton110_clicked(){
    playscript("bagin_min");
}

void MainWindow::on_pushButton111_clicked(){
    playscript("bagin_max");
}

void MainWindow::on_pushButton112_clicked(){
    playscript("bagin_vid");
}

//---------------Water in teacup ----------
void MainWindow::on_pushButton120_clicked(){
    playscript("Watercup_min");
}

void MainWindow::on_pushButton121_clicked(){
    playscript("Watercup_max");
}

void MainWindow::on_pushButton122_clicked(){
    playscript("Watercup_vid");
}

//---------------Fridge-------------------
void MainWindow::on_pushButton130_clicked(){
    playscript("Fridge_min");
}

void MainWindow::on_pushButton131_clicked(){
    playscript("Fridge_max");
}

void MainWindow::on_pushButton132_clicked(){
    playscript("Fridge_vid");
}

//----------------Milk--------------------
void MainWindow::on_pushButton140_clicked(){
    playscript("Milk_min");
}

void MainWindow::on_pushButton141_clicked(){
    playscript("Milk_max");
}

void MainWindow::on_pushButton142_clicked(){
    playscript("Milk_vid");
}

//----------------Sugar---------------------
void MainWindow::on_pushButton150_clicked(){
    playscript("Sugar_min");
}

void MainWindow::on_pushButton151_clicked(){
    playscript("Sugar_max");
}

void MainWindow::on_pushButton152_clicked(){
    playscript("Sugar_vid");
}

//---------------Bin----------------------
void MainWindow::on_pushButton160_clicked(){
    playscript("Bin_min");
}

void MainWindow::on_pushButton161_clicked(){
    playscript("Bin_max");
}

void MainWindow::on_pushButton162_clicked(){
    playscript("Bin_vid");
}

//void MainWindow::on_pushButton_2_clicked(){
    //ui.label_4->setText("Pressed button_2");
    //qnode.publish_2();

//}

//void MainWindow::on_pushButton_3_clicked(){
   // ui.label_4->setText("Pressed button_3");
    //qnode.publish();

//}

//void MainWindow::on_pushButton_4_clicked(){
    //ui.label_4->setText("Pressed button_4");
    //qnode.publish_2();
//}

}  // namespace interface

