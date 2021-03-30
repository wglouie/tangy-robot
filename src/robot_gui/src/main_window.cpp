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
#include <iostream>
#include "../include/robot_gui/main_window.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Char.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <face_detection/identifyAction.h>


#define default_tab 	 0
#define robotStatus_tab  1
#define telepresence_tab 3
#define bingo_tab	 4
#define log_tab 	 5
#define trivia_tab 2

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_gui {

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
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    //set the color of the buttons "start" and "reject"
    ui.button_Reject->setStyleSheet("background-color: rgb(194,0,0)");
    ui.button_Answer->setStyleSheet("background-color: rgb(48,125,25)");

    QObject::connect(&qnode, SIGNAL(trivia_activity()), this, SLOT(trivia_activity()), Qt::QueuedConnection);
    //show numbers and text on bingo tab
    QObject::connect(&qnode, SIGNAL(bingo_activity()), this, SLOT(bingo_activity()), Qt::QueuedConnection);

    //show text and speak in the default tab
    QObject::connect(&qnode, SIGNAL(default_activity()), this, SLOT(default_activity()), Qt::QueuedConnection);

    //reset the text in the default tab
    QObject::connect(&qnode, SIGNAL(reset_defaultTab()), this, SLOT(reset_defaultTab()), Qt::QueuedConnection);

    //change tab()
    QObject::connect(&qnode, SIGNAL(changeTab(int)), this, SLOT(changeTab(int)), Qt::QueuedConnection);

    //update battery info
    QObject::connect(&qnode, SIGNAL(updateBatteryInfo()),this, SLOT(updatePowerInfo()),Qt::QueuedConnection);

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /* TTS */
    QObject::connect(ui.ttsSayBtn, SIGNAL(clicked()), this, SLOT(sayIt()));

   // QObject::connect(ui.playBingoDemoBtn, SIGNAL(clicked()), this, SLOT(playBingoDemo()));

   // QObject::connect(ui.askToPlayBingoBtn, SIGNAL(clicked()), this, SLOT(askToPlayBingo()));

   // QObject::connect(ui.faceDetectionBtn, SIGNAL(clicked()), this, SLOT(faceDetection()));

   // QObject::connect(ui.resetBtn, SIGNAL(clicked()), this, SLOT(resetButtons()));

    //QObject::connect(ui.rigBingoBtn, SIGNAL(clicked()), this, SLOT(rigButton()));
	
    /*********************
    ** Auto Start
    **********************/

    if ( !qnode.init() ) {
        showNoMasterMessage();
        qnode.closeInterface();
        //emit qnode.rosShutdown();
    }

    // TTS Defaults
    QString language;
    bool male, webbased;
    qnode.getTTSDefaults(&language, &male, &webbased);
    ui.webBasedCheck->setChecked(webbased);
    int index = ui.langBox->findText(language);
    if(index >= 0){
       ui.langBox->setCurrentIndex(index);
    }
    if(male){
       ui.genderBox->setCurrentIndex(0);
    } else {
       ui.genderBox->setCurrentIndex(1);
    }
    
    trivia_curr_screen_text="";
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

void MainWindow::on_button_Answer_clicked(){

    ui.button_Answer->setEnabled(false);
    ui.button_Reject->setEnabled(false);

    qnode.waiting = false;
    qnode.callRejected = false;
//    qnode.sessionCompleted = false;
    //qnode.onCall = true;
    QApplication::processEvents();
 
    std::string aux;
    aux = "python ~/ros/tangy/src/robot_gui/files/callfriend.py " + qnode.skypeUser.str();

    system(aux.c_str());//"python ~/ros/tangy/src/robot_gui/files/callfriend.py echo123");

    ui.label_telepresence->setText("Session completed.\nThank you!");
    ui.label_telepresence->setWordWrap(true);
    qnode.onCall = false;
    qnode.sessionCompleted = true;
    QApplication::processEvents();

    //system("aplay ~/ros/tangy/src/telepresence/telepresence_gui/files/thank_you_very_much.wav");
	
    //ui.tab_manager->repaint();
    //QWidget::focusWidget();
    //QWidget::activateWindow();
    QWidget::setWindowState(Qt::WindowActive);
    QWidget::setWindowState(Qt::WindowMaximized);
    QApplication::processEvents();

    
    sayWithGUISettings("Thank you very much.");
    //QString voicecommand = "";
   // voicecommand = "echo \"Thank you very much.\" | festival --tts";
    //system(voicecommand.toStdString().c_str());

    sleep(3);
    ui.tab_manager->setCurrentIndex(qnode.tabIndex);

}

void MainWindow::on_button_Reject_clicked(){
    qnode.callRejected = true;
    qnode.waiting = false;
    ui.button_Answer->setEnabled(false);
    ui.button_Reject->setEnabled(false);
    QApplication::processEvents();

    sayWithGUISettings("Video call rejected");

    //QString voicecommand = "";
    //voicecommand = "echo \"Video call rejected!\" | festival --tts";
    //system(voicecommand.toStdString().c_str());

    ui.label_telepresence->setText("Session rejected.\nThank you very much!");
    ui.label_telepresence->setWordWrap(true);
    ui.tab_manager->repaint();
    QWidget::focusWidget();
    QApplication::processEvents();

    //voicecommand = "echo \"Thank you very much.\" | festival --tts";
    //system(voicecommand.toStdString().c_str());
    sayWithGUISettings("Thank you very much");

    sleep(3);

    ui.tab_manager->setCurrentIndex(qnode.tabIndex);
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

void MainWindow::changeTab(int index){
    qnode.tabIndex = ui.tab_manager->currentIndex();
    
    if(index == telepresence_tab){
    	system("python ~/ros/tangy/src/robot_gui/files/starts.py");
	telepresenceSession();
    }
   /* else if(index == bingo_tab){
	ui.tab_manager->setCurrentIndex(bingo_tab);
    	ui.tab_manager->repaint();
    	QWidget::focusWidget();
    	QApplication::processEvents();

    }*/
    else{
        ui.tab_manager->setCurrentIndex(index);
    	ui.tab_manager->repaint();
    	//QWidget::focusWidget();
        QWidget::setWindowState(Qt::WindowActive);
        QWidget::setWindowState(Qt::WindowMaximized);
        QApplication::processEvents();
        QWidget::focusWidget();
    	QApplication::processEvents();
    }

}
//void MainWindow::resetButtons(){
//    ROS_INFO("TESTIIGNIGNIGNIGNIGNIGN");
//    ui.playBingoDemoBtn->show();
//    ui.askToPlayBingoBtn->show();
//    ui.faceDetectionBtn->show();
//ui.rigBingoBtn->show();
//	ui.label_bingoInfo->setText("Bingo Game");
//    ui.label_bingoInfo->setWordWrap(true);
//}

//void MainWindow::rigButton(){
//    ui.rigBingoBtn->hide();
//    ui.playBingoDemoBtn->hide();
//    ui.askToPlayBingoBtn->hide();
//    ui.faceDetectionBtn->hide();
//	qnode.rigBingoDemo();
//}
//void MainWindow::faceDetection(){
//    actionlib::SimpleActionClient<bingo_demo_face::FaceFinderAction> ac("face_finder", true);

//    ROS_INFO("Waiting for face finder server to start.");
//    // wait for the action server to start
//    ac.waitForServer(); //will wait for infinite time

//    ROS_INFO("Face FInder server started, sending goal.");
//    // send a goal to the action
//    bingo_demo_face::FaceFinderGoal face_goal;
//    face_goal.goal = 1;

//    ac.sendGoal(face_goal);

//}

void MainWindow::updatePowerInfo(){
    ui.label_volBattery1->setText(QString::number((qnode.bat1_vol_)+(qnode.bat2_vol_)));
//    ui.label_volBattery2->setText(QString::number(batteryread.batmean_vol_));
    ui.label_tempBattery1->setText(QString::number(qnode.bat1_temp_));
    ui.label_tempBattery2->setText(QString::number(qnode.bat2_temp_));

    if(qnode.bat1_vol_ <= 20){
        ui.progressBar_battery1->setValue((int)(100*(qnode.bat1_vol_/20)));
        ui.progressBar_battery2->setValue((int)(100*(qnode.bat2_vol_/20)));
    }
    else{
        ui.progressBar_battery1->setValue(77);
        ui.progressBar_battery2->setValue(25);
    }

    ui.tab_manager->repaint();
    QApplication::processEvents();
}

void MainWindow::bingo_activity(){//(QString text){
	//ROS_INFO("Bingo activity slot entered");
	//FIT TEXT TO BOX

	int fit = false;
	this->ui.label_bingoInfo->setText("");
	
	QLabel *myLabel = this->ui.label_bingoInfo;
	QFont myFont = ui.label_bingoInfo->font();

	QString str = qnode.formatedstring;

	if(qnode.formatedstring.size()<5)
	{
	  std::cout<<"Point size = "<<400<< std::endl;
 		myFont.setPointSize(400);
		myLabel->setFont(myFont);
	} else if (qnode.formatedstring.size()<20){
	  std::cout<<"Point size = "<<95<< std::endl;
 		myFont.setPointSize(95);
		myLabel->setFont(myFont);
	} else {
	  std::cout<<"Point size = "<<75<< std::endl;
 		myFont.setPointSize(75);
		myLabel->setFont(myFont);
	}
	myLabel->setText(qnode.formatedstring);
	myLabel->setWordWrap(true);
  if(qnode.showText && qnode.speak){
  	//QApplication::processEvents();
  	//system(qnode.command_speech_s.c_str());
 		ui.tab_manager->repaint();
  	sayWithGUISettings(QString::fromStdString(qnode.aS->speech_));
		qnode.showText = false;
		qnode.speak = false;
  }
  else if(qnode.showText){
  	//ui.tab_manager->repaint();
  	//QApplication::processEvents();
	 	ui.tab_manager->repaint();

		qnode.showText = false;
		qnode.speak = false;
  }
  else if(qnode.speak){
  	//system(qnode.command_speech_s.c_str());
  	sayWithGUISettings(QString::fromStdString(qnode.aS->speech_));
		qnode.showText = false;
		qnode.speak = false;
  }

}

void MainWindow::trivia_activity(){
	//Define subtabs for trivia tab
  //Definitions for the text boxes:
  //label_single_text - text for subtab 0, with only one text box on the tab
  //label_mt2_1, label_mt2_2 - text boxes for subtab 1, with two text boxes on the tab
  //label_mt4_1, label_mt4_2, label_mt4_3, label_mt4_4 - text boxes for subtab 2, with four text boxes
  ui.genderBox->setCurrentIndex(0);
  if(ui.tab_manager->currentIndex()!=trivia_tab){
    ui.tab_manager->setCurrentIndex(trivia_tab);
  }
  if(qnode.formatedstring!=last_string){
    switch(qnode.subtab){
      case(TRIVIA_SUBTAB_0):
      {
        if(ui.trivia_question->currentIndex()!=TRIVIA_SUBTAB_0){
          ui.trivia_question->setCurrentIndex(TRIVIA_SUBTAB_0);
        }
        QLabel *myLabel_st = this -> ui.label_single_text;
        QFont ft=myLabel_st->font();
        ft.setPointSize(maxFontSize(qnode.formatedstring, myLabel_st));
        
        myLabel_st->setFont(ft);
        myLabel_st->setText(qnode.formatedstring);
        myLabel_st->setWordWrap(true);
        ui.trivia_question -> repaint();
  
        QApplication::processEvents();
        sayWithGUISettings(QString::fromStdString(qnode.aS->speech_));
        last_string=qnode.formatedstring;
        break;
      }
      case(TRIVIA_SUBTAB_1):
      {
        if(ui.trivia_question->currentIndex()!=TRIVIA_SUBTAB_1){
          ui.trivia_question->setCurrentIndex(TRIVIA_SUBTAB_1);
        }
        QLabel *myLabel_mt2_1 = this -> ui.label_mt2_1;
        QLabel *myLabel_mt2_2 = this -> ui.label_mt2_2;
        QFont ft0=myLabel_mt2_1->font();
        QFont ft1=myLabel_mt2_2->font();
        
        if(qnode.formatedstring.indexOf("\n")!=-1){
          QString str0=qnode.formatedstring.left(qnode.formatedstring.indexOf("\n"));
          QString str1=qnode.formatedstring.mid(qnode.formatedstring.indexOf("\n")+1);
          ft0.setPointSize(std::min(maxFontSize(str0, myLabel_mt2_1),maxFontSize(str1, myLabel_mt2_2)));
          ft1.setPointSize(std::min(maxFontSize(str0, myLabel_mt2_1),maxFontSize(str1, myLabel_mt2_2)));
          
          myLabel_mt2_1->setFont(ft0);
          myLabel_mt2_1->setText(str0);
          myLabel_mt2_1->setWordWrap(true);
          myLabel_mt2_2->setFont(ft1);
          myLabel_mt2_2->setText(str1);
          myLabel_mt2_2->setWordWrap(true);
          ui.trivia_question -> repaint();
          // std::string utf8_text0 = str0.toUtf8().constData();
          // ROS_INFO("DEBUG QSTRING: %s", utf8_text0.c_str());
          // std::string utf8_text1 = str1.toUtf8().constData();
          // ROS_INFO("DEBUG QSTRING: %s", utf8_text1.c_str());
          QApplication::processEvents();
          sayWithGUISettings(QString::fromStdString(qnode.aS->speech_));
        }else{
          ROS_ERROR("QString not formatted properly: [TRIVIA_SUBTAB_1, qnode.formatedstring.indexOf() = %d, qnode.formatedstring.size() = %d]",qnode.formatedstring.indexOf("\n"), qnode.formatedstring.size());
        }
        last_string=qnode.formatedstring;
        
        break;
      }
      case(TRIVIA_SUBTAB_2):
      {
        // std::string utf8_text0 = qnode.formatedstring.toUtf8().constData();
        // ROS_INFO("[TRIVIA_SUBTAB_2, qnode.formatedstring = %s]", utf8_text0.c_str());
        if(ui.trivia_question->currentIndex()!=TRIVIA_SUBTAB_2){
          ui.trivia_question->setCurrentIndex(TRIVIA_SUBTAB_2);
        }
        QLabel *myLabel_mt4_1 = this -> ui.label_mt4_1;
        QLabel *myLabel_mt4_2 = this -> ui.label_mt4_2;
        QLabel *myLabel_mt4_3 = this -> ui.label_mt4_3;
        QLabel *myLabel_mt4_4 = this -> ui.label_mt4_4;
        QFont f_mt4_1=myLabel_mt4_1->font();
        QFont f_mt4_2=myLabel_mt4_2->font();
        QFont f_mt4_3=myLabel_mt4_3->font();
        QFont f_mt4_4=myLabel_mt4_4->font();
  
        int q_ind=0;
  
        //Segment string into segments
        //Simultaneously check if the text inputs actually exist before painting on them subtab
        if(qnode.formatedstring.indexOf("\n")!=-1){
          //Split text and check whether each segment actually has text in it
          std::vector<QString> segments=splitText(qnode.formatedstring);
          
          //Resize fonts based on the string size of each of the segments
          int max_size=10000;
          
            //loop through the segment vector (with the exception of the first element, which holds the generally longer question text)
            //Each of the labels SHOULD be the same size, so just use the first label
          for(int ind_seg=1;ind_seg<segments.size();ind_seg++){
            int ftsz=maxFontSize(segments.at(ind_seg), myLabel_mt4_1);
            if(ftsz<max_size){
              max_size=ftsz;
            }
          }
          
          //Set the text for the first label (should be the trivia question)
          int fontSize_mt4_1 = maxFontSize(segments.at(0), myLabel_mt4_1);
          f_mt4_1.setPointSize(fontSize_mt4_1);
          myLabel_mt4_1->setFont(f_mt4_1);
          myLabel_mt4_1->setText(segments.at(0));
          myLabel_mt4_1->setWordWrap(true);
          
          //Set the other labels (should be the answers for the trivia question)
          f_mt4_2.setPointSize(max_size);
          myLabel_mt4_2->setFont(f_mt4_2);
          myLabel_mt4_2->setText(segments.at(1));
          myLabel_mt4_2->setWordWrap(true);
          
          f_mt4_3.setPointSize(max_size);
          myLabel_mt4_3->setFont(f_mt4_3);
          myLabel_mt4_3->setText(segments.at(2));
          myLabel_mt4_3->setWordWrap(true);
          
          f_mt4_4.setPointSize(max_size);
          myLabel_mt4_4->setFont(f_mt4_4);
          myLabel_mt4_4->setText(segments.at(3));
          myLabel_mt4_4->setWordWrap(true);
  
        }else{
          q_ind=-1;
          ROS_ERROR("QString not formatted properly: [TRIVIA_SUBTAB_1, qnode.formatedstring.indexOf() = %d, qnode.formatedstring.size() = %d]",qnode.formatedstring.indexOf("\n"), qnode.formatedstring.size());
        }
      
        ui.trivia_questionPage3 -> repaint();
        QApplication::processEvents();
        sayWithGUISettings(QString::fromStdString(qnode.aS->speech_));
        last_string=qnode.formatedstring;
        break;
      }
      default:
      {
        ROS_ERROR("Trying to bring up a tab which doesn't exist! [subtab # : %d]", qnode.subtab);
        break;
      }
        
    }
  }else{
    sayWithGUISettings(QString::fromStdString(qnode.aS->speech_));
  }
}


void MainWindow::default_activity(){//QString text){
    //printf("I'll show the text in the tab[2]\n\n");
    if(qnode.showText && qnode.speak){
    	ui.label_default->setText(qnode.formatedstring);
    	ui.label_default->setWordWrap(true);
    	ui.tab_manager->repaint();
	QApplication::processEvents();
	sayWithGUISettings(QString::fromStdString(qnode.aS->speech_));
    }
    else if(qnode.showText){

	ui.label_default->setText(qnode.formatedstring);
    	ui.label_default->setWordWrap(true);
    	ui.tab_manager->repaint();
	QApplication::processEvents();
    }
    else if(qnode.speak){
	sayWithGUISettings(QString::fromStdString(qnode.aS->speech_));
    }
    //printf("I showed the text in the tab\n\n");
}

void MainWindow::reset_defaultTab(){
	qnode.formatedstring = "<font color=\"black\" size=\"40\"> Tangy </font>";
	ui.label_default->setText(qnode.formatedstring);
    	ui.label_default->setWordWrap(true);
    	ui.tab_manager->repaint();
	QApplication::processEvents();
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
    QSettings settings("Qt-Ros Package", "robot_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "robot_gui");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event){
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::telepresenceSession(){
	QWidget::setWindowState(Qt::WindowActive);
	QWidget::setWindowState(Qt::WindowMaximized);
    	QApplication::processEvents();
	//ui.show();
	//QApplication::processEvents();

    	ui.label_telepresence->setText("Video call");

    	ui.button_Answer->setEnabled(true);
    	ui.button_Reject->setEnabled(true);
        ui.tab_manager->setCurrentIndex(telepresence_tab);
    	ui.tab_manager->repaint();
    	QWidget::focusWidget();
    	QApplication::processEvents();

    	//std::string voicecommand = "";//QString voicecommand_2 = "";
//    	voicecommand = "echo \"You have a Skype call! What would you like to do?\" | festival --tts";
	//voicecommand = "echo " + qnode.nameOfCaller.str() + "\"Wants to talk with you. Press the green button to accept the telepresence session or the red button to reject it.\" | festival --tts";
    	//system(voicecommand.c_str());
        sayWithGUISettings(QString::fromStdString(qnode.nameOfCaller.str()) + "Wants to talk with you. Press the green button to accept the telepresence session or the red button to reject it.");
}

void MainWindow::sayWithGUISettings(QString text){
	bool male = (ui.genderBox->currentIndex() == 0);
	QString lang = ui.langBox->currentText();
	bool webbased = ui.webBasedCheck->isChecked();
	if(lang == "en_us"){
		lang = "en";
	}
	if(lang == "en_uk"){
		lang = "uk";
	}
	speak(text, lang, male, webbased);
}

//void MainWindow::playBingoDemo(){

//    ui.playBingoDemoBtn->hide();
//    ui.askToPlayBingoBtn->hide();
//    ui.faceDetectionBtn->hide();
//	ui.rigBingoBtn->hide();
//    qnode.playBingoDemo();

//}

//void MainWindow::askToPlayBingo(){
//    qnode.askToPlayBingo();


//}


void MainWindow::sayIt(){
	sayWithGUISettings(ui.ttsInput->toPlainText());
}

void MainWindow::speak(QString text, QString lang, bool male, bool webbased){
	qnode.ttsSpeak(text, lang, male, webbased);
}

int MainWindow::maxFontSize(QString text, QLabel *label){
  label->setWordWrap(true);
  QRect rect=label->contentsRect();
  int height=rect.height()/1.25;
  int width=rect.width()/1.25;
  QFont ft=label->font();
  int fontsz=5;
  
  while(fontsz<300){
    QFont f(ft);
    f.setPointSize( fontsz );
    QRect r = QFontMetrics(f).boundingRect(0,0,width,height,Qt::TextWordWrap, text );
    if (r.height() <= height && r.width() <= width){
          fontsz=fontsz+5;
    }else{
          break;
    }
  }
  
  return fontsz;
}

std::vector< QString > MainWindow::splitText(QString text){
  using namespace std;
  vector<QString> segments;
  if(text.indexOf("\n")!=-1){
    QString frontseg=text.left(text.indexOf("\n"));
    QString backseg=text.mid(text.indexOf("\n")+1);
    segments.push_back(frontseg);
    
    int linebreak=backseg.indexOf("\n");
    while(linebreak!=-1){
      frontseg=backseg.left(linebreak);
      segments.push_back(frontseg);
      backseg=backseg.mid(linebreak+1);
      linebreak=backseg.indexOf("\n");
    }
    segments.push_back(backseg);
    return segments;
  }else{
    segments.push_back(text);
    return segments;
  }
}
  
}  // namespace robot_gui
