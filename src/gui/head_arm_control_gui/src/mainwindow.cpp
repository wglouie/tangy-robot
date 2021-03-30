#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <ros/ros.h>
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <publisher.h>
#include <string>


MainWindow::MainWindow(QWidget *parent, ros::NodeHandle *n) :
    QMainWindow(parent),
    ui(new Ui::MainWindow) {
  ROS_INFO("Interface has been started");
  nh = n;

  ui->setupUi(this);

  ui->head_time_value->setValidator( new QIntValidator(0, 100000, this) );
  ui->right_arm_time_value->setValidator( new QIntValidator(0, 100000, this) );
  ui->left_arm_time_value->setValidator( new QIntValidator(0, 100000, this) );

  publisher = new Publisher(nh);
}

void MainWindow::reset_head() {
  ui->head_tilt_slider->setValue(0);
  ui->head_pan_slider->setValue(0);
  ui->mouth_slider->setValue(0);
  ui->eye_tilt_slider->setValue(0);
  ui->left_eye_pan_slider->setValue(0);
  ui->right_eye_pan_slider->setValue(0);

  ui->head_tilt_value->setNum(0);
  ui->head_pan_value->setNum(0);
  ui->mouth_value->setNum(0);
  ui->eye_tilt_value->setNum(0);
  ui->left_eye_pan_value->setNum(0);
  ui->right_eye_pan_value->setNum(0);

  ui->head_time_value->setText(head_time);
 
  publisher->reset_head();
}

void MainWindow::reset_right_arm() {
  ui->right_arm_8_slider->setValue(0);
  ui->right_arm_7_slider->setValue(0);
  ui->right_arm_6_slider->setValue(0);
  ui->right_arm_5_slider->setValue(0);
  ui->right_arm_4_slider->setValue(0);
  ui->right_arm_3_slider->setValue(0);
  ui->right_arm_2_slider->setValue(0);
  ui->right_arm_1_slider->setValue(0);

  ui->right_arm_8_value->setNum(0);
  ui->right_arm_7_value->setNum(0);
  ui->right_arm_6_value->setNum(0);
  ui->right_arm_5_value->setNum(0);
  ui->right_arm_4_value->setNum(0);
  ui->right_arm_3_value->setNum(0);
  ui->right_arm_2_value->setNum(0);
  ui->right_arm_1_value->setNum(0);

  ui->right_arm_time_value->setText(right_arm_time);

  publisher->reset_right_arm();
}

void MainWindow::reset_left_arm() {
  ui->left_arm_8_slider->setValue(0);
  ui->left_arm_7_slider->setValue(0);
  ui->left_arm_6_slider->setValue(0);
  ui->left_arm_5_slider->setValue(0);
  ui->left_arm_4_slider->setValue(0);
  ui->left_arm_3_slider->setValue(0);
  ui->left_arm_2_slider->setValue(0);
  ui->left_arm_1_slider->setValue(0);

  ui->left_arm_8_value->setNum(0);
  ui->left_arm_7_value->setNum(0);
  ui->left_arm_6_value->setNum(0);
  ui->left_arm_5_value->setNum(0);
  ui->left_arm_4_value->setNum(0);
  ui->left_arm_3_value->setNum(0);
  ui->left_arm_2_value->setNum(0);
  ui->left_arm_1_value->setNum(0);

  ui->left_arm_time_value->setText(left_arm_time);

  publisher->reset_left_arm();
}

void MainWindow::setParameters(QStringList max, QStringList min, QStringList reset) {
  ui->head_tilt_slider->setRange(min[0].toInt() - reset[0].toInt(), max[0].toInt() - reset[0].toInt());
  ui->head_pan_slider->setRange(min[1].toInt() - reset[1].toInt(), max[1].toInt() - reset[1].toInt());
  ui->mouth_slider->setRange(min[2].toInt() - reset[2].toInt(), max[2].toInt() - reset[2].toInt());
  ui->eye_tilt_slider->setRange(min[3].toInt() - reset[3].toInt(), max[3].toInt() - reset[3].toInt());
  ui->left_eye_pan_slider->setRange(min[4].toInt() - reset[4].toInt(), max[4].toInt() - reset[4].toInt());
  ui->right_eye_pan_slider->setRange(min[5].toInt() - reset[5].toInt(), max[5].toInt() - reset[5].toInt());
  ui->head_time_value->setText(reset[6]);

  ui->right_arm_8_slider->setRange(min[6].toInt() - reset[7].toInt(), max[6].toInt() - reset[7].toInt());
  ui->right_arm_7_slider->setRange(min[7].toInt() - reset[8].toInt(), max[7].toInt() - reset[8].toInt());
  ui->right_arm_6_slider->setRange(min[8].toInt() - reset[9].toInt(), max[8].toInt() - reset[9].toInt());
  ui->right_arm_5_slider->setRange(min[9].toInt() - reset[10].toInt(), max[9].toInt() - reset[10].toInt());
  ui->right_arm_4_slider->setRange(min[10].toInt() - reset[11].toInt(), max[10].toInt() - reset[11].toInt());
  ui->right_arm_3_slider->setRange(min[11].toInt() - reset[12].toInt(), max[11].toInt() - reset[12].toInt());
  ui->right_arm_2_slider->setRange(min[12].toInt() - reset[13].toInt(), max[12].toInt() - reset[13].toInt());
  ui->right_arm_1_slider->setRange(min[13].toInt() - reset[14].toInt(), max[13].toInt() - reset[14].toInt());
  ui->right_arm_time_value->setText(reset[15]);

  ui->left_arm_8_slider->setRange(min[14].toInt() - reset[16].toInt(), max[14].toInt() - reset[16].toInt());
  ui->left_arm_7_slider->setRange(min[15].toInt() - reset[17].toInt(), max[15].toInt() - reset[17].toInt());
  ui->left_arm_6_slider->setRange(min[16].toInt() - reset[18].toInt(), max[16].toInt() - reset[18].toInt());
  ui->left_arm_5_slider->setRange(min[17].toInt() - reset[19].toInt(), max[17].toInt() - reset[19].toInt());
  ui->left_arm_4_slider->setRange(min[18].toInt() - reset[20].toInt(), max[18].toInt() - reset[20].toInt());
  ui->left_arm_3_slider->setRange(min[19].toInt() - reset[21].toInt(), max[19].toInt() - reset[21].toInt());
  ui->left_arm_2_slider->setRange(min[20].toInt() - reset[22].toInt(), max[20].toInt() - reset[22].toInt());
  ui->left_arm_1_slider->setRange(min[21].toInt() - reset[23].toInt(), max[21].toInt() - reset[23].toInt());
  ui->left_arm_time_value->setText(reset[24]);

  head_time = reset[6];
  right_arm_time = reset[15];
  left_arm_time = reset[24];

  publisher->setParameters(reset);
}

void MainWindow::setConnections() {
//HEAD
  //update the number in the vlaue portion
  QObject::connect(ui->head_tilt_slider, SIGNAL(sliderMoved(int)), ui->head_tilt_value, SLOT(setNum(int)));
  QObject::connect(ui->head_pan_slider, SIGNAL(sliderMoved(int)), ui->head_pan_value, SLOT(setNum(int)));
  QObject::connect(ui->mouth_slider, SIGNAL(sliderMoved(int)), ui->mouth_value, SLOT(setNum(int)));
  QObject::connect(ui->eye_tilt_slider, SIGNAL(sliderMoved(int)), ui->eye_tilt_value, SLOT(setNum(int)));
  QObject::connect(ui->left_eye_pan_slider, SIGNAL(sliderMoved(int)), ui->left_eye_pan_value, SLOT(setNum(int)));
  QObject::connect(ui->right_eye_pan_slider, SIGNAL(sliderMoved(int)), ui->right_eye_pan_value, SLOT(setNum(int)));

  //update the number in the vlaue portion
  QObject::connect(ui->head_tilt_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_head_tilt(int)));
  QObject::connect(ui->head_pan_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_head_pan(int)));
  QObject::connect(ui->mouth_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_mouth(int)));
  QObject::connect(ui->eye_tilt_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_eye_tilt(int)));
  QObject::connect(ui->left_eye_pan_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_left_eye_pan(int)));
  QObject::connect(ui->right_eye_pan_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_right_eye_pan(int)));
  QObject::connect(ui->head_time_value, SIGNAL(textChanged(const QString)), publisher, SLOT(update_head_time(const QString)));

  //send the number to the robot
  QObject::connect(ui->head_tilt_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_head()));
  QObject::connect(ui->head_pan_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_head()));
  QObject::connect(ui->mouth_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_head()));
  QObject::connect(ui->eye_tilt_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_head()));
  QObject::connect(ui->left_eye_pan_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_head()));
  QObject::connect(ui->right_eye_pan_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_head()));

  //reset
  QObject::connect(ui->reset_head_button, SIGNAL(clicked()), this, SLOT(reset_head()));
  QObject::connect(ui->reset_head_button, SIGNAL(clicked()), publisher, SLOT(send_head()));

//RIGHT ARM
  //update the number in the vlaue portion
  QObject::connect(ui->right_arm_8_slider, SIGNAL(sliderMoved(int)), ui->right_arm_8_value, SLOT(setNum(int)));
  QObject::connect(ui->right_arm_7_slider, SIGNAL(sliderMoved(int)), ui->right_arm_7_value, SLOT(setNum(int)));
  QObject::connect(ui->right_arm_6_slider, SIGNAL(sliderMoved(int)), ui->right_arm_6_value, SLOT(setNum(int)));
  QObject::connect(ui->right_arm_5_slider, SIGNAL(sliderMoved(int)), ui->right_arm_5_value, SLOT(setNum(int)));
  QObject::connect(ui->right_arm_4_slider, SIGNAL(sliderMoved(int)), ui->right_arm_4_value, SLOT(setNum(int)));
  QObject::connect(ui->right_arm_3_slider, SIGNAL(sliderMoved(int)), ui->right_arm_3_value, SLOT(setNum(int)));
  QObject::connect(ui->right_arm_2_slider, SIGNAL(sliderMoved(int)), ui->right_arm_2_value, SLOT(setNum(int)));
  QObject::connect(ui->right_arm_1_slider, SIGNAL(sliderMoved(int)), ui->right_arm_1_value, SLOT(setNum(int)));

  //update the number in the vlaue portion
  QObject::connect(ui->right_arm_8_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_right_arm_8(int)));
  QObject::connect(ui->right_arm_7_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_right_arm_7(int)));
  QObject::connect(ui->right_arm_6_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_right_arm_6(int)));
  QObject::connect(ui->right_arm_5_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_right_arm_5(int)));
  QObject::connect(ui->right_arm_4_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_right_arm_4(int)));
  QObject::connect(ui->right_arm_3_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_right_arm_3(int)));
  QObject::connect(ui->right_arm_2_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_right_arm_2(int)));
  QObject::connect(ui->right_arm_1_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_right_arm_1(int)));
  QObject::connect(ui->right_arm_time_value, SIGNAL(textChanged(const QString)), publisher, SLOT(update_right_arm_time(const QString)));

  //send the number to the robot
  QObject::connect(ui->right_arm_8_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_right_arm()));
  QObject::connect(ui->right_arm_7_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_right_arm()));
  QObject::connect(ui->right_arm_6_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_right_arm()));
  QObject::connect(ui->right_arm_5_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_right_arm()));
  QObject::connect(ui->right_arm_4_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_right_arm()));
  QObject::connect(ui->right_arm_3_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_right_arm()));
  QObject::connect(ui->right_arm_2_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_right_arm()));
  QObject::connect(ui->right_arm_1_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_right_arm()));

  //reset
  QObject::connect(ui->reset_right_arm_button, SIGNAL(clicked()), this, SLOT(reset_right_arm()));
  QObject::connect(ui->reset_right_arm_button, SIGNAL(clicked()), publisher, SLOT(send_right_arm()));

//LEFT ARM
  //update the number in the vlaue portion
  QObject::connect(ui->left_arm_8_slider, SIGNAL(sliderMoved(int)), ui->left_arm_8_value, SLOT(setNum(int)));
  QObject::connect(ui->left_arm_7_slider, SIGNAL(sliderMoved(int)), ui->left_arm_7_value, SLOT(setNum(int)));
  QObject::connect(ui->left_arm_6_slider, SIGNAL(sliderMoved(int)), ui->left_arm_6_value, SLOT(setNum(int)));
  QObject::connect(ui->left_arm_5_slider, SIGNAL(sliderMoved(int)), ui->left_arm_5_value, SLOT(setNum(int)));
  QObject::connect(ui->left_arm_4_slider, SIGNAL(sliderMoved(int)), ui->left_arm_4_value, SLOT(setNum(int)));
  QObject::connect(ui->left_arm_3_slider, SIGNAL(sliderMoved(int)), ui->left_arm_3_value, SLOT(setNum(int)));
  QObject::connect(ui->left_arm_2_slider, SIGNAL(sliderMoved(int)), ui->left_arm_2_value, SLOT(setNum(int)));
  QObject::connect(ui->left_arm_1_slider, SIGNAL(sliderMoved(int)), ui->left_arm_1_value, SLOT(setNum(int)));
 
  //update the number in the vlaue portion
  QObject::connect(ui->left_arm_8_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_left_arm_8(int)));
  QObject::connect(ui->left_arm_7_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_left_arm_7(int)));
  QObject::connect(ui->left_arm_6_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_left_arm_6(int)));
  QObject::connect(ui->left_arm_5_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_left_arm_5(int)));
  QObject::connect(ui->left_arm_4_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_left_arm_7(int)));
  QObject::connect(ui->left_arm_3_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_left_arm_3(int)));
  QObject::connect(ui->left_arm_2_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_left_arm_2(int)));
  QObject::connect(ui->left_arm_1_slider, SIGNAL(sliderMoved(int)), publisher, SLOT(update_left_arm_1(int)));
  QObject::connect(ui->left_arm_time_value, SIGNAL(textChanged(const QString)), publisher, SLOT(update_left_arm_time(const QString)));

  //send the number to the robot
  QObject::connect(ui->left_arm_8_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_left_arm()));
  QObject::connect(ui->left_arm_7_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_left_arm()));
  QObject::connect(ui->left_arm_6_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_left_arm()));
  QObject::connect(ui->left_arm_5_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_left_arm()));
  QObject::connect(ui->left_arm_4_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_left_arm()));
  QObject::connect(ui->left_arm_3_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_left_arm()));
  QObject::connect(ui->left_arm_2_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_left_arm()));
  QObject::connect(ui->left_arm_1_slider, SIGNAL(sliderReleased()), publisher, SLOT(send_left_arm()));
  
  //reset
  QObject::connect(ui->reset_left_arm_button, SIGNAL(clicked()), this, SLOT(reset_left_arm()));
  QObject::connect(ui->reset_left_arm_button, SIGNAL(clicked()), publisher, SLOT(send_left_arm()));
}

MainWindow::~MainWindow() {
    ROS_INFO("Interface has been closed");
    delete ui;
}
