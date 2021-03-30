#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <QMainWindow>
#include <QWidget>
#include <QApplication>
#include <QPushButton>
#include <QLineEdit>
#include <ros/ros.h>


class Publisher : public QMainWindow {
  Q_OBJECT

  public:
    Publisher(ros::NodeHandle *n);
    ~Publisher();
    void setParameters(QStringList reset);

  public slots:
    void update_head_tilt(int value);
    void update_head_pan(int value);
    void update_mouth(int value);
    void update_eye_tilt(int value);
    void update_left_eye_pan(int value);
    void update_right_eye_pan(int value);
    void update_head_time(const QString value);
    void reset_head();

    void update_right_arm_8(int value);    
    void update_right_arm_7(int value);    
    void update_right_arm_6(int value);    
    void update_right_arm_5(int value);    
    void update_right_arm_4(int value);    
    void update_right_arm_3(int value);    
    void update_right_arm_2(int value);    
    void update_right_arm_1(int value);  
    void update_right_arm_time(const QString value);
    void reset_right_arm();  

    void update_left_arm_8(int value);  
    void update_left_arm_7(int value);    
    void update_left_arm_6(int value);    
    void update_left_arm_5(int value);    
    void update_left_arm_4(int value);    
    void update_left_arm_3(int value);    
    void update_left_arm_2(int value);    
    void update_left_arm_1(int value);     
    void update_left_arm_time(const QString value); 
    void reset_left_arm();  

    void send_head();
    void send_right_arm();
    void send_left_arm();

  private:
    ros::NodeHandle *nh;
    ros::Publisher head_pub;
    ros::Publisher arms_pub;

    int head_tilt_value;
    int head_pan_value;
    int mouth_value;
    int eye_tilt_value;
    int left_eye_pan_value;
    int right_eye_pan_value;
    int head_time_value;

    int right_arm_8_value;
    int right_arm_7_value;    
    int right_arm_6_value;    
    int right_arm_5_value;    
    int right_arm_4_value;    
    int right_arm_3_value;    
    int right_arm_2_value;    
    int right_arm_1_value;    
    int right_arm_time_value;

    int left_arm_8_value;
    int left_arm_7_value;
    int left_arm_6_value;
    int left_arm_5_value;
    int left_arm_4_value;
    int left_arm_3_value;
    int left_arm_2_value;
    int left_arm_1_value;
    int left_arm_time_value;

    int head_tilt_reset;
    int head_pan_reset;
    int mouth_reset;
    int eye_tilt_reset;
    int left_eye_pan_reset;
    int right_eye_pan_reset;
    int head_time_reset;

    int right_arm_8_reset;
    int right_arm_7_reset;    
    int right_arm_6_reset;    
    int right_arm_5_reset;    
    int right_arm_4_reset;    
    int right_arm_3_reset;    
    int right_arm_2_reset;    
    int right_arm_1_reset;    
    int right_arm_time_reset;

    int left_arm_8_reset;
    int left_arm_7_reset;
    int left_arm_6_reset;
    int left_arm_5_reset;
    int left_arm_4_reset;
    int left_arm_3_reset;
    int left_arm_2_reset;
    int left_arm_1_reset;
    int left_arm_time_reset;
};

#endif //PUBLISHER_H
