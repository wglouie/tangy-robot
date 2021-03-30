#include <publisher.h>
#include <string>
#include <ros/ros.h>
#include <head_arm_control_gui/Slider.h>
#include <drrobot_h20_player/HeadCmd.h>
#include <drrobot_h20_arm_player/ArmCmd.h>

#define NOCONTROL -32768

Publisher::Publisher(ros::NodeHandle *n) {
  ROS_INFO("Publisher created");
  nh = n;
  //slider_pub = n->advertise<head_arm_control_gui::Slider>("cmd_head", 50);
  head_pub = n->advertise<drrobot_h20_player::HeadCmd>("cmd_pose_head", 100);
  arms_pub = n->advertise<drrobot_h20_arm_player::ArmCmd>("cmd_arm", 100);
}

Publisher::~Publisher() {
  ROS_INFO("Publisher closed");
}

void Publisher::update_head_tilt(int value) {
  head_tilt_value = head_tilt_reset + value;
}

void Publisher::update_head_pan(int value) {
  head_pan_value = head_pan_reset + value;
}

void Publisher::update_mouth(int value) {
  mouth_value = mouth_reset + value;
}

void Publisher::update_eye_tilt(int value) {
  eye_tilt_value = eye_tilt_reset + value;
}

void Publisher::update_left_eye_pan(int value) {
  left_eye_pan_value = left_eye_pan_reset + value;
}

void Publisher::update_right_eye_pan(int value) {
  right_eye_pan_value = right_eye_pan_reset + value;
}

void Publisher::update_head_time(const QString value) { 
  head_time_value = value.toInt();
}

void Publisher::reset_head() {
  head_tilt_value = head_tilt_reset;
  head_pan_value = head_pan_reset;
  mouth_value = mouth_reset;
  eye_tilt_value = eye_tilt_reset;
  left_eye_pan_value = left_eye_pan_reset;
  right_eye_pan_value = right_eye_pan_reset;
  head_time_value = head_time_reset;
}

void Publisher::update_right_arm_8(int value) {
  right_arm_8_value = right_arm_8_reset + value;
}

void Publisher::update_right_arm_7(int value) {
  right_arm_7_value = right_arm_7_reset + value;
}

void Publisher::update_right_arm_6(int value) {
  right_arm_6_value = right_arm_6_reset + value;
}

void Publisher::update_right_arm_5(int value) {
  right_arm_5_value = right_arm_5_reset + value;
}

void Publisher::update_right_arm_4(int value) {
  right_arm_4_value = right_arm_4_reset + value;
}

void Publisher::update_right_arm_3(int value) {
  right_arm_3_value = right_arm_3_reset + value;
}

void Publisher::update_right_arm_2(int value) {
  right_arm_2_value = right_arm_2_reset + value;
}

void Publisher::update_right_arm_1(int value) {
	std::cout << "right_arm_1" << std::endl;
  right_arm_1_value = right_arm_1_reset + value;
}

void Publisher::update_right_arm_time(const QString value) { 
  right_arm_time_value = value.toInt();
}

void Publisher::reset_right_arm() {
  right_arm_8_value = right_arm_8_reset;
  right_arm_7_value = right_arm_7_reset;    
  right_arm_6_value = right_arm_6_reset;    
  right_arm_5_value = right_arm_5_reset;    
  right_arm_4_value = right_arm_4_reset;    
  right_arm_3_value = right_arm_3_reset;    
  right_arm_2_value = right_arm_2_reset;    
  right_arm_1_value = right_arm_1_reset;    
  right_arm_time_value = right_arm_time_reset;
}

void Publisher::update_left_arm_8(int value) {
  left_arm_8_value = left_arm_8_reset + value;
}
 
void Publisher::update_left_arm_7(int value) {
  left_arm_7_value = left_arm_7_reset + value;
}

void Publisher::update_left_arm_6(int value) {
  left_arm_6_value = left_arm_6_reset + value;
}

void Publisher::update_left_arm_5(int value) {
  left_arm_5_value = left_arm_5_reset + value;
}

void Publisher::update_left_arm_4(int value) {
  left_arm_4_value = left_arm_4_reset + value;
}

void Publisher::update_left_arm_3(int value) {
  left_arm_3_value = left_arm_3_reset + value;
}

void Publisher::update_left_arm_2(int value) {
  left_arm_2_value = left_arm_2_reset + value;
}

void Publisher::update_left_arm_1(int value) {
  left_arm_1_value = left_arm_1_reset + value;
}

void Publisher::update_left_arm_time(const QString value) { 
  left_arm_time_value = value.toInt();
}

void Publisher::reset_left_arm() {
  left_arm_8_value = left_arm_8_reset;
  left_arm_7_value = left_arm_7_reset;
  left_arm_6_value = left_arm_6_reset;
  left_arm_5_value = left_arm_5_reset;
  left_arm_4_value = left_arm_4_reset;
  left_arm_3_value = left_arm_3_reset;
  left_arm_2_value = left_arm_2_reset;
  left_arm_1_value = left_arm_1_reset;

  left_arm_time_value = left_arm_time_reset;
}

void Publisher::send_head() {
  drrobot_h20_player::HeadCmd message;

  message.neck_x_rotation = head_tilt_value;
  message.neck_z_rotation = head_pan_value;
  message.mouth = mouth_value;
  message.upper_head = eye_tilt_value;
  message.left_eye = left_eye_pan_value;
  message.right_eye = right_eye_pan_value;
  message.flag = head_time_value;

  head_pub.publish(message);
}

void Publisher::send_right_arm() {
  drrobot_h20_arm_player::ArmCmd message;

  message.right_arm[0] = right_arm_1_value;
  message.right_arm[1] = right_arm_2_value;
  message.right_arm[2] = right_arm_3_value;
  message.right_arm[3] = right_arm_4_value;
  message.right_arm[4] = right_arm_5_value;
  message.right_arm[5] = right_arm_6_value;
  message.right_arm[6] = right_arm_7_value;
  message.right_arm[7] = right_arm_8_value;
  
  message.left_arm[0] = left_arm_1_value;
  message.left_arm[1] = left_arm_2_value;
  message.left_arm[2] = left_arm_3_value;
  message.left_arm[3] = left_arm_4_value;
  message.left_arm[4] = left_arm_5_value;
  message.left_arm[5] = left_arm_6_value;
  message.left_arm[6] = left_arm_7_value;
  message.left_arm[7] = left_arm_8_value;

  message.vel = right_arm_time_value;
  message.speed = false;

  arms_pub.publish(message);
}

void Publisher::send_left_arm() {
  drrobot_h20_arm_player::ArmCmd message;

  message.right_arm[0] = right_arm_1_value;
  message.right_arm[1] = right_arm_2_value;
  message.right_arm[2] = right_arm_3_value;
  message.right_arm[3] = right_arm_4_value;
  message.right_arm[4] = right_arm_5_value;
  message.right_arm[5] = right_arm_6_value;
  message.right_arm[6] = right_arm_7_value;
  message.right_arm[7] = right_arm_8_value;
  
  message.left_arm[0] = left_arm_1_value;
  message.left_arm[1] = left_arm_2_value;
  message.left_arm[2] = left_arm_3_value;
  message.left_arm[3] = left_arm_4_value;
  message.left_arm[4] = left_arm_5_value;
  message.left_arm[5] = left_arm_6_value;
  message.left_arm[6] = left_arm_7_value;
  message.left_arm[7] = left_arm_8_value;

  message.vel = left_arm_time_value;
  message.speed = false;

  arms_pub.publish(message);
}

void Publisher::setParameters(QStringList reset) {
    head_tilt_reset = reset[0].toInt();
    head_pan_reset = reset[1].toInt();
    mouth_reset = reset[2].toInt();
    eye_tilt_reset = reset[3].toInt();
    left_eye_pan_reset = reset[4].toInt();
    right_eye_pan_reset = reset[5].toInt();
    head_time_reset = reset[6].toInt();

    right_arm_8_reset = reset[7].toInt();
    right_arm_7_reset = reset[8].toInt();
    right_arm_6_reset = reset[9].toInt();
    right_arm_5_reset = reset[10].toInt();
    right_arm_4_reset = reset[11].toInt();
    right_arm_3_reset = reset[12].toInt();
    right_arm_2_reset = reset[13].toInt();
    right_arm_1_reset = reset[14].toInt();
    right_arm_time_reset = reset[15].toInt();

    left_arm_8_reset = reset[16].toInt();
    left_arm_7_reset = reset[17].toInt();
    left_arm_6_reset = reset[18].toInt();
    left_arm_5_reset = reset[19].toInt();
    left_arm_4_reset = reset[20].toInt();
    left_arm_3_reset = reset[21].toInt();
    left_arm_2_reset = reset[22].toInt();
    left_arm_1_reset = reset[23].toInt();
    left_arm_time_reset = reset[24].toInt();
 
    head_tilt_value = reset[0].toInt();
    head_pan_value = reset[1].toInt();
    mouth_value = reset[2].toInt();
    eye_tilt_value = reset[3].toInt();
    left_eye_pan_value = reset[4].toInt();
    right_eye_pan_value = reset[5].toInt();
    head_time_value = reset[6].toInt();

    right_arm_8_value = reset[7].toInt();
    right_arm_7_value = reset[8].toInt();
    right_arm_6_value = reset[9].toInt();
    right_arm_5_value = reset[10].toInt();
    right_arm_4_value = reset[11].toInt();
    right_arm_3_value = reset[12].toInt();
    right_arm_2_value = reset[13].toInt();
    right_arm_1_value = reset[14].toInt();
    right_arm_time_value = reset[15].toInt();

    left_arm_8_value = reset[16].toInt();
    left_arm_7_value = reset[17].toInt();
    left_arm_6_value = reset[18].toInt();
    left_arm_5_value = reset[19].toInt();
    left_arm_4_value = reset[20].toInt();
    left_arm_3_value = reset[21].toInt();
    left_arm_2_value = reset[22].toInt();
    left_arm_1_value = reset[23].toInt();
    left_arm_time_value = reset[24].toInt();
}










