//============================================================================
// Name        : neck_handler.cpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : This program controls the neck of the robot.
//               Reset the neck position with reset_neck_pos()
/*<servo id="head_tilt"><max value="4440"/><reset value="4100"/><min value="3050"/></servo><servo id="head_pan"><max value="4550"/><reset value="3450"/><min value="2350"/>*/

//Amir changed the neck_increment_dir fnction completely, also the constructor may be changed.
//===========================================================================

#include <ros/ros.h>
#include <string>
#include <drrobot_h20_player/HeadCmd.h>
#include "neck_handler.h"
#include <math.h>
#define ROTATION_DEFINITION 50
#define ROTATION_TIME 100000
#define MAX_VERTICAL_CHANGE -600

  
neck_handler::neck_handler(std::string name)
{
   
   pub_head1 = nh_.advertise<drrobot_h20_player::HeadCmd>("/cmd_pose_head", 1000);
   sleep(2);
   sub_head = nh_.subscribe("/cmd_pose_head",100, &neck_handler::record_last_posCB, this);
   
   last_neck_x_rotation=NECK_TILT;
   last_neck_z_rotation=NECK_PAN;
   last_mouth=MOUTH;
   last_upper_head=UPPER_HEAD;
   last_left_eye=LEFT_EYE;
   last_right_eye=RIGHT_EYE;
   stored_neck_x=NECK_TILT;
   stored_neck_z=NECK_PAN;
   ROS_INFO("Neck starting out with positions neck tilt, neck pan, mouth, upper head, left eye, right eye = [%d,%d,%d,%d,%d,%d]", last_neck_x_rotation, last_neck_z_rotation, last_mouth, last_upper_head, last_left_eye, last_right_eye);
   previousmovement=0;
}
//Callback way to record last position
neck_handler::~neck_handler()
{
}
void neck_handler::record_last_posCB(drrobot_h20_player::HeadCmd msg)
{
   last_neck_x_rotation=msg.neck_x_rotation;
   last_neck_z_rotation=msg.neck_z_rotation;
   last_mouth=msg.mouth;
   last_upper_head=msg.upper_head;
   last_left_eye=msg.left_eye;
   last_right_eye=msg.right_eye;
}
//Non-callback way to record last position
void neck_handler::record_last_pos(drrobot_h20_player::HeadCmd msg)
{
   last_neck_x_rotation=msg.neck_x_rotation;
   last_neck_z_rotation=msg.neck_z_rotation;
   last_mouth=msg.mouth;
   last_upper_head=msg.upper_head;
   last_left_eye=msg.left_eye;
   last_right_eye=msg.right_eye;
}

/*Reset head position*/
void neck_handler::reset_neck_pos()
{
    // pub_head = nh_.advertise<drrobot_h20_player::HeadCmd>("/cmd_head", 1);
    drrobot_h20_player::HeadCmd cmdhead_;

    int neck_x_rotation_ = NECK_TILT;
    int neck_z_rotation_= NECK_PAN;
    int mouth_= MOUTH;
    int upper_head_ = UPPER_HEAD;
    int left_eye_ = LEFT_EYE;
    int right_eye_ = RIGHT_EYE;


    cmdhead_.neck_x_rotation = neck_x_rotation_; //Head Tilt
    cmdhead_.neck_z_rotation = neck_z_rotation_; //Head Pan
    cmdhead_.mouth = mouth_;
    cmdhead_.upper_head = upper_head_;
    cmdhead_.left_eye = left_eye_;
    cmdhead_.right_eye = right_eye_;
    cmdhead_.flag = 700;

    pub_head1.publish(cmdhead_);
    pub_head1.publish(cmdhead_);
    pub_head1.publish(cmdhead_);
    // pub_head2.publish(cmdhead_);
    record_last_pos(cmdhead_);
}

void neck_handler::rand_neck_pos(){
    drrobot_h20_player::HeadCmd cmdhead_;

    int neck_x_rotation_ = NECK_TILT;
    int neck_z_rotation_= NECK_PAN;
    int mouth_= MOUTH;
    int upper_head_ = UPPER_HEAD;
    int left_eye_ = LEFT_EYE;
    int right_eye_ = RIGHT_EYE;
    
    while(abs(neck_z_rotation_-last_neck_z_rotation)<100){
        int rand_pan=(rand()% 1000)-500;
        neck_z_rotation_=rand_pan+NECK_PAN;
    }


    cmdhead_.neck_x_rotation = neck_x_rotation_; //Head Tilt
    cmdhead_.neck_z_rotation = neck_z_rotation_; //Head Pan
    cmdhead_.mouth = mouth_;
    cmdhead_.upper_head = upper_head_;
    cmdhead_.left_eye = left_eye_;
    cmdhead_.right_eye = right_eye_;
    cmdhead_.flag = 1000;


    pub_head1.publish(cmdhead_);
    sleep(1);
    pub_head1.publish(cmdhead_);
    record_last_pos(cmdhead_);
}

void neck_handler::look_down(){
    drrobot_h20_player::HeadCmd cmdhead_;

    int neck_x_rotation_ = NECK_TILT-50;
    int neck_z_rotation_= NECK_PAN;
    int mouth_= MOUTH;
    int upper_head_ = UPPER_HEAD;
    int left_eye_ = LEFT_EYE;
    int right_eye_ = RIGHT_EYE;
    
    cmdhead_.neck_x_rotation = neck_x_rotation_; //Head Tilt
    cmdhead_.neck_z_rotation = neck_z_rotation_; //Head Pan
    cmdhead_.mouth = mouth_;
    cmdhead_.upper_head = upper_head_;
    cmdhead_.left_eye = left_eye_;
    cmdhead_.right_eye = right_eye_;
    cmdhead_.flag = 1000;


    pub_head1.publish(cmdhead_);
    sleep(1);
    pub_head1.publish(cmdhead_);
    sleep(1);
    pub_head1.publish(cmdhead_);
    
}

void neck_handler::increment_dir(int leftrightgoal, int updowngoal)
{
    int neck_x_rotation_ = 0;
    int neck_z_rotation_= 0;
    int mouth_= 0;
    int upper_head_ = 0;
    int left_eye_ = 0;
    int right_eye_ = 0;
    int flag_ = 0;
    int goalvertical,goalsidetoside;

    pub_ = n_.advertise<drrobot_h20_player::HeadCmd>("cmd_head", 1);

    cmdhead_.neck_x_rotation = updowngoal;
    cmdhead_.neck_z_rotation = -(leftrightgoal);
    //cmdhead_.neck_z_rotation = -(leftrightgoal+37);
    cmdhead_.mouth = mouth_;
    cmdhead_.upper_head = upper_head_;
    cmdhead_.left_eye = left_eye_;
    cmdhead_.right_eye = right_eye_;
    cmdhead_.flag = flag_;
    pub_.publish(cmdhead_);
}

void neck_handler::look_at_card_1(){
    stored_neck_x=last_neck_x_rotation;
    stored_neck_z=last_neck_z_rotation;
    int neck_x_rotation_= NECK_TILT-400;
    int neck_z_rotation_=last_neck_z_rotation;
    int mouth_=last_mouth;
    int upper_head_=last_upper_head;
    int left_eye_=last_left_eye;
    int right_eye_=last_right_eye;
    
    drrobot_h20_player::HeadCmd cmdhead_;

    cmdhead_.neck_x_rotation = neck_x_rotation_; //Head Tilt
    cmdhead_.neck_z_rotation = neck_z_rotation_; //Head Pan
    cmdhead_.mouth = mouth_;
    cmdhead_.upper_head = upper_head_;
    cmdhead_.left_eye = left_eye_;
    cmdhead_.right_eye = right_eye_;
    cmdhead_.flag = 1000;


    pub_head1.publish(cmdhead_);
    sleep(1);
    pub_head1.publish(cmdhead_);
    
}
void neck_handler::look_at_card_2(){
  
    int neck_x_rotation_= NECK_TILT-300;
    int neck_z_rotation_=last_neck_z_rotation;
    int mouth_=last_mouth;
    int upper_head_=last_upper_head;
    int left_eye_=last_left_eye;
    int right_eye_=last_right_eye;
    
    drrobot_h20_player::HeadCmd cmdhead_;

    cmdhead_.neck_x_rotation = neck_x_rotation_; //Head Tilt
    cmdhead_.neck_z_rotation = neck_z_rotation_; //Head Pan
    cmdhead_.mouth = mouth_;
    cmdhead_.upper_head = upper_head_;
    cmdhead_.left_eye = left_eye_;
    cmdhead_.right_eye = right_eye_;
    cmdhead_.flag = 1000;


    pub_head1.publish(cmdhead_);
    sleep(1);
    pub_head1.publish(cmdhead_);
    
}
void neck_handler::look_at_card_3(){
  
    int neck_x_rotation_= NECK_TILT-500;
    int neck_z_rotation_=last_neck_z_rotation;
    int mouth_=last_mouth;
    int upper_head_=last_upper_head;
    int left_eye_=last_left_eye;
    int right_eye_=last_right_eye;
    
    drrobot_h20_player::HeadCmd cmdhead_;

    cmdhead_.neck_x_rotation = neck_x_rotation_; //Head Tilt
    cmdhead_.neck_z_rotation = neck_z_rotation_; //Head Pan
    cmdhead_.mouth = mouth_;
    cmdhead_.upper_head = upper_head_;
    cmdhead_.left_eye = left_eye_;
    cmdhead_.right_eye = right_eye_;
    cmdhead_.flag = 1000;


    pub_head1.publish(cmdhead_);
    sleep(1);
    pub_head1.publish(cmdhead_);
    
}

void neck_handler::look_back_to_person(){

    drrobot_h20_player::HeadCmd cmdhead_;

    int neck_x_rotation_= stored_neck_x;
    int neck_z_rotation_=stored_neck_z;
    int mouth_=last_mouth;
    int upper_head_=last_upper_head;
    int left_eye_=last_left_eye;
    int right_eye_=last_right_eye;

    cmdhead_.neck_x_rotation = neck_x_rotation_; //Head Tilt
    cmdhead_.neck_z_rotation = neck_z_rotation_; //Head Pan
    cmdhead_.mouth = mouth_;
    cmdhead_.upper_head = upper_head_;
    cmdhead_.left_eye = left_eye_;
    cmdhead_.right_eye = right_eye_;
    cmdhead_.flag = 1000;


    pub_head1.publish(cmdhead_);
    sleep(1);
    pub_head1.publish(cmdhead_);
    record_last_pos(cmdhead_);
}

//Neck attempts to turn the provided angles from its current position
void neck_handler::turn(float horiz_ang, float vert_ang){
    using namespace std;
    drrobot_h20_player::HeadCmd cmdhead_;
    cmdhead_.mouth = last_mouth;
    cmdhead_.upper_head = last_upper_head;
    cmdhead_.flag = 500;
    int init_x=last_neck_x_rotation;
    int init_z=last_neck_z_rotation;
    int incr_time=125000;
    int max_eye_pan=1000;
    
    if(horiz_ang!=0||vert_ang!=0){
      int pan_=horizAngToServo(horiz_ang);
      int tilt_=vertAngToServo(vert_ang);
      
      //Constants
      int incr=100;
      int p_num=max(sqrt(pow(pan_/incr,2)) , double (8));
      
      int eye_pan;
      if(pan_>0){
        eye_pan=min(max_eye_pan, p_num*incr)*2.5;
      }else if(pan_<0){
        eye_pan=-min(sqrt(pow(max_eye_pan,2)), sqrt(pow(p_num*incr,2)))*1.5;
      }else{
        eye_pan=0;
      }
      ROS_INFO("Pan_=[%d], tilt_=[%d], eye_pan =[%d], p_num=[%d]", pan_, tilt_, eye_pan, p_num);
      
      
      for(int i = 0; i < p_num; i++){
          cmdhead_.neck_x_rotation = tilt_/p_num * i + NECK_TILT;
          cmdhead_.neck_z_rotation = pan_/p_num * i+NECK_PAN;
          cmdhead_.left_eye = LEFT_EYE - int ( double (eye_pan) * double (p_num-i)/ double (p_num));
          cmdhead_.right_eye = RIGHT_EYE - int ( double (eye_pan) * double (p_num-i)/ double (p_num));
          pub_head1.publish(cmdhead_);
          usleep(incr_time);
          //ROS_INFO("Sending servo goals [x,z,m,uh,le,re]=[%d,%d,%d,%d,%d,%d]",cmdhead_.neck_x_rotation, cmdhead_.neck_z_rotation,cmdhead_.mouth, cmdhead_.upper_head,cmdhead_.left_eye,cmdhead_.right_eye);
          record_last_pos(cmdhead_);
      }
      cmdhead_.neck_x_rotation = tilt_ + NECK_TILT;
      cmdhead_.neck_z_rotation = pan_ + NECK_PAN;
      cmdhead_.left_eye = last_left_eye;
      cmdhead_.right_eye = last_right_eye;
      pub_head1.publish(cmdhead_);
      record_last_pos(cmdhead_);
      usleep(incr_time);
      //ROS_INFO("Sending servo goals [x,z,m,uh,le,re]=[%d,%d,%d,%d,%d,%d]",cmdhead_.neck_x_rotation, cmdhead_.neck_z_rotation,cmdhead_.mouth, cmdhead_.upper_head,cmdhead_.left_eye,cmdhead_.right_eye);
    }else{
      int p_num=8;
      int incrz = (NECK_PAN - init_z)/p_num;
      int incrx = (NECK_TILT - init_x)/p_num;
      int eye_pan;

      if(incrz>0){
        eye_pan=min(max_eye_pan, p_num*incrz)*3;
      }else{
        eye_pan=-min(sqrt(pow(max_eye_pan,2)), sqrt(pow(p_num*incrz,2)))*1.5;
      }
      ROS_INFO("Pan_=[%d], tilt_=[%d], eye_pan =[%d], p_num=[%d]", (NECK_PAN - init_z), (NECK_TILT - init_x), eye_pan, p_num);
      
      for(int i = 0; i < p_num; i++){
          cmdhead_.neck_x_rotation = init_x+incrx*i;
          cmdhead_.neck_z_rotation = init_z + incrz*i;
          cmdhead_.left_eye = LEFT_EYE - int ( double (eye_pan) * double (p_num-i)/ double (p_num));
          cmdhead_.right_eye = RIGHT_EYE - int ( double (eye_pan) * double (p_num-i)/ double (p_num));
          pub_head1.publish(cmdhead_);
          usleep(incr_time);
          //ROS_INFO("Sending servo goals [x,z,m,uh,le,re]=[%d,%d,%d,%d,%d,%d]",cmdhead_.neck_x_rotation, cmdhead_.neck_z_rotation,cmdhead_.mouth, cmdhead_.upper_head,cmdhead_.left_eye,cmdhead_.right_eye);
          record_last_pos(cmdhead_);
      }
      usleep(incr_time*3);
    }
    
    record_last_pos(cmdhead_);
}

void neck_handler::turn_limited(float horiz_ang, float vert_ang){
    drrobot_h20_player::HeadCmd cmdhead_;
    int neck_x_rotation_= last_neck_x_rotation;
    int neck_z_rotation_=last_neck_z_rotation;
    int mouth_=last_mouth;
    int upper_head_=last_upper_head;
    int left_eye_=last_left_eye;
    int right_eye_=last_right_eye;
    
    neck_x_rotation_=neck_x_rotation_+vertAngToServo(vert_ang);
    neck_z_rotation_=neck_z_rotation_+horizAngToServo(horiz_ang);
    

    cmdhead_.neck_x_rotation = neck_x_rotation_; //Head Tilt
    cmdhead_.neck_z_rotation = neck_z_rotation_; //Head Pan
    cmdhead_.mouth = mouth_;
    cmdhead_.upper_head = upper_head_;
    cmdhead_.left_eye = left_eye_;
    cmdhead_.right_eye = right_eye_;
    cmdhead_.flag = 500;


    pub_head1.publish(cmdhead_);
    sleep(1);
    pub_head1.publish(cmdhead_);

}

// A servo value of 4500 for head pan is 45 deg to the right of the robot, 2400 is 45 deg to the left
int neck_handler::horizAngToServo(float ang){
  int servo;
  servo=int (ang*1050/45);
  
  return servo;
}
int neck_handler::vertAngToServo(float ang){
  int servo;
  servo=int (ang*500/45);
  return servo;
  
}

void neck_handler::nod(){
    drrobot_h20_player::HeadCmd cmdhead_;
    int init_x= last_neck_x_rotation;
    cmdhead_.neck_z_rotation=last_neck_z_rotation;
    cmdhead_.mouth=last_mouth;
    cmdhead_.upper_head=last_upper_head;
    cmdhead_.left_eye=last_left_eye;
    cmdhead_.right_eye=last_right_eye;
    cmdhead_.flag=500;
  
    for(int i=0;i<10;i++){
      cmdhead_.neck_x_rotation=init_x - 150*i;
      pub_head1.publish(cmdhead_);
      usleep(100000);
    }
    //ROS_INFO("cmdhead_ neck_x = %d", cmdhead_.neck_x_rotation);
    for(int j=10;j>0;j--){
      cmdhead_.neck_x_rotation=init_x-150*j;
      pub_head1.publish(cmdhead_);
      usleep(100000);
    }
    pub_head1.publish(cmdhead_);
    //ROS_INFO("cmdhead_ neck_x = %d", cmdhead_.neck_x_rotation);
}


void neck_handler::shake(){
    drrobot_h20_player::HeadCmd cmdhead_;
    int init_z= last_neck_z_rotation;
    cmdhead_.neck_x_rotation=last_neck_x_rotation;
    cmdhead_.mouth=last_mouth;
    cmdhead_.upper_head=last_upper_head;
    cmdhead_.left_eye=last_left_eye;
    cmdhead_.right_eye=last_right_eye;
    cmdhead_.flag=500;
  
    for(int i=0;i<8;i++){
      cmdhead_.neck_z_rotation=init_z - 150*i;
      pub_head1.publish(cmdhead_);
      usleep(80000);
    }
    for(int j=0;j<16;j++){
      cmdhead_.neck_z_rotation=init_z - 150*8 + 150*j;
      pub_head1.publish(cmdhead_);
      usleep(80000);
    }
    for(int i=0;i<16;i++){
      cmdhead_.neck_z_rotation=init_z + 150*8 - 150*i;
      pub_head1.publish(cmdhead_);
      usleep(80000);
    }
    for(int j=8;j>0;j--){
      cmdhead_.neck_z_rotation=init_z - 150*j;
      pub_head1.publish(cmdhead_);
      usleep(80000);
    }
    
}
