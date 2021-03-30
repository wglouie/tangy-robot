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
   
   pub_head1 = nh_.advertise<drrobot_h20_player::HeadCmd>("/cmd_pose_head", 100);
   sleep(2);
   sub_head = nh_.subscribe("/cmd_pose_head",100, &neck_handler::record_last_posCB, this);
   
   last_neck_x_rotation=3800;
   last_neck_z_rotation=3450;
   last_mouth=3500;
   last_upper_head=3500;
   last_left_eye=3500;
   last_right_eye=3450;
   stored_neck_x=3800;
   stored_neck_z=3450;
   
///////////////////////////////////////////////////////
   previousmovement=0;
//////////////////////////////////////////////////////////
}

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
  // ROS_INFO("last_neck_x_rotation = [%d]", last_neck_x_rotation);
  // ROS_INFO("last_neck_z_rotation=[%d]", last_neck_z_rotation);
}

/*Reset head position*/
void neck_handler::reset_neck_pos()
{
    // pub_head = nh_.advertise<drrobot_h20_player::HeadCmd>("/cmd_head", 1);
    drrobot_h20_player::HeadCmd cmdhead_;

    int neck_x_rotation_ = 3600;
    int neck_z_rotation_= 3450;
    int mouth_= 3500;
    int upper_head_ = 3500;
    int left_eye_ = 3500;
    int right_eye_ = 3450;


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
    // pub_head2.publish(cmdhead_);
}

void neck_handler::rand_neck_pos(){
    drrobot_h20_player::HeadCmd cmdhead_;

    int neck_x_rotation_ = 3600;
    int neck_z_rotation_=3450;
    int mouth_= 3500;
    int upper_head_ = 3500;
    int left_eye_ = 3500;
    int right_eye_ = 3450;
    
    while(abs(neck_z_rotation_-last_neck_z_rotation)<100){
        int rand_pan=(rand()% 1000)-500;
        neck_z_rotation_=rand_pan+3450;
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
}

void neck_handler::look_down(){
    drrobot_h20_player::HeadCmd cmdhead_;

    int neck_x_rotation_ = 3550;
    int neck_z_rotation_=3450;
    int mouth_= 3500;
    int upper_head_ = 3500;
    int left_eye_ = 3500;
    int right_eye_ = 3450;

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

/*
    if((20-previousmovement)>leftrightgoal && (-20-previousmovement)<leftrightgoal)
    {
	previousmovement=leftrightgoal;
	leftrightgoal=0;
    }
*/ 
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
    //usleep(ROTATION_TIME);

    //goalsidetoside=fabs(leftrightgoal/ROTATION_DEFINITION);
    //goalvertical=fabs(updowngoal/ROTATION_DEFINITION);

    //drrobot_h20_player::HeadCmd cmdhead_;
    //ros::NodeHandle nh_;
    //ros::Publisher pub_;
	    
    //pub_ = n_.advertise<drrobot_h20_player::HeadCmd>("cmd_head", (goalsidetoside+goalvertical));
/*
    int i;
    for(i=0;i<goalsidetoside;i++)
    {
	if(leftrightgoal<0)
	{
		neck_z_rotation_ = -ROTATION_DEFINITION;
	}
	else if(leftrightgoal>0)
	{
		neck_z_rotation_ = ROTATION_DEFINITION;
	}
	mouth_ = 0;
        upper_head_ = 0;
        left_eye_ = 0;
        right_eye_ = 0;
	neck_x_rotation_ = 0;
	cmdhead_.neck_x_rotation = neck_x_rotation_;
        cmdhead_.neck_z_rotation = -neck_z_rotation_;
	cmdhead_.mouth = mouth_;
	cmdhead_.upper_head = upper_head_;
	cmdhead_.left_eye = left_eye_;
	cmdhead_.right_eye = right_eye_;
	cmdhead_.flag = flag_;
	pub_.publish(cmdhead_);
	usleep(ROTATION_TIME);
    }

    for(i=0;i<goalvertical;i++)
    {
	if(updowngoal<0)
	{
		neck_x_rotation_ = -ROTATION_DEFINITION;
	}
	else if(updowngoal>0)
	{
		neck_x_rotation_ = ROTATION_DEFINITION;
	}
	mouth_ = 0;
        upper_head_ = 0;
        left_eye_ = 0;
        right_eye_ = 0;
	neck_z_rotation_ = 0;
	cmdhead_.neck_x_rotation = neck_x_rotation_;
        cmdhead_.neck_z_rotation = -neck_z_rotation_;
	cmdhead_.mouth = mouth_;
	cmdhead_.upper_head = upper_head_;
	cmdhead_.left_eye = left_eye_;
	cmdhead_.right_eye = right_eye_;
	cmdhead_.flag = flag_;
	pub_.publish(cmdhead_);
	usleep(ROTATION_TIME);
        }
     }
*/
}

void neck_handler::look_at_card_1(){
    stored_neck_x=last_neck_x_rotation;
    stored_neck_z=last_neck_z_rotation;
    // ROS_INFO("stored_neck_x_rotation = [%d]", stored_neck_x);
    // ROS_INFO("stored_neck_z_rotation=[%d]", stored_neck_z);
    int neck_x_rotation_= 3200;
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
  
    int neck_x_rotation_= 3300;
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
  
    int neck_x_rotation_= 3100;
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

}
