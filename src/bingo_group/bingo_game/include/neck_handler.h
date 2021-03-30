//============================================================================
// Name        : neck_handler.hpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : This program controls the neck of the robot.
//               Reset the neck position with reset_neck_pos()
//============================================================================

#ifndef neck_handler_H_
#define neck_handler_H_
#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <drrobot_h20_player/HeadCmd.h>

class neck_handler {
  public:
    int last_neck_x_rotation;
    int last_neck_z_rotation;
    int last_mouth;
    int last_upper_head;
    int last_left_eye;
    int last_right_eye;
    int stored_neck_x;
    int stored_neck_z;
    int previousmovement;


    ros::NodeHandle n_;    
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher pub_head1;
    ros::Publisher pub_head2;
    ros::Subscriber sub_head;

    drrobot_h20_player::HeadCmd cmdhead_;
    neck_handler(std::string name);
    ~neck_handler();
    
    void record_last_posCB(drrobot_h20_player::HeadCmd msg);
    void reset_neck_pos();
    void rand_neck_pos();
    void look_down();                 //at person
    void look_at_card_1();            //Three different positions for looking at the card
    void look_at_card_2();
    void look_at_card_3();
    void look_back_to_person();       //after looking at a card
    //void increment_dir(int left_right, int up_down);  //left_right and up_down are servo values
    void increment_dir(int leftrightgoal, int updowngoal);

};



#endif
