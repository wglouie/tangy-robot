/*!
 * drrobot_keyboard_teleop.cpp
 * Copyright (c) 2011, Dr Robot Inc
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!

@mainpage
  drrobot_keyboard_teleop for demonstration and testing published geometry_msgs/Twist message to drrobot_player.
  It will use 4 keys to control robot move around
  a/A -- 0.5/1 full speed turn to left
  w/W -- 0.5/1 full speed forward
  d/D -- 0.5/1 full speed turn to right
  s/S -- 0.5/1 full speed backward
  if no key pressed, it will stop robot
<hr>

@section usage Usage
@par     After start roscore, you need load robot configuration file to parameter server first.
          For example, I90 robot, you need load drrobotplayer_I90.yaml use command "rosparam load drrobotplayer_I90.yaml"
          then run drrobot_player first.
@verbatim
$ drrobot_keyboard_teleop
@endverbatim

<hr>
@section topic ROS topics

Publishes to (name / type):
-@b drrobot_cmd_vel: will publish drrobot_cmd_vel Message to drrobot_player. For robot from Dr Robot Inc, we only need provide linear.x
    as going forward/backward speed, and angular.z as turning speed. drrobot_player will transform these command value to encoder control
    command value and send them to motion control system on the robot
<hr>
*/
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>


#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <drrobot_h20_player/HeadCmd.h>

#define KEYCODE_T 0x74
#define KEYCODE_G 0x67
#define KEYCODE_H 0x68
#define KEYCODE_F 0x66
#define KEYCODE_R 0x72
#define KEYCODE_Y 0x79
#define KEYCODE_U 0x75
#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_O 0x6f
#define KEYCODE_L 0x6c
#define KEYCODE_C 0x63



class DrRobotKeyboardTeleopNode
{
    private:
        drrobot_h20_player::HeadCmd cmdhead_;
        ros::NodeHandle n_;
        ros::Publisher pub_;

    public:
        DrRobotKeyboardTeleopNode()
        {
            //pub_ = n_.advertise<geometry_msgs::Twist>("drrobot_cmd_vel", 1);
            pub_ = n_.advertise<drrobot_h20_player::HeadCmd>("cmd_head", 1);
            ros::NodeHandle n_private("~");
        }

        ~DrRobotKeyboardTeleopNode() { }
        void keyboardLoop();

        void stopRobot()
        {
            cmdhead_.neck_x_rotation = 0;
            cmdhead_.neck_x_rotation = 0;
            cmdhead_.mouth = 0;
            cmdhead_.upper_head = 0;
            cmdhead_.left_eye = 0;
            cmdhead_.right_eye = 0;
            pub_.publish(cmdhead_);
        }
};

DrRobotKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"drrobot_teleope_keyboard", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    DrRobotKeyboardTeleopNode tbk;

    boost::thread t = boost::thread(boost::bind(&DrRobotKeyboardTeleopNode::keyboardLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.stopRobot();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}

void DrRobotKeyboardTeleopNode::keyboardLoop()
{
    char c;
    int neck_x_rotation_ = 0;
    int neck_z_rotation_= 0;
    int mouth_= 0;
    int upper_head_ = 0;
    int left_eye_ = 0;
    int right_eye_ = 0;
    int flag_ = 0;
    bool dirty = false;
    int counting=0;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use \nT\nF\nG\nH keys to control the robot");
    puts("Press Shift to move faster");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    for(;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (dirty == true)
            {
                stopRobot();
                dirty = false;
            }	

            continue;
        }

        switch(c)
        {

	    //Tiago: I reduced these speeds - 0.2 (or even 0.1) and 0.5 with caps-lock on.
            case KEYCODE_T:
                neck_x_rotation_ = 20;

		neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
            case KEYCODE_G:
                neck_x_rotation_ = -20;

		neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
                
            case KEYCODE_F:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 20;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
            case KEYCODE_H:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = -20;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
                
            case KEYCODE_R:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 20;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
            case KEYCODE_Y:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = -20;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
                
            case KEYCODE_U:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = -20;
                right_eye_ = 0;
                dirty = true;
                break;
            case KEYCODE_I:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 20;
                right_eye_ = 0;
                dirty = true;
                break;
                
            case KEYCODE_J:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = -20;
                dirty = true;
                break;
            case KEYCODE_K:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 20;
                dirty = true;
                break;
                
            case KEYCODE_O:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 20;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
            case KEYCODE_L:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = -20;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;

	    case KEYCODE_C:
		flag_ = 1;
		dirty = true;
		break;

            default:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = false;
        }

        cmdhead_.neck_x_rotation = neck_x_rotation_;
        cmdhead_.neck_z_rotation = neck_z_rotation_;
	counting += neck_z_rotation_;
	ROS_INFO("\n\n\n THIS IS THE STUFF: %d \n\n\n", counting);
        cmdhead_.mouth = mouth_;
        cmdhead_.upper_head = upper_head_;
        cmdhead_.left_eye = left_eye_;
        cmdhead_.right_eye = right_eye_;
        cmdhead_.flag = flag_;
	ROS_INFO(" \n\n\n\n drugs : %d drugged flag: %d\n\n\n ",cmdhead_.neck_z_rotation,cmdhead_.flag);
        pub_.publish(cmdhead_);
    }
}

