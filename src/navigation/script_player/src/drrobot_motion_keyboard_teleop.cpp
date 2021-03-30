#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>


#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <drrobot_h20_player/HeadCmd.h>

#include "drrobot_script_teleop.hpp"
#include "drrobot_head_script.hpp"

//Base Motion
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_W_CAP 0x57
#define KEYCODE_A_CAP 0x41
#define KEYCODE_S_CAP 0x53
#define KEYCODE_D_CAP 0x44

//head Motion
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

//Poses
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32

class DrRobotMotionKeyboardTeleopNode
{
    private:
        geometry_msgs::Twist cmdvel_;
        drrobot_h20_player::HeadCmd cmdhead_;
        ros::NodeHandle n_;
        ros::Publisher pub_base, pub_head;
        
        DrRobotHeadScript drrobotheadscript_;
        DrRobotScriptTeleop drrobotscriptteleop_;
        
        std::string xmlFile_head, xmlFile_arm;
        

    public:
        DrRobotMotionKeyboardTeleopNode()
        {
 			pub_head = n_.advertise<drrobot_h20_player::HeadCmd>("cmd_head", 1);
            pub_base = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            ros::NodeHandle n_private("~");
            xmlFile_head = "/home/nikiminha/ros/tangy/src/navigation/drrobot_h20_arm_player/script/Pose_head.xml";
            xmlFile_arm = "/home/nikiminha/ros/tangy/src/navigation/drrobot_h20_arm_player/script/Pose_arm.xml";
        }

        ~DrRobotMotionKeyboardTeleopNode() { }
        void keyboardLoop();

        void stopRobot()
        {
            cmdvel_.linear.x = 0.0;
            cmdvel_.angular.z = 0.0;
            cmdhead_.neck_x_rotation = 0;
            cmdhead_.neck_x_rotation = 0;
            cmdhead_.mouth = 0;
            cmdhead_.upper_head = 0;
            cmdhead_.left_eye = 0;
            cmdhead_.right_eye = 0;
            pub_head.publish(cmdhead_);
            pub_base.publish(cmdvel_);
        }
};

DrRobotMotionKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"drrobot_teleope_keyboard", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    DrRobotMotionKeyboardTeleopNode tbk;

    boost::thread t = boost::thread(boost::bind(&DrRobotMotionKeyboardTeleopNode::keyboardLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.stopRobot();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}

void DrRobotMotionKeyboardTeleopNode::keyboardLoop()
{
    char c;
    double maxVel = 1.0;
    double maxTurn = 1.0;
    int neck_x_rotation_ = 0;
    int neck_z_rotation_= 0;
    int mouth_= 0;
    int upper_head_ = 0;
    int left_eye_ = 0;
    int right_eye_ = 0;
    int flag_ = 0;
    bool dirty = false;


    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
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
        	case KEYCODE_1:
        		drrobotscriptteleop_.getPose(xmlFile_arm);
        		break;
        		
        	case KEYCODE_2:
        		drrobotheadscript_.getPose(xmlFile_head);
        		break;
        		
            case KEYCODE_W:
                maxVel = 0.2;
                maxTurn = 0;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
            case KEYCODE_S:
                maxVel = -0.2;
                maxTurn = 0;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
            case KEYCODE_A:
                maxVel = 0;
                maxTurn = 0.2;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
            case KEYCODE_D:
                maxVel = 0;
                maxTurn = -0.2;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;

            case KEYCODE_W_CAP:
                maxVel = 0.5;
                maxTurn = 0;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
            case KEYCODE_S_CAP:
                maxVel = -0.5;
                maxTurn = 0;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
            case KEYCODE_A_CAP:
                maxVel = 0;
                maxTurn = 0.5;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
            case KEYCODE_D_CAP:
                maxVel = 0;
                maxTurn = -0.5;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                dirty = true;
                break;
            case KEYCODE_T:
                neck_x_rotation_ = 20;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
            case KEYCODE_G:
                neck_x_rotation_ = -20;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
                
            case KEYCODE_F:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 20;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
            case KEYCODE_H:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = -20;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
                
            case KEYCODE_R:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 20;
                left_eye_ = 0;
                right_eye_ = 0;
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
            case KEYCODE_Y:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = -20;
                left_eye_ = 0;
                right_eye_ = 0;
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
                
            case KEYCODE_U:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = -20;
                right_eye_ = 0;
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
            case KEYCODE_I:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 20;
                right_eye_ = 0;
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
                
            case KEYCODE_J:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = -20;
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
            case KEYCODE_K:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 20;
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
                
            case KEYCODE_O:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 20;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
            case KEYCODE_L:
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = -20;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
                maxVel = 0;
                maxTurn = 0;
                dirty = true;
                break;
            case KEYCODE_C:
				flag_ = 1;
				maxVel = 0;
                maxTurn = 0;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
				dirty = true;
				break;
			default:
				flag_ = 0;
				maxVel = 0;
                maxTurn = 0;
                neck_x_rotation_ = 0;
                neck_z_rotation_ = 0;
                mouth_ = 0;
                upper_head_ = 0;
                left_eye_ = 0;
                right_eye_ = 0;
				dirty = true;
				break;
        }

        cmdvel_.linear.x = maxVel;
        cmdvel_.angular.z = maxTurn;
        cmdhead_.neck_x_rotation = neck_x_rotation_;
        cmdhead_.neck_z_rotation = neck_z_rotation_;
        cmdhead_.mouth = mouth_;
        cmdhead_.upper_head = upper_head_;
        cmdhead_.left_eye = left_eye_;
        cmdhead_.right_eye = right_eye_;
        cmdhead_.flag = flag_;
        pub_head.publish(cmdhead_);
        pub_base.publish(cmdvel_);
    }
}
