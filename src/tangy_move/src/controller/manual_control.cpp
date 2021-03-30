#include <ros/ros.h>
#include <tangy_move/tangy_move.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <iostream>

std_msgs::String game_state;

void game_state_cb(const std_msgs::String::ConstPtr& msg){
  game_state=*msg;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "manual_control");
	ros::NodeHandle nh;

	// publish controls that are not directly to move_base
  	ros::Publisher velCommand = nh.advertise<std_msgs::String>("control_command", 1000);
  	ros::Subscriber game_state_sub= nh.subscribe("game_state", 1, game_state_cb);
	ros::Publisher pausePublisher = nh.advertise<std_msgs::String>("pauseSub", 1000);
	// turn input buffering off
	struct termios terminal_io;
    	tcgetattr(STDIN_FILENO, &terminal_io); //get the current terminal I/O structure
    	terminal_io.c_lflag &= ~ICANON; //Manipulate the flag bits to do what you want it to do
    	tcsetattr(STDIN_FILENO, TCSANOW, &terminal_io); //Apply the new settings

	// navigation client
	navigationClient navClient;
	navClient.initialize(nh);

	bool paused = false;

	// control loop
	while(ros::ok()) {
		// take input from the keyboard
		char input;
		std::cin.get(input);

		// set up string message for publishing (if not set then default command is a move operation)
		std_msgs::String str;
   		str.data = "move";
		switch (input) {
			// forward
			case 'w':
				if(paused)
					navClient.move_straight_manual(0.3);
				break;
			// backward
			case 's':
				if(paused)
					navClient.move_straight_manual(-0.3);
				break;
			// turn left
			case 'a':
				if(paused)
					navClient.rotate_manual(0.5);
				break;
			// turn right
			case 'd':
				if(paused)
					navClient.rotate_manual(-0.5);
				break;
			// stop
			case 'q':
				if(paused)
					navClient.move_straight_manual(0);
				break;
			// pause bingo game
			case 'p':
				if(paused == false) {
					ROS_INFO("Paused");
					str.data = "pause";
					std::cout << game_state.data.c_str();
					paused = true;
					pausePublisher.publish(str);
					//navClient.pause();
				} else {
					ROS_INFO("Unpaused");
					str.data = "unpause";
					std::cout << game_state.data.c_str();
					paused = false;
					pausePublisher.publish(str);				
					//navClient.unpause();
				}
				break;
			case 'c':
				return 0;
				break;
			default:
				break;
		}
		velCommand.publish(str);

	}
	return 0;
}
