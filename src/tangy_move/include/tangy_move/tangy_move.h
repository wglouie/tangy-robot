#ifndef TANGY_MOVE_H
#define TANGY_MOVE_H

#include <ros/ros.h>
#include <string>
#include <tangy_move/navServerCmd.h>

class navigationClient {
	public:
		// functions
        navigationClient();
		~navigationClient();
		void initialize(ros::NodeHandle nh);
		void move_straight (double distance);
		void move_straight_manual (double distance);
		void move(std::string frame, double x,double y);
		void face(std::string frame, double x, double y);
		void rotate(std::string frame, double theta);
		void rotate_no_wait(std::string frame, double theta);
		void rotate_manual (double angle);
		void pause();
		void unpause();

	private:
		tangy_move::navServerCmd navServerCmd;

		// variables
		ros::NodeHandle nh;
		ros::ServiceClient navServerCmdPub;
};



#endif
