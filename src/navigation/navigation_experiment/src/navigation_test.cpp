#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iterator>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	ros::init(argc, argv, "simple_navigation_goals");

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
	 
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}


    double locations[1][7] = {
                    {1.000,0.000,0.000,0.000,0.000,0.000, 1.000}/*,
                    {0.350, 0.300, 0.000,0.000, 0.000, -0.702, 0.712},
                    {0.350, -0.300, 0.000,0.000, 0.000, 0.999, 0.054},
                    {0.050, -0.300, 0.000,0.000, 0.000, 0.717, 0.697}
                    {0.050,0.000,0.000,0.000,0.000,0.000, 1.000}*/ 

/* 		    {0.050,0.750,0.000,0.000,0.000,0.000, 1.000},
                    {0.885, 0.728, 0.000,0.000, 0.000, -0.702, 0.712},
                    {0.916, -0.700, 0.000,0.000, 0.000, 0.999, 0.054},
                    {0.054, -0.877, 0.000,0.000, 0.000, 0.717, 0.697},
                    {0.050,0.750,0.000,0.000,0.000,0.000, 1.000} */
    };

        //0 - Position(0.522, 0.903, 0.000), Orientation(0.000, 0.000, 0.208, 0.978) = Angle: 0.418
        //1 - Position(1.441, 1.227, 0.000), Orientation(0.000, 0.000, -0.577, 0.816) = Angle: -1.231
        //2 - Position(1.416, -0.050, 0.000), Orientation(0.000, 0.000, -0.692, 0.722) = Angle: -1.529
        //3 - Position(0.630, -0.720, 0.000), Orientation(0.000, 0.000, 1.000, 0.028) = Angle: 3.085
        //4 - Position(0.109, 0.098, 0.000), Orientation(0.000, 0.000, 0.687, 0.726) = Angle: 1.516

    int rounds = 1;
    for(int i = 0; i < rounds; ++i)
    {
        for(int id = 0; id < 1; ++id)
        {

            printf("Starting new navigation to %i", id);
            move_base_msgs::MoveBaseGoal goal;

            //we'll send a goal to the robot to move 1 meter forward
            goal.target_pose.header.frame_id = "/base_link";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = locations[id][0];
            goal.target_pose.pose.position.y = locations[id][1];
            goal.target_pose.pose.position.z = locations[id][2];
            goal.target_pose.pose.orientation.x = locations[id][3];
            goal.target_pose.pose.orientation.y = locations[id][4];
            goal.target_pose.pose.orientation.z = locations[id][5];
            goal.target_pose.pose.orientation.w = locations[id][6];

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            float dur = 10.0; // before 30
            if (id == 0)
                dur = 10.0; //before 40
            ac.waitForResult(ros::Duration(dur));

            std::stringstream ss;
            ss << id;
            std::string line = ss.str();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                std::string message = "Hooray, the base moved reached the goal - " + ss.str();
                ROS_INFO(message.c_str());
                line = line + ",1\n";
            }
            else{
                std::string message = "The base failed to move to the goal for some reason - " + ss.str();
                ROS_INFO(message.c_str());
                line = line + ",0\n";
            }
            //write in the file
            std::ofstream outfile;
            outfile.open("/home/tangerine/ros/tangy/src/navigation/navigation_experiment/experiment/navigation_experiment.txt", std::ios_base::app);
            outfile << line;
            outfile.close();

        }
    }
    std::string voicecommand = "echo \"I am getting dizzy! I am done with it!\" | festival --tts";
    system(voicecommand.c_str());


	return 0;
}
