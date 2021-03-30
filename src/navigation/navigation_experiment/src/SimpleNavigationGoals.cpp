#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <queue>
#include <algorithm>
#include <iterator>

#define PI 3.14159265

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SimpleNavigationGoals{
public:
  MoveBaseClient ac;
  double current_x, current_y, goal_x, goal_y, dist_x_goal, dist_y_goal, dist_from_goal;
  bool isDone;
  std::queue<double> locations[2];

  SimpleNavigationGoals():
  //Set up the client. It's publishing to topic "move_base", and is set to auto-spin
    ac("move_base", true)
  {
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
  }

  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResult::ConstPtr& result)
  {
    std::string line; 

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      std::string message = "Hooray, the base moved reached the goal - "; 
      ROS_INFO(message.c_str());
      line = "1 - Using its own tolerance.\n";
    }
    else
    {
      std::string message = "The base failed to move to the goal for some reason - "; 
      ROS_INFO(message.c_str());
      line = "0\n";

      if(isDone == true)
        line = "1 - Using my tolerance.\n";
    }

    //write in the file
    std::ofstream outfile;
    outfile.open("/home/tangerine/ros/tangy/src/navigation/navigation_experiment/experiment/navigation_experiment.txt", 	std::ios_base::app);
    outfile << line;
    outfile.close();
  }

  // Called once when the goal becomes active
  void activeCb()
  {
    ROS_INFO("Goal just went active...");
    isDone = false;

    while(!locations[0].empty())
      locations[0].pop();

    while(!locations[1].empty())
      locations[1].pop();
  }
	
  // Called every time feedback is received for the goal
  void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
  {
    // Get the current position
    current_x = feedback->base_position.pose.position.x;
    current_y = feedback->base_position.pose.position.y;

    // Store feedback for later sampling
    locations[0].push(current_x);
    locations[1].push(current_y);

    // Calculate the current distance (x,y) from the goal
    dist_x_goal = goal_x - current_x;
    dist_y_goal = goal_y - current_y; 

    dist_from_goal = sqrt(dist_x_goal*dist_x_goal + dist_y_goal*dist_y_goal);

    ROS_INFO("Current position: x = [%lf], y = [%lf]\n"
	     "Goal position: x = [%lf], y = [%lf]\n"
	     "Distance from goal: [%lf]\n", current_x, current_y, goal_x, goal_y, dist_from_goal);
		
    if(dist_from_goal <= 0.15)
    {
      isDone = true;
      ac.cancelGoal();
      ROS_INFO("I'm cancelling the goal.");
    }

    if(locations[0].size() > 100 && locations[1].size() > 100)
    {
      if(current_x == locations[0].front() && current_y == locations[1].front())
	ac.cancelGoal();

      locations[0].pop();
      locations[1].pop();
    }
  }

  //Send a goal to the server
  void send(const move_base_msgs::MoveBaseGoal goal)
  {		
    goal_x = goal.target_pose.pose.position.x;
    goal_y = goal.target_pose.pose.position.y;

    //Once again, have to used boost::bind because you are inside a class
    ac.sendGoal(goal, boost::bind(&SimpleNavigationGoals::doneCb, this, _1, _2),
                	boost::bind(&SimpleNavigationGoals::activeCb, this),
			boost::bind(&SimpleNavigationGoals::feedbackCb, this, _1));	
  }

};
			
int main (int argc, char **argv)
{
  ros::init(argc, argv, "simple_navigation_goals");

  double locations[5][7] = {


    //{6.276,9.157,0.000,0.000,0.000,0.000,1.000},
    //{0.000,0.000,0.000,0.000,0.000,0.000,1.000}

    /*{1.969,1.215,0.000,0.000,0.000,0.000,1.000},
    {4.216,1.406,0.000,0.000,0.000,0.707,0.707},
    {4.216,6.550,0.000,0.000,0.000,0.707,0.707},
    {6.130,8.400,0.000,0.000,0.000,0.707,0.707},
    {6.276,9.157,0.000,0.000,0.000,0.000,1.000},
    {12.511,9.120,0.000,0.000,0.000,0.000,1.000},
    {23.352,9.675,0.000,0.000,0.000,0.000,1.000},
    {31.261,9.950,0.000,0.000,0.000,0.000,1.000},
    {0.000,0.000,0.000,0.000,0.000,0.000,1.000}*/

    /*{0.050,0.600,0.000,0.000,0.000,0.000,1.000},
    {0.550, 0.000, 0.000,0.000, 0.000, -0.707, 0.707}//,
    {0.850, -0.550, 0.000,0.000, 0.000, 1.000, 0.000},
    {0.050, -0.550, 0.000,0.000, 0.000, 0.707, 0.707},
    {0.000,0.000,0.000,0.000,0.000,0.000, 1.000} */
	//binngo
{1.287, 1.021, 0.000, 0.000, 0.000, 0.177, 0.984},
{3.889, 1.391, 0.000, 0.000, 0.000, 0.441, 0.898},
{4.481, 4.113, 0.00, 0.000, 0.000, -0.779, 0.627},
{3.149, 1.266, 0.000,0.000, 0.000, 1.000, -0.017},
{00.325, 0.710, 0.000, 0.000, 0.000, 0.992, -0.130}

  };
 	
  //Initialize the client
  SimpleNavigationGoals client;

  int rounds = 5;
  for(int i = 0; i < rounds; ++i)
  {
    for(int id = 0; id < 5; ++id)
    {
      printf("Starting new navigation to %i", id);
      move_base_msgs::MoveBaseGoal goal;

      goal.target_pose.header.frame_id = "/map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = locations[id][0];
      goal.target_pose.pose.position.y = locations[id][1];
      goal.target_pose.pose.position.z = locations[id][2];
      goal.target_pose.pose.orientation.x = locations[id][3];
      goal.target_pose.pose.orientation.y = locations[id][4];
      goal.target_pose.pose.orientation.z = locations[id][5];
      goal.target_pose.pose.orientation.w = locations[id][6];
	  
      ROS_INFO("Sending goal");
      client.send(goal);      
      client.ac.waitForResult();	
    }
  }
	
  std::string voicecommand = "echo \"I am getting dizzy! I am done with it!\" | festival --tts";
  system(voicecommand.c_str());

  return 0;
}


