#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <trivia/TriviaGameAction.h>
#include <iostream>
#include <std_msgs/String.h>
#include <signal.h>

volatile sig_atomic_t stop;

void
inthand(int signum)
{
    stop = 1;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "start_trivia");
  ros::NodeHandle nh;
  ros::Publisher endGamePub=nh.advertise<std_msgs::String>("end_game", 100);
  signal(SIGINT, inthand);
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<trivia::TriviaGameAction> ac("trivia_game", true);

  ROS_INFO("Waiting for trivia server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("trivia server started, sending goal.");
  // send a goal to the action
  trivia::TriviaGameGoal trivia_goal;
  trivia_goal.start = 0;

  int gameType = -1;
  bool finished_before_timeout;
  
  while(gameType != 1 && ros::ok() && !stop)
  {
    	std::cout << "Please input: 0 = new game, 1 = end game: " << std::endl;
    	std::cin >> gameType;
    	while(gameType < 0 || gameType > 2  && ros::ok()) {
    		std::cout << "Invalid input. Please input: 0 = new game, 1 = end game: " << std::endl;
    		std::cin >> gameType;
    	}
      if(gameType==0)	{
    	  trivia_goal.start = gameType;
    	  ac.sendGoal(trivia_goal);
      }else{
        std_msgs::String msg;
        msg.data = "y";
        endGamePub.publish(msg);
        
      }
    	//wait for the action to return
    	 // finished_before_timeout = ac.waitForResult();
  }
 

/*  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
    else
  {
      ROS_INFO("Action did not finish before the time out.");
  }*/
  
  ros::spinOnce();
  //exit
  return 0;
}
