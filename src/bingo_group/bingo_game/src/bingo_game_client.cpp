#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <bingo_game/BingoGameAction.h>
#include <iostream>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "start_bingo");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<bingo_game::BingoGameAction> ac("bingo_game", true);

  ROS_INFO("Waiting for bingo server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Bingo server started, sending goal.");
  // send a goal to the action
  bingo_game::BingoGameGoal bingo_goal;
  bingo_goal.start = 0;

  int gameType = -1;
  bool finished_before_timeout;
  
  while(gameType != 2)
  {
    std::cout << "Please input your desired game type: 0 = new game, 1 = continue previous game, 2 = end game, 3= demo game, 4 = Hello O'Neil, 5=some stuff" << std::endl;
    std::cin >> gameType;
    while(gameType < 0 || gameType > 5 ) {
        std::cout << "Please input your desired game type: 0 = new game, 1 = continue previous game, 2 = end game, 3= demo game, 4 = Hello O'Neil, 5=some stuff" << std::endl;
        std::cin >> gameType;
  	}
  	bingo_goal.start = gameType; 
  	ac.sendGoal(bingo_goal);
  	//wait for the action to return
 	finished_before_timeout = ac.waitForResult();
  }
 

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
    else
  {
      ROS_INFO("Action did not finish before the time out.");
  }
  
  ros::spinOnce();
  //exit
  return 0;
}
