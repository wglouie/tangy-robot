#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <execution/ExecutionAction.h>
#include <execution/execution_client.h>


int main (int argc, char **argv)
{
  ros::init(argc, argv, "execution_client");


  // USING CLIENT AS CLASS
  
  ExecutionClient a("execution");
  
  ROS_INFO("Waiting for execution server to start.");
  // wait for the action server to start
  a.ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Execution server started, sending goal.");
  // send a goal to the action
  execution::ExecutionGoal execution_goal;
  execution_goal.newrequest = 1;
  execution_goal.request = "replan";
  //execution_goal.request = "current_state";
  a.sendGoal(execution_goal);
  
  
  execution::ExecutionResult result;



  //Wait for the action to return
  bool finished_before_timeout = a.ac.waitForResult(ros::Duration(120.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = a.ac.getState();
    ROS_INFO("Getting Result\n");
    result = *a.ac.getResult();
    
    ROS_INFO("Action finished: %s",state.toString().c_str());
    //ROS_INFO("Result (Plan): \n%s",result.plan.c_str());  
    
    /*
    if (result.solutionExists){
        ROS_INFO("Plan found!");
        ROS_INFO("Plan to be executed: \n%s",result.plan.c_str());
    }
    else if (result.isUnsolvable){
         ROS_INFO("Problem is unsolvable");
    }
    */
      
  }
  else{
    ROS_INFO("Action did not finish before the time out.");    
    //Cancelling the goals
    a.ac.cancelAllGoals();
    ROS_INFO("Goal Canceled");

  }

  
  /*
  //Stopping the goal before it finishes
  sleep(10);
  a.ac.cancelGoal();
  ROS_INFO("Goal Canceled");
  */

  
  //exit
  return 0;
}
