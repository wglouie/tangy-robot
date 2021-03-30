#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <planning/PlanningAction.h>
#include <planning/planning_client.h>


int main (int argc, char **argv)
{
  ros::init(argc, argv, "planning_client");


  // USING CLIENT AS CLASS
  
  PlanningClient a("planning");
  
  ROS_INFO("Waiting for planning server to start.");
  // wait for the action server to start
  a.ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Planning server started, sending goal.");
  // send a goal to the action
  planning::PlanningGoal planning_goal;
  planning_goal.newrequest = 1;
  planning_goal.sender = ros::this_node::getName();
  a.sendGoal(planning_goal);
  
  
  planning::PlanningResult result;



  //Wait for the action to return
  bool finished_before_timeout = a.ac.waitForResult(ros::Duration(60.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = a.ac.getState();
    ROS_INFO("Getting Result\n");
    result = *a.ac.getResult();
    
    ROS_INFO("Action finished: %s",state.toString().c_str());
    //ROS_INFO("Result (Plan): \n%s",result.plan.c_str());  
    
    if (result.solutionExists){
        ROS_INFO("Solution found!");
        ROS_INFO("Result (Plan): \n%s",result.plan.c_str());          
    }
    else if (result.isUnsolvable){
         ROS_INFO("Problem is unsolvable");
    }
      
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  
  
  
  
  /*
  //Stopping the goal before it finishes
  sleep(5);  
  a.ac.cancelGoal();
  ROS_INFO("Goal Canceled");
  */
  
  

  
  
  // NOT USING CLIENT AS A CLASS
  /*
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<planning::PlanningAction> ac("planning", true);
  

  ROS_INFO("Waiting for planning server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Planning server started, sending goal.");
  // send a goal to the action
  planning::PlanningGoal planning_goal;
  planning_goal.newrequest = 1;
  ac.sendGoal(planning_goal);
  
  
  planning::PlanningResult result;

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Getting Result\n");
    result = *ac.getResult();
    
    ROS_INFO("Action finished: %s",state.toString().c_str());
    ROS_INFO("Results: %s",result.plan.c_str());    
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  
   */

  //exit
  return 0;
}
