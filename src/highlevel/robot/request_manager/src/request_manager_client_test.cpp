#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <request_manager/RequestManagerAction.h>
#include <request_manager/request_manager_client.h>
#include <vector>


int main (int argc, char **argv)
{
  ros::init(argc, argv, "request_manager_client");

  //float location1[6] = {0.001,0.002,0.003,0.004,0.005,0.006};
  //float location2[6] = {1.001,1.002,1.003,1.004,1.005,1.006};
  // USING CLIENT AS CLASS
  
  RequestManagerClient a("request_manager");
  

  ROS_INFO("Waiting for robot's request manager server to start.");
  // wait for the action server to start
  a.ac.waitForServer(); //will wait for infinite time
 
  ROS_INFO("Robot's Request Manager server started, sending goal.");
/*  // send a goal to the action
  request_manager::RequestManagerGoal request_manager_goal;
  request_manager_goal.newrequest = 1;
  request_manager_goal.command = "10.001: (move r1 l0 l1 cs1) [20.000]";
  //request_manager_goal.coordinates.push_back("l0:0.001,0.002,0.003,0.004,0.005,0.006,0.007"); 
  //request_manager_goal.coordinates.push_back("l1:3.786,1.590,0.000,0.000,0.000,-0.028,1.000");
  request_manager_goal.info.push_back("l0:0.550,0.000,0.000,0.000,0.000,-0.707,0.707"); 
  request_manager_goal.info.push_back("l1:3.786,1.590,0.000,0.000,0.000,-0.028,1.000");

  a.sendGoal(request_manager_goal);
  
  request_manager::RequestManagerResult result;

  //Wait for the action to return
  bool finished_before_timeout = a.ac.waitForResult(ros::Duration(120.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = a.ac.getState();
    ROS_INFO("Getting Result\n");
    result = *a.ac.getResult();
    
    //ROS_INFO("Action finished: %s",state.toString().c_str());
    ROS_INFO("Action finished: %s",(a.succeeded)?"succeeded":"failed");
    //ROS_INFO("Result (Plan): \n%s",result.plan.c_str());     
      
  }
  else{
    ROS_INFO("Action did not finish before the time out.");    
    //Cancelling the goals
    a.ac.cancelAllGoals();
    ROS_INFO("Goal Canceled");

  }


printf("\n****************************\nsending new goal in 3 seconds\n****************************\n");
sleep(3);

//---------------------------------------------------------------------------------------
*/
	request_manager::RequestManagerGoal request_manager_goal_aa;
	request_manager_goal_aa.newrequest = 1;
	request_manager_goal_aa.command = "15.001: (dotelepresence r1 s2 user1 l1 cs1) [20.000]";
	request_manager_goal_aa.info.push_back("s2:Rafael,echo123");
	request_manager_goal_aa.info.push_back("user1:Rafael"); 
	request_manager_goal_aa.info.push_back("l1:3.786,1.590,0.000,0.000,0.000,-0.028,1.000");

	a.sendGoal(request_manager_goal_aa);

	//Wait for the action to return
	bool finished_before_timeout_aa = a.ac.waitForResult(ros::Duration(120.0));
	if (finished_before_timeout_aa)
	{
		actionlib::SimpleClientGoalState state = a.ac.getState();
		ROS_INFO("Getting Result\n");
		//result = *a.ac.getResult();

		//ROS_INFO("Action finished: %s",state.toString().c_str());
		ROS_INFO("Action finished: %s",(a.succeeded)?"succeeded":"failed");
		//ROS_INFO("Result (Plan): \n%s",result.plan.c_str());  	  
	}
	else{
		ROS_INFO("Action did not finish before the time out.");    
		//Cancelling the goals
		a.ac.cancelAllGoals();
		ROS_INFO("Goal Canceled");

	}


printf("\n****************************\nsending new goal in 5 seconds\n****************************\n");
sleep(3);

//----------------------------------------------------------------------------------------
  request_manager::RequestManagerGoal request_manager_goal5;
  request_manager_goal5.newrequest = 1;
  request_manager_goal5.command = "10.001: (remind r1 l0 l1 cs1) [20.000]";
  request_manager_goal5.info.push_back("l0:3.786,1.590,0.000,0.000,0.000,-0.028,1.000");
  request_manager_goal5.info.push_back("l1:4.456,3.811,0.000,0.000,0.000,-0.006,1.000"); 

  a.sendGoal(request_manager_goal5);
  
  
  request_manager::RequestManagerResult result5;


  //Wait for the action to return
  bool finished_before_timeout5 = a.ac.waitForResult(ros::Duration(120.0));
  if (finished_before_timeout5)
  {
    actionlib::SimpleClientGoalState state = a.ac.getState();
    ROS_INFO("Getting Result\n");
    result5 = *a.ac.getResult();
    
    //ROS_INFO("Action finished: %s",state.toString().c_str());
    ROS_INFO("Action finished: %s",(a.succeeded)?"succeeded":"failed");
      
  }
  else{
    ROS_INFO("Action did not finish before the time out.");    
    //Cancelling the goals
    a.ac.cancelAllGoals();
    ROS_INFO("Goal Canceled");

  }/*
//--------------------------------------------------------------------------------------

printf("\n****************************\nsending new goal in 3 seconds\n****************************\n");
sleep(3);
//---------------------------------------------------------------------------------------

	request_manager::RequestManagerGoal request_manager_goal_a;
	request_manager_goal_a.newrequest = 1;
	request_manager_goal_a.command = "15.001: (dotelepresence r1 s2 user1 l1 cs1) [20.000]";
	request_manager_goal_a.info.push_back("s2:Rafael,echo123");
	request_manager_goal_a.info.push_back("user1:Rafael"); 
	request_manager_goal_a.info.push_back("l1:3.786,1.590,0.000,0.000,0.000,-0.028,1.000");

	a.sendGoal(request_manager_goal_a);

	//Wait for the action to return
	bool finished_before_timeout_a = a.ac.waitForResult(ros::Duration(120.0));
	if (finished_before_timeout_a)
	{
		actionlib::SimpleClientGoalState state = a.ac.getState();
		ROS_INFO("Getting Result\n");
		//result = *a.ac.getResult();

		//ROS_INFO("Action finished: %s",state.toString().c_str());
		ROS_INFO("Action finished: %s",(a.succeeded)?"succeeded":"failed");	
		//ROS_INFO("Result (Plan): \n%s",result.plan.c_str());  	  
	}
	else{
		ROS_INFO("Action did not finish before the time out.");    
		//Cancelling the goals
		a.ac.cancelAllGoals();
		ROS_INFO("Goal Canceled");

	}

//------------------------------------------------------------------------------------


printf("\n****************************\nsending new goal in 5 seconds\n****************************\n");
sleep(3);

//----------------------------------------------------------------------------------------
  request_manager::RequestManagerGoal request_manager_goal1;
  request_manager_goal1.newrequest = 1;
  request_manager_goal1.command = "10.001: (move r1 l0 l1 cs1) [20.000]";
  request_manager_goal1.info.push_back("l0:4.456,3.811,0.000,0.000,0.000,-0.006,1.000");
  request_manager_goal1.info.push_back("l1:-0.132,-0.092,0.000,0.000,0.000,-0.703, 0.711");
  //request_manager_goal1.info.push_back("l1:0.550,0.000,0.000,0.000,0.000,-0.707,0.707"); 

  a.sendGoal(request_manager_goal1);
  
  
  request_manager::RequestManagerResult result1;


  //Wait for the action to return
  bool finished_before_timeout1 = a.ac.waitForResult(ros::Duration(120.0));
  if (finished_before_timeout1)
  {
    actionlib::SimpleClientGoalState state = a.ac.getState();
    ROS_INFO("Getting Result\n");
    result1 = *a.ac.getResult();
    
    //ROS_INFO("Action finished: %s",state.toString().c_str());
    ROS_INFO("Action finished: %s",(a.succeeded)?"succeeded":"failed");
      
  }
  else{
    ROS_INFO("Action did not finish before the time out.");    
    //Cancelling the goals
    a.ac.cancelAllGoals();
    ROS_INFO("Goal Canceled");

  }
*/
//--------------------------------------------------------------------------
  //exit
  return 0;
}
