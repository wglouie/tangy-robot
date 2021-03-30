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
#include <costmap_2d/costmap_2d.h>
#include <visualization_msgs/Marker.h>
#include <new_layer/new_layer.h>
#include <navigation_experiment/CheckNavigationGoal.h> 
#include <tf/transform_listener.h>
#include <nav_msgs/GetPlan.h>
#include <unistd.h>

#define PI 3.14159265

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SimpleNavigationGoals{
public:
  MoveBaseClient ac;
  double current_x;
  double current_y;
  double goal_x;
  double goal_y;
  double dist_x_goal;
  double dist_y_goal;
  double dist_from_goal;
  bool isDone;

  //inline definitons to follow
  SimpleNavigationGoals():

  //constructor
  ac("move_base", true) {
    while(!ac.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for move_base action server");
    }
  }

  void doneCb(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResult::ConstPtr& result) {
    ROS_INFO("Hooray, I have reached the goal");
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
      ROS_INFO("Using my own tolerance.");
    else 
      ROS_INFO("Using your tolerance");
  }

  void activeCb() {
    ROS_INFO("Goal just went active...");
  }
	
  void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback) {
    // Get the current position
    current_x = feedback->base_position.pose.position.x;
    current_y = feedback->base_position.pose.position.y;

    // Calculate the current distance (x,y) from the goal
    dist_x_goal = goal_x - current_x;
    dist_y_goal = goal_y - current_y; 
    dist_from_goal = sqrt(dist_x_goal*dist_x_goal + dist_y_goal*dist_y_goal);
    if(dist_from_goal <= 0.20) 
      ac.cancelGoal();
  }

  //Send a goal to the server
  void send(const move_base_msgs::MoveBaseGoal goal) {		
    goal_x = goal.target_pose.pose.position.x;
    goal_y = goal.target_pose.pose.position.y;

    //Once again, have to used boost::bind because you are inside a class
    ac.sendGoal(goal, boost::bind(&SimpleNavigationGoals::doneCb, this, _1, _2),
                	boost::bind(&SimpleNavigationGoals::activeCb, this),
			boost::bind(&SimpleNavigationGoals::feedbackCb, this, _1));	
  }

};
			
int main (int argc, char **argv) {
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("nav_goal_marker", 1000);
  ros::Time now = ros::Time::now();

  ROS_INFO("Starting new navigation");
 
  //Goal to be sent to move_base
  //double goal[7] = {3.55, -0.3, 0.000, 0.000, 0.000, 0.0, 0.0};
  //double goal[7] = {1, 0, 0.000, 0.000, 0.000, 0.693, 0.721};
  //double goal[7] = {2, 0, 0.000, 0.000, 0.000, 0.0, 1};
  //double goal[7] = {3.604, 1.455, 0.000, 0.000, 0.000, -0.066, 0.998};
  //double goal[7] = {1.827, -0.5, 0.000, 0.000, 0.000, 0.000, 1};
  double goal[7] = {5.861, -0.625, 0.000, 0.000, 0.000, 0.000, 1};

  move_base_msgs::MoveBaseGoal goal_msg;
  goal_msg.target_pose.header.frame_id = "/map";
  goal_msg.target_pose.header.stamp = now;
  goal_msg.target_pose.pose.position.x = goal[0];
  goal_msg.target_pose.pose.position.y = goal[1];
  goal_msg.target_pose.pose.position.z = goal[2];
  goal_msg.target_pose.pose.orientation.x = goal[3];
  goal_msg.target_pose.pose.orientation.y = goal[4];
  goal_msg.target_pose.pose.orientation.z = goal[5];
  goal_msg.target_pose.pose.orientation.w = goal[6];

  //create 10 messages to store our safe goals, initialize them to be the same as our original goal
  move_base_msgs::MoveBaseGoal safe_goal_msg[10];
  for(int i = 0; i < 10; i++) {
    safe_goal_msg[i] = goal_msg;
  } 

  //the "cost" of the goal we're going to choose, set it to something high so it will lose to the actual goals
    //cost function = weight1 * length of plan + weight2 * distance to original goal
  int desired_goal = 10000;
 
  //keep track of all the plan lengths when we get them
  int costs[10];

//POINT FINDING ALGORITHM
  //how precise we want the search to be
  float RESOLUTION = 0.1;

  //To check if the goals we get are reachable we need to get the start position of the robot
  //Get the transform for /map to /base_link
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try{
    listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/base_link", "/map", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  //Use the transform to get the (x,y) of the start
  double start_x = transform.getOrigin().x();
  double start_y = transform.getOrigin().y();

  //Service for getting our plan
  nav_msgs::GetPlan getplan_srv; 
  getplan_srv.request.start.header.frame_id = "/map";
  getplan_srv.request.start.header.stamp = now;

  getplan_srv.request.start.pose.position.x = start_x;
  getplan_srv.request.start.pose.position.y = start_y;    

  //We want to make sure the goal is 100% accessible
  getplan_srv.request.tolerance = 0.0; 

  ros::ServiceClient client_getplan = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

  getplan_srv.request.goal.header.frame_id = "/map";
  getplan_srv.request.goal.header.stamp = now;
  
  //Use our service we created in move_base to get "safe goals" and their costs (100 --> not accessible)
  ros::ServiceClient client_check = n.serviceClient<navigation_experiment::CheckNavigationGoal>("/move_base/global_costmap/check_goal_layer/check_nav_goal");
  navigation_experiment::CheckNavigationGoal checkGoal_srv; 

  //get our x and y for our goal
  double goal_x = goal[0];
  double goal_y = goal[1];

  //whether or not we've found the goal
  bool goal_found[7];

  //if its already a good spot then we're done
  ROS_INFO("Checking if goal is okay...");
  bool already_safe = false;

  //check the costmap at the goal
  checkGoal_srv.request.x = goal_x;
  checkGoal_srv.request.y = goal_y;
  client_check.call(checkGoal_srv);

  if(checkGoal_srv.response.safe == true) {
    ROS_INFO("Goal is okay");
    already_safe = true;
  }
  else { //closest safe point to the north, south, east, west respectively
    ROS_INFO("Goal needs to be moved");
    
    //Ask the plugin for the bounds on the map
    int min_bound_x = checkGoal_srv.response.min_x;
    int max_bound_x = checkGoal_srv.response.max_x;
    int min_bound_y = checkGoal_srv.response.min_y;
    int max_bound_y = checkGoal_srv.response.max_y;

    goal_found[0] = false;
    while(goal_found[0] == false && checkGoal_srv.request.x > min_bound_x && checkGoal_srv.request.x < max_bound_x                    	        && checkGoal_srv.request.y > min_bound_y && checkGoal_srv.request.y < max_bound_y) {
      ROS_INFO("Starting goal 0");
      checkGoal_srv.request.x -= RESOLUTION;
      client_check.call(checkGoal_srv);
      if(checkGoal_srv.response.safe == true) {
        getplan_srv.request.goal.pose.position.x = checkGoal_srv.request.x;
        getplan_srv.request.goal.pose.position.y = checkGoal_srv.request.y;
        client_getplan.call(getplan_srv);
        int size = getplan_srv.response.plan.poses.size();
        if(size == 0) {
           continue;
         }
         costs[0] = size;
         safe_goal_msg[0].target_pose.pose.position.x = checkGoal_srv.request.x;
         safe_goal_msg[0].target_pose.pose.position.y = checkGoal_srv.request.y;
         checkGoal_srv.request.x = goal_x;
         goal_found[0] = true;
      }
    }
    goal_found[1] = false;
    while(goal_found[1] == false && checkGoal_srv.request.x > min_bound_x && checkGoal_srv.request.x < max_bound_x                    	        && checkGoal_srv.request.y > min_bound_y && checkGoal_srv.request.y < max_bound_y) {
      ROS_INFO("Starting goal 1");
      checkGoal_srv.request.x += RESOLUTION;
      client_check.call(checkGoal_srv);
      if(checkGoal_srv.response.safe == true) {
        getplan_srv.request.goal.pose.position.x = checkGoal_srv.request.x;
        getplan_srv.request.goal.pose.position.y = checkGoal_srv.request.y;
        client_getplan.call(getplan_srv);
        int size = getplan_srv.response.plan.poses.size();
        if(size == 0) {
           continue;
         }
         costs[1] = size;
         safe_goal_msg[1].target_pose.pose.position.x = checkGoal_srv.request.x;
         safe_goal_msg[1].target_pose.pose.position.y = checkGoal_srv.request.y;
         checkGoal_srv.request.x = goal_x;
         goal_found[1] = true;
      }
    }
    goal_found[2] = false;
    while(goal_found[2] == false && checkGoal_srv.request.x > min_bound_x && checkGoal_srv.request.x < max_bound_x                    	        && checkGoal_srv.request.y > min_bound_y && checkGoal_srv.request.y < max_bound_y) {
      ROS_INFO("Starting goal 2");
      checkGoal_srv.request.y -= RESOLUTION;
      client_check.call(checkGoal_srv);
      if(checkGoal_srv.response.safe == true) {
        getplan_srv.request.goal.pose.position.x = checkGoal_srv.request.x;
        getplan_srv.request.goal.pose.position.y = checkGoal_srv.request.y;
        client_getplan.call(getplan_srv);
        int size = getplan_srv.response.plan.poses.size();
        if(size == 0) {
           continue;
         }
         costs[2] = size;
         safe_goal_msg[2].target_pose.pose.position.x = checkGoal_srv.request.x;
         safe_goal_msg[2].target_pose.pose.position.y = checkGoal_srv.request.y;
         checkGoal_srv.request.y = goal_y;
         goal_found[2] = true;
      }
    }
    goal_found[3] = false;
    while(goal_found[3] == false && checkGoal_srv.request.x > min_bound_x && checkGoal_srv.request.x < max_bound_x                    	        && checkGoal_srv.request.y > min_bound_y && checkGoal_srv.request.y < max_bound_y) {
      ROS_INFO("Starting goal 3");
      checkGoal_srv.request.y += RESOLUTION;
      client_check.call(checkGoal_srv);
      if(checkGoal_srv.response.safe == true) {
        getplan_srv.request.goal.pose.position.x = checkGoal_srv.request.x;
        getplan_srv.request.goal.pose.position.y = checkGoal_srv.request.y;
        client_getplan.call(getplan_srv);
        int size = getplan_srv.response.plan.poses.size();
        if(size == 0) {
           continue;
         }
         costs[3] = size;
         safe_goal_msg[3].target_pose.pose.position.x = checkGoal_srv.request.x;
         safe_goal_msg[3].target_pose.pose.position.y = checkGoal_srv.request.y;
         checkGoal_srv.request.y = goal_y;
         goal_found[3] = true;
      }
    }
/*
    //closest point to the robot thats also accessible (expanding circle search)
    float radius = RESOLUTION;
    bool done = false;

    while(!done) {
      ROS_INFO("CHECKING CIRCLE OF RADIUS: %f", radius);
      //start at the top of the circle
      checkGoal_srv.request.x = goal_x;
      checkGoal_srv.request.y = goal_y + radius;

      //start the derivative at 1 - R
      float d = RESOLUTION - radius;

      //move your circle to be centered at the origin
      float relative_x = checkGoal_srv.request.x - goal_x;
      float relative_y = checkGoal_srv.request.y - goal_y;

      while(relative_x <= relative_y) { //our last point on the eigth is when x=y
        //check if we have a safe spot using symmetry of the circle
        checkGoal_srv.request.x = goal_x + relative_x; 
        checkGoal_srv.request.y = goal_y + relative_y; 
        client_check.call(checkGoal_srv);
        if (checkGoal_srv.response.safe == true) {
          //check if safe point is accessible
          getplan_srv.request.goal.pose.position.x = checkGoal_srv.request.x;;
          getplan_srv.request.goal.pose.position.y = checkGoal_srv.request.y;;
          client_getplan.call(getplan_srv);
          int size = getplan_srv.response.plan.poses.size();
          if(size != 0) {
            costs[4] = size;
            done = true;
            break;
          }
        } //1 + +
 
        checkGoal_srv.request.x = goal_x + relative_x; 
        checkGoal_srv.request.y = goal_y - relative_y; 
        client_check.call(checkGoal_srv);
        if (checkGoal_srv.response.safe == true) {
          //check if safe point is accessible
          getplan_srv.request.goal.pose.position.x = checkGoal_srv.request.x;;
          getplan_srv.request.goal.pose.position.y = checkGoal_srv.request.y;;
          client_getplan.call(getplan_srv);
          int size = getplan_srv.response.plan.poses.size();
          if(size != 0) {
            costs[4] = size;
            done = true;
            break;
          }
        } //2 + -
       
        checkGoal_srv.request.x = goal_x - relative_x; 
        checkGoal_srv.request.y = goal_y + relative_y; 
        client_check.call(checkGoal_srv);
        if (checkGoal_srv.response.safe == true) {
          //check if safe point is accessible
          getplan_srv.request.goal.pose.position.x = checkGoal_srv.request.x;;
          getplan_srv.request.goal.pose.position.y = checkGoal_srv.request.y;;
          client_getplan.call(getplan_srv);
          int size = getplan_srv.response.plan.poses.size();
          if(size != 0) {
            costs[4] = size;
            done = true;
            break;
          }
        } //3 - +

        checkGoal_srv.request.x = goal_x - relative_x; 
        checkGoal_srv.request.y = goal_y - relative_y; 
        client_check.call(checkGoal_srv);
        if (checkGoal_srv.response.safe == true) {
          //check if safe point is accessible
          getplan_srv.request.goal.pose.position.x = checkGoal_srv.request.x;;
          getplan_srv.request.goal.pose.position.y = checkGoal_srv.request.y;;
          client_getplan.call(getplan_srv);
          int size = getplan_srv.response.plan.poses.size();
          if(size != 0) {
            costs[4] = size;
            done = true;
            break;
          }
        } //4 - -

        checkGoal_srv.request.x = goal_x + relative_y; 
        checkGoal_srv.request.y = goal_y + relative_x; 
        client_check.call(checkGoal_srv);
        if (checkGoal_srv.response.safe == true) {
          //check if safe point is accessible
          getplan_srv.request.goal.pose.position.x = checkGoal_srv.request.x;;
          getplan_srv.request.goal.pose.position.y = checkGoal_srv.request.y;;
          client_getplan.call(getplan_srv);
          int size = getplan_srv.response.plan.poses.size();
          if(size != 0) {
            costs[4] = size;
            done = true;
            break;
          }
        } //5 + + R

        checkGoal_srv.request.x = goal_x + relative_y; 
        checkGoal_srv.request.y = goal_y - relative_x; 
        client_check.call(checkGoal_srv);
        if (checkGoal_srv.response.safe == true) {
          //check if safe point is accessible
          getplan_srv.request.goal.pose.position.x = checkGoal_srv.request.x;;
          getplan_srv.request.goal.pose.position.y = checkGoal_srv.request.y;;
          client_getplan.call(getplan_srv);
          int size = getplan_srv.response.plan.poses.size();
          if(size != 0) {
            costs[4] = size;
            done = true;
            break;
          }
        } //6 + - R

        checkGoal_srv.request.x = goal_x - relative_y; 
        checkGoal_srv.request.y = goal_y + relative_x; 
        client_check.call(checkGoal_srv);
        if (checkGoal_srv.response.safe == true) {
          //check if safe point is accessible
          getplan_srv.request.goal.pose.position.x = checkGoal_srv.request.x;;
          getplan_srv.request.goal.pose.position.y = checkGoal_srv.request.y;;
          client_getplan.call(getplan_srv);
          int size = getplan_srv.response.plan.poses.size();
          if(size != 0) {
            costs[4] = size;
            done = true;
            break;
          }
        } //7 - + R

        checkGoal_srv.request.x = goal_x - relative_y; 
        checkGoal_srv.request.y = goal_y - relative_x; 
        client_check.call(checkGoal_srv);
        if (checkGoal_srv.response.safe == true) {
          //check if safe point is accessible
          getplan_srv.request.goal.pose.position.x = checkGoal_srv.request.x;;
          getplan_srv.request.goal.pose.position.y = checkGoal_srv.request.y;;
          client_getplan.call(getplan_srv);
          int size = getplan_srv.response.plan.poses.size();
          if(size != 0) {
            costs[4] = size;
            done = true;
            break;
          }
        } //8 -- R

        //if not move along the circle
        checkGoal_srv.request.x = goal_x + relative_x; 
        checkGoal_srv.request.y = goal_y + relative_y; 
        if(d >= 0) { //above the midpoint move southeast
          d = d + 2 * checkGoal_srv.request.x - 2 * checkGoal_srv.request.y + 5;
          checkGoal_srv.request.x += RESOLUTION;
          checkGoal_srv.request.y -= RESOLUTION;
        }
        else { //move east
          d = d + 2 * checkGoal_srv.request.x + 3;
          checkGoal_srv.request.x += RESOLUTION;
        }  
        relative_x = checkGoal_srv.request.x - goal_x;
        relative_y = checkGoal_srv.request.y - goal_y;           
      }
      radius += RESOLUTION;
    } 

    safe_goal_msg[4].target_pose.pose.position.x = checkGoal_srv.request.x;
    safe_goal_msg[4].target_pose.pose.position.y = checkGoal_srv.request.y;
*/
    //find the best goal to use based on the cost
    int min_cost = 10000;
    for(int i = 0; i < 1; i++) {
      float dist = sqrt((safe_goal_msg[i].target_pose.pose.position.x - goal[0])*(safe_goal_msg[i].target_pose.pose.position.x - goal[0]) + (safe_goal_msg[i].target_pose.pose.position.y - goal[1])*(safe_goal_msg[i].target_pose.pose.position.y - goal[1]));
      ROS_INFO("The cost of %d is: %d", i, costs[i]);
      costs[i] = costs[i] + 20 * dist;
      ROS_INFO("The cost of %d is: %d", i, costs[i]);
      if(costs[i]  < min_cost) {
        min_cost = costs[i];
        desired_goal = i;
      } 
    }
  }

  ROS_INFO("Got all the points");
  /*
  ROS_INFO("Waiting 6 seconds");
  for(int i = 6; i > 0; i--) {
     ROS_INFO("%d", i);
     unsigned int microseconds;
     sleep(1);
   }
  */
//POINT FIND ALGORITHM ENDS

  //Set the shape for the markers we'll be publishing to rviz later
  uint32_t shape_test = visualization_msgs::Marker::CYLINDER;
  uint32_t shape[10];
  for(int i = 0; i <10; i++) {
    if(costs[i] != 100)
      shape[i] = visualization_msgs::Marker::CYLINDER;
    else
      shape[i] = visualization_msgs::Marker::SPHERE;
  }

  ros::Rate r(10.0);

  //set up markers for rviz
  visualization_msgs::Marker marker_test;
  visualization_msgs::Marker marker[10];

  marker_test.header.frame_id = "/map";
  marker_test.header.stamp = now;
  for(int i = 0; i <10; i++) {
    marker[i].header.frame_id = "/map";
    marker[i].header.stamp = now;
  }

  marker_test.ns = "test_shapes";
  marker_test.id = 0; 
  for(int i = 0; i <10; i++) {
    std::string marker_name_space;
    std::stringstream sstm;
    sstm << "basic_shapes_" << i;
    marker_name_space = sstm.str();    

    marker[i].ns = marker_name_space;
    marker[i].id = 0;
  }

  marker_test.type = shape_test;
  for(int i = 0; i <10; i++) {
    marker[i].type = shape[i];
  }

  marker_test.action = visualization_msgs::Marker::ADD;
  for(int i = 0; i <10; i++) {
    marker[i].action = visualization_msgs::Marker::ADD;
  }
  
  marker_test.pose = goal_msg.target_pose.pose;
  for(int i = 0; i <10; i++) {
    marker[i].pose = safe_goal_msg[i].target_pose.pose;
  }
  marker_test.scale.x = 0.1;
  marker_test.scale.y = 0.1;
  marker_test.scale.z = 0.6;   

  for(int i = 0; i <10; i++) {
    marker[i].scale.x = 0.1;
    marker[i].scale.y = 0.1;
    marker[i].scale.z = 0.6;
  }
  marker[4].scale.x = 0.05;
  marker[4].scale.y = 0.05;
  marker[4].scale.z = 1;
 
  marker_test.color.r = 1.0f;
  marker_test.color.g = 0.0f;
  marker_test.color.b = 0.0f;
  marker_test.color.a = 1.0;
  for(int i = 0; i <10; i++) {
    marker[i].color.r = 1.0f;
    marker[i].color.g = 0.0f;
    marker[i].color.b = 1.0f;
    marker[i].color.a = 1.0;
  } 
  marker[4].color.r = 1.0f;
  marker[4].color.g = 1.0f;
  marker[4].color.b = 1.0f;
  marker[4].color.a = 1.0;  

  marker_test.lifetime = ros::Duration();
  for(int i = 0; i <10; i++) {
    marker[i].lifetime = ros::Duration();
  }

  //publish the markers
  marker_pub.publish(marker_test);
  for(int i = 0; i < 5; i++) {
    marker_pub.publish(marker[i]);
  }     

  //set the new navigation goal
  if(already_safe == false) {
    //set the new angle to be looking at the goal
    goal_msg.target_pose.pose.position.x = safe_goal_msg[desired_goal].target_pose.pose.position.x;
    goal_msg.target_pose.pose.position.y = safe_goal_msg[desired_goal].target_pose.pose.position.y;
  }

  //send the navigation goal
  ROS_INFO("Sending Navigation Goal");

  SimpleNavigationGoals client_send;
  
  //get close to the position
  client_send.send(goal_msg);      
  client_send.ac.waitForResult();	

  //set the new goal to be where we are positioned
  goal_msg.target_pose.pose.position.x = (transform.getOrigin().x() + goal_msg.target_pose.pose.position.x)/2;
  goal_msg.target_pose.pose.position.y = (transform.getOrigin().y() + goal_msg.target_pose.pose.position.y)/2;

  //spin in place publish twist message to cmd_vel
  double yaw0,pitch0,roll0;
  double yaw1,pitch1,roll1;   
  ros::Publisher velocity_command = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  try{
    listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/base_link", "/map", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  tf::Quaternion q1 = transform.getRotation();
  tf::Matrix3x3 m0(q1);
  m0.getEulerYPR(yaw1,pitch1,roll1);

  tf::Quaternion q0(goal_msg.target_pose.pose.orientation.x, goal_msg.target_pose.pose.orientation.y,                                   
                      goal_msg.target_pose.pose.orientation.z, goal_msg.target_pose.pose.orientation.w);
  tf::Matrix3x3 m1(q0);
  m1.getEulerYPR(yaw0,pitch0,roll0);

  yaw0 = 1.5;
  double angle_difference = fabs(yaw0 - yaw1) / (2*PI);
   

  geometry_msgs::Twist spin;
  spin.linear.x = 0;
  spin.linear.y = 0;
  spin.linear.z = 0;  
  spin.angular.x = 0;
  spin.angular.y = 0;
  if(yaw0 < yaw1)
    spin.angular.z = 1;
  else
    spin.angular.z = -1;

  ROS_INFO("The angle differnece is: %f", angle_difference);
  while( angle_difference > 0.1) {
    try{
      listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(10.0) );
      listener.lookupTransform("/base_link", "/map", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    q1 = transform.getRotation();
    tf::Matrix3x3 m2(q1);
    m2.getEulerYPR(yaw1,pitch1,roll1);

    tf::Quaternion q3(goal_msg.target_pose.pose.orientation.x, goal_msg.target_pose.pose.orientation.y,                                   
                        goal_msg.target_pose.pose.orientation.z, goal_msg.target_pose.pose.orientation.w);
    tf::Matrix3x3 m3(q3);
    m3.getEulerYPR(yaw0,pitch0,roll0); 
      yaw0 = 1.3;
      angle_difference = fabs(yaw0 - yaw1) / (2*PI);
      ROS_INFO("The angle differnece is: %f", angle_difference);
      ROS_INFO("Tangy's angle is: %f", yaw1);

      velocity_command.publish(spin);
  }
  spin.angular.z = 0;
  velocity_command.publish(spin);
  return 0;
}


