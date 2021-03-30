//============================================================================
// Name        : ArmMovementClient.hpp
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description :Send goals to arms; use arms_moving() to see if arms are moving
//============================================================================

#ifndef ArmMovementClient_H_
#define ArmMovementClient_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <drrobot_h20_arm_player/armAction.h>
#include <string>

class ArmMovementClient{


public:

  int state;

    ArmMovementClient(std::string name):
        //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
        ac("ArmMovement", true),
        //Stores the name
        action_name(name)
    {
      state=0;
        //Get connection to a server
        ROS_INFO("%s Waiting For Server...", action_name.c_str());
        //Wait for the connection to be valid
        ac.waitForServer();
        ROS_INFO("%s Got a Server...", action_name.c_str());
    }


    ArmMovementClient(std::string name,float wait_time):
        //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
        ac("ArmMovement", true),
        //Stores the name
        action_name(name)
    {
      state=0;
        //Get connection to a server
        ROS_INFO("%s Waiting For Server...", action_name.c_str());
        //Wait for the connection to be valid
        if( ac.waitForServer(ros::Duration(wait_time)) ){
            ROS_INFO("%s Got a Server...", action_name.c_str());
        } else {
            ROS_INFO("%s Failed to get a Server...", action_name.c_str());
        }

    }

    // Called once when the goal completes
    void doneCb(const actionlib::SimpleClientGoalState& state,
            const drrobot_h20_arm_player::armResultConstPtr& result)
    {
        ROS_INFO("Feedback worked! Finished moving arms");
    }

    // Called once when the goal becomes active
    void activeCb()
    {
        ROS_INFO("Goal just went active...");
    }

    // Called every time feedback is received for the goal
    void feedbackCb(const drrobot_h20_arm_player::armFeedbackConstPtr& feedback)
    {
      ROS_INFO("Client has received Feedback from server!");
      if(feedback->state==0){
        ROS_INFO("Arms are not moving!");
        state=0;
      }else if(feedback->state==1){
        ROS_INFO("Arms are moving!");
        state=1;
      }

    }

    //Send a goal to the server
    void send(double x, double y, double z)
    {
        drrobot_h20_arm_player::armGoal newGoal;

        newGoal.x = x;
        newGoal.y = y;
        newGoal.z = z;
        newGoal.behavior="neutral";

    //Once again, have to used boost::bind because you are inside a class
    ac.sendGoal(newGoal, boost::bind(&ArmMovementClient::doneCb, this, _1, _2),
    boost::bind(&ArmMovementClient::activeCb, this),
    boost::bind(&ArmMovementClient::feedbackCb, this, _1));
    }
    void send(std::string behavior)
    {
        drrobot_h20_arm_player::armGoal newGoal;

        newGoal.behavior=behavior;
        newGoal.x=100;
        newGoal.y=100;
        newGoal.z=100;

    //Once again, have to used boost::bind because you are inside a class
    ac.sendGoal(newGoal, boost::bind(&ArmMovementClient::doneCb, this, _1, _2),
    boost::bind(&ArmMovementClient::activeCb, this),
    boost::bind(&ArmMovementClient::feedbackCb, this, _1));
    }
    bool arms_moving(){
      if(state==0){
        return false;
      }else{
        return true;
      }
    }
/*    void move()
    {
      std::string msg;
      msg="move";
    //Once again, have to used boost::bind because you are inside a class
    ac.sendGoal(msg, boost::bind(&ArmMovementClient::doneCb, this, _1, _2),
    boost::bind(&ArmMovementClient::activeCb, this),
    boost::bind(&ArmMovementClient::feedbackCb, this, _1));
    }
    */
private:
    actionlib::SimpleActionClient<drrobot_h20_arm_player::armAction> ac;
    std::string action_name;
};

#endif
