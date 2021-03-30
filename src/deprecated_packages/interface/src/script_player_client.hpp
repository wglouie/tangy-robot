//============================================================================
// Name        : drrobot_script_teleop.cpp
// Author      : Ananias Paiva
// Version     :
// Copyright   : Your copyright notice
// Description : Control arms using poses written on a script
//============================================================================

#ifndef SCRIPT_PLAYER_CLIENT_HPP_
#define SCRIPT_PLAYER_CLIENT_HPP_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <script_player/ScriptPlayAction.h>
#include <string>

class ScriptClient
{

    public:
        actionlib::SimpleActionClient<script_player::ScriptPlayAction> ac;
	std::string action_name;

    	ScriptClient(std::string name);
    	
    	~ScriptClient();
    	
    	void send(std::string goal);
	void feedbackCb(const script_player::ScriptPlayFeedbackConstPtr& feedback);
	void activeCb();
	void doneCb(const actionlib::SimpleClientGoalState& state,
			const script_player::ScriptPlayResultConstPtr& result);
	
};

#endif
