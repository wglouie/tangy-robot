//***************************ARMS ARE TURNED OFF UNTIL THEY HAVE BEEN FIXED**********************************************//
//***************************CHANGES ARE IN ARMSERVERHANDLER FUNCTION AT THE BOTTOM OF THE FILE**************************//

#include <tangyRobot.h>
void tangyRobot::plan_wave() {
	armServerHandler("plan wave goodbye");
}

void tangyRobot::plan_clap() {
	armServerHandler("plan clap");
}
    
void tangyRobot::plan_point_at_screen() {
	armServerHandler("plan point at screen");
}

void tangyRobot::plan_celebrate() {
	armServerHandler("plan celebrate");
}

void tangyRobot::plan_point_at() {
	//TODO: Make the robot point at something
}

void tangyRobot::plan_convo_gesture() {
	armServerHandler("plan convo gesture");
}

void tangyRobot::plan_laugh_gesture() {
	armServerHandler("plan laugh gesture");
}
    
void tangyRobot::wave() {
	armServerHandler("execute wave goodbye");
}
   
void tangyRobot::point_at_screen() {
	armServerHandler("execute point at screen");
}
    
void tangyRobot::celebrate() {
	armServerHandler("execute celebrate");
}

void tangyRobot::clap() {
	armServerHandler("execute clap");
}

void tangyRobot::point_at() {
  //TODO: Make the robot point at something
}
    
void tangyRobot::convo_gesture() {
	armServerHandler("execute convo gesture");
}

void tangyRobot::laugh_gesture() {
	armServerHandler("execute laugh gesture");
}
    
void tangyRobot::armServerHandler(std::string goal) {
  if(!armMovementClient.arms_moving()){
  	if(goal.compare("plan wave goodbye")==0) {
  		wave_plan_num++;
  		armMovementClient.send(goal);
  		ROS_INFO("There are [%d] plans for waving stored.", wave_plan_num);
  	} else if(goal.compare("plan point at screen")==0) {
  		point_at_screen_plan_num++;
  		armMovementClient.send(goal);
  		ROS_INFO("There are [%d] plans for point at screen stored.", point_at_screen_plan_num);
  	} else if(goal.compare("plan celebrate")==0) {
  		celebrate_plan_num++;
  		armMovementClient.send(goal);
  		ROS_INFO("There are [%d] plans for celebrate stored.", celebrate_plan_num);
  	} else if(goal.compare("plan convo gesture")==0) {
  		convo_gesture_num++;
  		armMovementClient.send(goal);
  		ROS_INFO("There are [%d] plans for convo gesture stored.", convo_gesture_num);
  	} else if(goal.compare("plan laugh gesture")==0) {
  		laugh_gesture_num++;
  		armMovementClient.send(goal);
  		ROS_INFO("There are [%d] plans for laugh gesture stored.", laugh_gesture_num);
  	} else if(goal.compare("plan clap")==0) {
  		clap_gesture_num++;
  		armMovementClient.send(goal);
  		ROS_INFO("There are [%d] plans for clap stored.", clap_gesture_num);
  		
  	}else if(goal.compare("execute wave goodbye")==0 && wave_plan_num>0) {
  		armMovementClient.send(goal);
  	} else if(goal.compare("execute point at screen")==0 && point_at_screen_plan_num >0) {
  		armMovementClient.send(goal);
  	} else if(goal.compare("execute celebrate")==0 && celebrate_plan_num>0) {
  		armMovementClient.send(goal);
  	} else if(goal.compare("execute convo gesture")==0 && convo_gesture_num>0) {
  		armMovementClient.send(goal);
  	} else if(goal.compare("execute laugh gesture")==0 && laugh_gesture_num>0) {
  		armMovementClient.send(goal);
  	} else if(goal.compare("execute clap gesture")==0 && clap_gesture_num>0) {
  		// armMovementClient.send(goal);
  	}else{
  		ROS_WARN("There are no plans stored for this interaction yet: [%s]", goal.c_str());
  	}
  }
}
