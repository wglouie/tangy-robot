#include <BingoGameFullServer/BingoGameFullServer.h>

void BingoGameFullServer::plan_wave() {
	debug_print("planning wave");
	armServerHandler("plan wave goodbye");
}
    
void BingoGameFullServer::plan_point_at_screen() {
	debug_print("pointing at screen");
	armServerHandler("plan point at screen");
}

void BingoGameFullServer::plan_celebrate() {
	debug_print("planning celebrate");
	armServerHandler("plan celebrate");
}

void BingoGameFullServer::plan_point_at() {
	debug_print("planning point at");
}

void BingoGameFullServer::plan_convo_gesture() {
	debug_print("planning convo gesture");
	armServerHandler("plan convo gesture");
}

void BingoGameFullServer::plan_laugh_gesture() {
	debug_print("planning laugh gesture");
	armServerHandler("plan laugh gesture");
}
    
void BingoGameFullServer::wave() {
	debug_print("waving");
	armServerHandler("execute wave goodbye");
}
   
void BingoGameFullServer::point_at_screen() {
	debug_print("pointing at screen");
	armServerHandler("execute point at screen");
}
    
void BingoGameFullServer::celebrate() {
	debug_print("celebrating");
	armServerHandler("execute celebrate");
}
    
void BingoGameFullServer::point_at() {
	debug_print("pointing at");
}
    
void BingoGameFullServer::convo_gesture() {
	armServerHandler("execute convo gesture");
}

void BingoGameFullServer::laugh_gesture() {
	armServerHandler("execute laugh gesture");
}
    
void BingoGameFullServer::armServerHandler(std::string goal) {
	#ifdef MOVE_ARMS
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
	}else{
		ROS_WARN("There are no plans stored for this interaction yet: [%s]", goal.c_str());
	}
	#endif
}
