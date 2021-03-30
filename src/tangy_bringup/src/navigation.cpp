#include <tangyRobot.h>


/*Move base ActionClient
Handles all calls for moving robot to a certain location*/
void tangyRobot::move(std::string frame, float x, float y){
	reset_head_pos();
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	navClient.move(frame,x,y);

}

void tangyRobot::rotate_no_wait(std::string frame, float ang){
	reset_head_pos();
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	navClient.rotate_no_wait(frame, ang);

}

void tangyRobot::rotate(std::string frame, float ang){
	reset_head_pos();
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	navClient.rotate(frame, ang);

}

void tangyRobot::face(float x, float y){
	
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	navClient.face("map", x, y);

}

void tangyRobot::rotate_manual(double ang){
	
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	navClient.rotate_manual(ang);

  
}

void tangyRobot::move_straight(float distance){
	
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	navClient.move_straight(distance);
	rotate("map", 0);
	
}

void tangyRobot::move_straight_no_rotate(float distance){
	
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	navClient.move_straight(distance);
	
}

/*Move to a certain location to help someone*/
void tangyRobot::move_to_help(float x, float y){
    reset_head_pos();
    ROS_INFO("Moving to help at location: [%f, %f]!", x, y);
        move("map", x -0.20, y);	// -0.15 User Studies
		rotate("map", 0);
        // move_straight(0.3);                 JACOB
		while(pause){
		}
}

void tangyRobot::move_to_orig_pos(){
    reset_head_pos();
    rotate("map", 3.1415);
    move("map", 0, 0);
    rotate("map", 0);
}

void tangyRobot::pauseCallback(const std_msgs::String::ConstPtr& str) {
	if(str->data.compare("pause")==0) {
		pause=true;
		navClient.pause();
	} else if(str->data.compare("unpause")==0) {
		pause=false;
		navClient.unpause();
	}
}