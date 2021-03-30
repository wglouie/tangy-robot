#include <BingoGameFullServer/BingoGameFullServer.h>

/*Move base ActionClient
Handles all calls for moving robot to a certain location*/
void BingoGameFullServer::move(std::string frame, float x, float y){
	#ifdef MOVE
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	// moveBaseClient.sendGoalAndWait(goal);
	navClient.move(frame,x,y);
	//Give some time for localization
	//sleep(3);
	#endif
}

void BingoGameFullServer::rotate_no_wait(std::string frame, float ang){
	#ifdef MOVE
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	navClient.rotate_no_wait(frame, ang);
	//Give some time for localization
// 	sleep(3);
	#endif
}

void BingoGameFullServer::rotate(std::string frame, float ang){
	#ifdef MOVE
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	navClient.rotate(frame, ang);
	//Give some time for localization
// 	sleep(3);
	#endif
}

void BingoGameFullServer::face(float x, float y){
	#ifdef MOVE
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	navClient.face("map", x, y);
	//Give some time for localization
// 	sleep(3);
	#endif
}

void BingoGameFullServer::rotate_manual(double ang){
	#ifdef MOVE
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	navClient.rotate_manual(ang);
	//Give some time for localization
// 	sleep(3);
	#endif
  
}

void BingoGameFullServer::move_straight(float distance){
	#ifdef MOVE
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	navClient.move_straight(distance);
	rotate("map", 0);
	#endif
}

void BingoGameFullServer::move_straight_no_rotate(float distance){
	#ifdef MOVE
	while(armMovementClient.arms_moving()){
		//wait for arms to stop moving
	}
	navClient.move_straight(distance);
	#endif
}

/*Move to a certain location to help someone*/
void BingoGameFullServer::move_to_help(float x, float y){
    reset_head_pos();
    ROS_INFO("Moving to help at location: [%f, %f]!", x, y);
        move("map", x -0.20, y);	// -0.15 User Studies
		rotate("map", 0);
        // move_straight(0.3);                 JACOB
		if(pause_game){
    		pause_everything();
		}
}

void BingoGameFullServer::move_to_orig_pos(){
    reset_head_pos();
    debug_print("move_to_orig_pos");
    rotate("map", 3.1415);
    move("map", 0, 0);
    rotate("map", 0);
}

void BingoGameFullServer::rotate_to_face(){
  /*ROS_INFO("Rotating to face.");
	faceDetectionClient.send(10);
	double begin=ros::Time::now().toSec();
	bool timeout=false;

	while(!faceDetectionClient.goal_completed()&& timeout!=true) {
		double end=ros::Time::now().toSec();
		if(end>(begin+2)){
		  timeout=true;
		}
	}
	if((faceDetectionClient.person_of_interest()).compare("No Users")!=0) {

		std::vector<int> pos_of_face=faceDetectionClient.get_person_of_interest_position();
    while((pos_of_face[1]>340||pos_of_face[1]<300)&&(faceDetectionClient.person_of_interest()).compare("No Users")!=0){
      ROS_INFO("The face is at (%d,%d)",pos_of_face[0],pos_of_face[1]);
    	pos_of_face=faceDetectionClient.get_person_of_interest_position();
  		if(pos_of_face[1]<320){
          rotate_manual(0.5);
          // ROS_INFO("Face received, rotating left");
  		}else{
          rotate_manual(-0.5);
          // ROS_INFO("Face received, rotating right");
  		}
    	faceDetectionClient.send(1);
    	double begin=ros::Time::now().toSec();
    	bool timeout=false;
    
    	while(!faceDetectionClient.goal_completed()&& timeout!=true) {
    		double end=ros::Time::now().toSec();
    		if(end>(begin+0.5)){
    		  timeout=true;
    		}
    	}
    }
    ROS_INFO("Done");
    rotate_manual(0);

	}
*/
}
