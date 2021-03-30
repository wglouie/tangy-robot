#include <tangyRobot.h>

/*Reset head position*/
void tangyRobot::reset_head_pos() {
	neckHandler.reset_neck_pos();
}

void tangyRobot::rand_head_pos() {
	neckHandler.rand_neck_pos();
}

void tangyRobot::look_down() {
	neckHandler.look_down();
}

void tangyRobot::look_at_card_1(){
    neckHandler.look_at_card_1();
}
void tangyRobot::look_at_card_2(){
    neckHandler.look_at_card_2();
}
void tangyRobot::look_at_card_3(){
    neckHandler.look_at_card_3();
}


void tangyRobot::look_back_to_person() {
	neckHandler.look_back_to_person();
}

void tangyRobot::ready_to_nod(){
	std_msgs::Bool msg;
	msg.data=true;
	nod_pub.publish(msg);
}

void tangyRobot::not_ready_to_nod(){
	std_msgs::Bool msg;
	msg.data=false;
	nod_pub.publish(msg);
}

void tangyRobot::moveNeck(int horizAng, int vertAng){
  neckHandler.turn(horizAng, vertAng);
  
}

void tangyRobot::nod(){
  neckHandler.nod();
  
}
void tangyRobot::shake(){
  neckHandler.shake();
  
}
