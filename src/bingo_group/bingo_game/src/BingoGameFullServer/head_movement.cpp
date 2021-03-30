#include <BingoGameFullServer/BingoGameFullServer.h>

/*Reset head position*/
void BingoGameFullServer::reset_head_pos() {
	#ifdef MOVE_HEAD
	neckHandler.reset_neck_pos();
	#endif
}

void BingoGameFullServer::rand_head_pos() {
	#ifdef MOVE_HEAD
	neckHandler.rand_neck_pos();
	#endif
}

void BingoGameFullServer::look_down() {
	#ifdef MOVE_HEAD
	neckHandler.look_down();
	#endif
}

void BingoGameFullServer::look_at_card_1() {
	#ifdef MOVE_HEAD
	neckHandler.look_at_card_1();
	#endif
}

void BingoGameFullServer::look_at_card_2() {
	#ifdef MOVE_HEAD
	neckHandler.look_at_card_2();
	#endif
}

void BingoGameFullServer::look_at_card_3() {
	#ifdef MOVE_HEAD
	neckHandler.look_at_card_3();
	#endif
}

void BingoGameFullServer::look_back_to_person() {
	#ifdef MOVE_HEAD
	neckHandler.look_back_to_person();
	#endif
}

void BingoGameFullServer::ready_to_nod(){
 	#ifdef MOVE_HEAD
	std_msgs::Bool msg;
	msg.data=true;
	nod_pub.publish(msg);
	#endif
}

void BingoGameFullServer::not_ready_to_nod(){
 	#ifdef MOVE_HEAD
	std_msgs::Bool msg;
	msg.data=false;
	nod_pub.publish(msg);
	#endif
}