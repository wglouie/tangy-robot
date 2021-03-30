#include <BingoGameFullServer/BingoGameFullServer.h>

void BingoGameFullServer::pauseCallback(const std_msgs::String::ConstPtr& str) {
	if(str->data.compare("pause")==0) {
		pause_game=true;
		navClient.pause();
	} else if(str->data.compare("unpause")==0) {
		pause_game=false;
		navClient.unpause();
	}
}

void BingoGameFullServer::pause_everything() {
	while(pause_game) {
		//wait
	}
}
