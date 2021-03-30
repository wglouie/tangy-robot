#include <BingoGameFullServer/BingoGameFullServer.h>


void BingoGameFullServer::play_music(std::string type){
  musicPlayer.play_music(type);
}
void BingoGameFullServer::stop_music(){
  musicPlayer.stop_music();
}
void BingoGameFullServer::pause_music(){
  musicPlayer.pause_music();
}
void BingoGameFullServer::resume_music(){
  musicPlayer.resume_music();
}

void BingoGameFullServer::set_music_volume(int vol){
  musicPlayer.change_vol(vol);
}
	
