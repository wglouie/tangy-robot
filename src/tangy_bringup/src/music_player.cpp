#include <tangyRobot.h>

void tangyRobot::play_music(std::string type){
  musicPlayer.play_music(type);
}
void tangyRobot::stop_music(){
  musicPlayer.stop_music();
}
void tangyRobot::pause_music(){
  musicPlayer.pause_music();
}
void tangyRobot::resume_music(){
  musicPlayer.resume_music();
}

void tangyRobot::set_music_volume(int vol){
  musicPlayer.change_vol(vol);
}
	
void tangyRobot::beep(){
  musicPlayer.beep();
}