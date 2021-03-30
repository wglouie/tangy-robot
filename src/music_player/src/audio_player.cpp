//
// Requires the use of libSDL_mixer; available from libsdl devel packages
//
// sudo apt-get install libsdl-mixer1.2-dev libsdl1.2-dev
//
// To build:  g++ `sdl-config --cflags --libs` PlayMusic.cpp -lSDL_mubuntuixer -o play
//
// To run:  play <file.mp3>
//

#include "audio_player.h"

audioPlayer::audioPlayer() {

  if(init()){
    ROS_INFO("Audio player initiated!");
  }else{
    ROS_WARN("Audio player not initiated!");
  }
}
audioPlayer::~audioPlayer(){
  Mix_CloseAudio();
}

bool audioPlayer::init(){

       // Init

   if (SDL_Init(SDL_INIT_AUDIO) != 0)
   {
      std::cerr << "SDL_Init ERROR: " << SDL_GetError() << std::endl;
      return -1;
   }

   // Open Audio device
   if (Mix_OpenAudio(16000, AUDIO_S16SYS, 2, 2048) != 0) // original voice is 15000 Aug 12, 2014; 17500 Feb 22, 2016
   {
      std::cerr << "Mix_OpenAudio ERROR: " << Mix_GetError() << std::endl;
      return -1;
   }
   Mix_VolumeMusic(100);
}


void audioPlayer::play_file(std::string filePath){
  if(!filePath.empty()){
      ROS_INFO("Playing audio clip from %s!", filePath.c_str());
      stop_playing=false;
      Mix_Music *audio;
      audio= Mix_LoadMUS(filePath.c_str());
      
      if (audio)
      {
        // Start Playback
        if (Mix_PlayMusic(audio, 1) == 0)
        {
           unsigned int startTime = SDL_GetTicks();
      
           // Wait
           while (Mix_PlayingMusic() && !stop_playing)
           {
              SDL_Delay(500);
           }
        }
        else
        {
           std::cerr << "Mix_PlayMusic ERROR: " << Mix_GetError() << std::endl;
        }
      
        // Free File
        Mix_FreeMusic(audio);
        audio = 0;
      }
      else
      {
        std::cerr << "Mix_LoadMuS ERROR: " << Mix_GetError() << std::endl;
      }
  }else{
    ROS_INFO("Invalid file path name: %s!", filePath.c_str());
  }
}

void audioPlayer::volumeCB(const std_msgs::Int32::ConstPtr& vol){
  Mix_VolumeMusic(vol->data);
}

void audioPlayer::playControlCB(const std_msgs::String::ConstPtr& str){
  std::string state=str->data;
  
  if(state.compare("pause")==0){
    ROS_INFO("Pausing audio!");
    Mix_PauseMusic();
  }else if(state.compare("resume")==0){
    ROS_INFO("Resuming audio!");
    Mix_ResumeMusic();
  }else if(state.compare("stop")==0){
    ROS_INFO("Stopping audio!");
    stop_playing=true;
    Mix_FadeOutMusic(2000);
  }
}

int main(int argc, char** argv){
	ros::init(argc,argv,"audio_player");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(0);
	spinner.start();
	audioPlayer server;
	ros::Subscriber volume_sub=n.subscribe<std_msgs::Int32>("audio_volume",10, &audioPlayer::volumeCB, &server);
	ros::Subscriber audio_control_sub=n.subscribe<std_msgs::String>("audio_controller",10, &audioPlayer::playControlCB, &server);

	ros::waitForShutdown();
 	return 0;
}
