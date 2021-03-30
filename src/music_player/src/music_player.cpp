//
// Requires the use of libSDL_mixer; available from libsdl devel packages
//
// sudo apt-get install libsdl-mixer1.2-dev libsdl1.2-dev
//
// To build:  g++ `sdl-config --cflags --libs` PlayMusic.cpp -lSDL_mubuntuixer -o play
//
// To run:  play <file.mp3>
//

#include "music_player.h"

musicServer::musicServer() :
as_(nh_, "music_player", false),
		action_name_("music_player") {

	nh_.getParam("/musicFile1", filepath1);
	nh_.getParam("/musicFile2", filepath2);
	nh_.getParam("/musicFile3", filepath3);
	nh_.getParam("/musicFile4", filepath4);
	nh_.getParam("/musicFile5", filepath5);
	nh_.getParam("/musicFile6", filepath6);
	nh_.getParam("/beepSound", beeppath);
	nh_.getParam("/celebrationMusic", celebratepath);

  as_.registerGoalCallback(boost::bind(&musicServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&musicServer::preemptCB, this));
  as_.start();
  if(init()){
    ROS_INFO("Music player initiated!");
  }else{
    ROS_WARN("Music player not initiated!");
  }
}
musicServer::~musicServer(){
  Mix_CloseAudio();
}

bool musicServer::init(){

       // Init
  // SDL_OpenAudio();
   if (SDL_Init(SDL_INIT_AUDIO) != 0)
   {
      std::cerr << "SDL_Init ERROR: " << SDL_GetError() << std::endl;
      return -1;
   }

   // Open Audio device
   if (Mix_OpenAudio(44100, AUDIO_S16SYS, 2, 2048) != 0)
   {
      std::cerr << "Mix_OpenAudio ERROR: " << Mix_GetError() << std::endl;
      return -1;
   }
   Mix_VolumeMusic(80);
}


void musicServer::play_file(std::string filePath){
	if(filePath.size()!=0){
	   Mix_Music *music;
	   music= Mix_LoadMUS(filePath.c_str());

	  //Create a pointer. The load methods will return a pointer.
	/*  SDL_RWops *file;
	  Loads from a file.
	  file = SDL_RWFromFile(filePath.c_str(), "r");
	  ROS_INFO("RWops created!");*/
	   
	  // int freesrc=0;
	  // Mix_Chunk chunk=*Mix_LoadWAV(filePath.c_str());
	  // ROS_INFO("Chunk created!");
	  // Mix_PlayChannel (-1, &chunk, 1);
	  // if (SDL_LoadWAV(celebratepath.c_str(), &wav_spec, &wav_buffer, &wav_length) == NULL) {
	  //   fprintf(stderr, "Could not open test.wav: %s\n", SDL_GetError());
	  //   } else {
	  //       /* Do stuff with the WAV data, and then...
	  //       SDL_FreeWAV(wav_buffer);
	  //   }*/
	    
	    
	   if (music)
	   {
	      // Start Playback
	      if (Mix_PlayMusic(music, 1) == 0)
	      {
      		 unsigned int startTime = SDL_GetTicks();
      
      		 // Wait
      		 while (Mix_PlayingMusic() && !stop_playing)
      		 {
      		    SDL_Delay(500);
      		 }
	      }else{
      		 std::cerr << "Mix_PlayMusic ERROR: " << Mix_GetError() << std::endl;
	      }

  	      // Free File
  	      Mix_FreeMusic(music);
  	      music = 0;
  	   }else{
	      std::cerr << "Mix_LoadMuS ERROR: " << Mix_GetError() << std::endl;
       }
  }

}

void musicServer::goalCB(){
  music_player::music_playerGoal goal=*(as_.acceptNewGoal());
  stop_playing=false;
  std::string state=goal.start;
  int volume = goal.volume;
  if(state.compare("normal")==0){
      while(!stop_playing){
        int i=rand()%6+1;
        switch(i){
          case 1:
            ROS_INFO("Playing music file from [%s]", filepath1.c_str());
            play_file(filepath1);
            break;
          case 2:
            ROS_INFO("Playing music file from [%s]", filepath2.c_str());
            play_file(filepath2);
            break;
          case 3:
            ROS_INFO("Playing music file from [%s]", filepath3.c_str());
            play_file(filepath3);
            break;
          case 4:
            ROS_INFO("Playing music file from [%s]", filepath4.c_str());
            play_file(filepath4);
            break;
          case 5:
            ROS_INFO("Playing music file from [%s]", filepath5.c_str());
            play_file(filepath5);
            break;
          case 6:
            ROS_INFO("Playing music file from [%s]", filepath6.c_str());
            play_file(filepath6);
            break;
          }
      }
    }else if(state.compare("celebrate")==0){
      ROS_INFO("Playing celebration music file!");
      while(!stop_playing){
        Mix_VolumeMusic(100);
        play_file(celebratepath);
      }
    }else if(state.compare("beep")==0){
      ROS_INFO("Playing beep audio file!");
      Mix_VolumeMusic(100);
      play_file(beeppath);
    }else{
      ROS_INFO("Attempting to play file from path: %s", state.c_str());
      play_file(state);
    }
    as_.setSucceeded();
}

void musicServer::preemptCB(){
  Mix_FadeOutMusic(2000);
  stop_playing = true;
  ROS_INFO("Preempted!");
  // as_.setPreempted();
}

void musicServer::volumeCB(const std_msgs::Int32::ConstPtr& vol){
  Mix_VolumeMusic(vol->data);
}

void musicServer::playControlCB(const std_msgs::String::ConstPtr& str){
  std::string state=str->data;
  
  if(state.compare("pause")==0){
    ROS_INFO("Pausing music!");
    Mix_PauseMusic();
  }else if(state.compare("resume")==0){
    ROS_INFO("Resuming music!");
    Mix_ResumeMusic();
  }else if(state.compare("stop")==0){
    ROS_INFO("Stopping music!");
    stop_playing=true;
    Mix_FadeOutMusic(2000);
  }
}

int main(int argc, char** argv){
	ros::init(argc,argv,"music_player");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(0);
	spinner.start();
	srand(time(0));
	musicServer server;
	ros::Subscriber volume_sub=n.subscribe<std_msgs::Int32>("music_volume",10, &musicServer::volumeCB, &server);
	ros::Subscriber music_control_sub=n.subscribe<std_msgs::String>("music_controller",10, &musicServer::playControlCB, &server);

	ros::waitForShutdown();
 	return 0;
}
