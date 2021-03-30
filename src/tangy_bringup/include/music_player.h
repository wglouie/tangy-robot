//============================================================================
// Name        : music_player.h
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description :
//============================================================================

#ifndef music_player_H_
#define music_player_H_
#include <ros/ros.h>
#include <string>
#include <SDL/SDL.h>
#include <SDL/SDL_mixer.h>
#include <iostream>

class music_player {
  public:
  
    music_player();
    ~music_player();
    
    bool init();
    void setVolume(int vol);
    void play_file(std::string curr_num);

};
#endif