#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <text_to_speech/tts.h>
#include <text_to_speech/TextToSpeechServer.h>
#include <tr1/functional>
#include <fstream>

int main(int argc, char **argv)
{

	// initialise ros
    ros::init(argc, argv, "tangy_tts");

	// create name of directory where the audio files are placed.
	// this is ~/.tangy_tts_resource/
	std::string homedir = getenv("HOME");
	std::string resourceDir = homedir+"/.tts_resource/";

	// create text to speech server
    TextToSpeechServer server("text_to_speech_server");
	if(!server.init(resourceDir)){
		
		ROS_ERROR("The TTS-Server could not be initialized!");
		return 1;
	}

	// create node handler and service advertiser
    ros::NodeHandle nh;
	ros::ServiceServer serviceServer = 
		nh.advertiseService("tts",
		&TextToSpeechServer::speak_callback, &server);

	// run
    ros::spin();
    return 0;
}
