#include "ros/ros.h"
#include "text_to_speech/tts.h"
#include <text_to_speech/TextToSpeechClient.h>

int main(int argc, char **argv)
{
	// initialize ros
	ros::init(argc, argv, "tts_example");

    //Initialize the client
    TextToSpeechClient client(ros::this_node::getName());
    client.send_action_goal("I am a winner of the world");
    //client.exe_spch_and_gest("Hello world", "action_executor", "/database/demonstration_gestures/psi_pose");

/*
	// check parameters
	if (argc != 5)
	{
		ROS_INFO("usage: tts_example [lang] [gender] [text] [webbased (true, false)]");
		return 1;
	}

	// create node handler
	ros::NodeHandle n;

	// service client for text_to_speech
	ros::ServiceClient client =
		n.serviceClient<text_to_speech::tts>("tts");

	// create and initialize request
	text_to_speech::tts srv;
	srv.request.speak = argv[3];
	srv.request.language = argv[1]; //en for male 
	if(strcmp(argv[2], "m") == 0){
		srv.request.gender = text_to_speech::tts::Request::MALE;
	} else {
		srv.request.gender = text_to_speech::tts::Request::FEMALE;
	}
	bool webbased;
	if(strcmp(argv[4], "true") == 0){
		srv.request.webbased = true;
	} else {
		srv.request.webbased = false;
	}

	// call request and dump whether it was successfull or not.
	if (client.call(srv))
	{
		if(srv.response.success){
			ROS_INFO("Success!");
		} else {
			ROS_INFO("Error!");
			ROS_ERROR(srv.response.error.c_str());
		}
	}
	else
	{
		ROS_ERROR("Failed to call service. Error:");
		return 1;
	}
*/
	return 0;
}

