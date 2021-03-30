#ifndef __TEXT_TO_SPEECH_SERVER_H__
#define __TEXT_TO_SPEECH_SERVER_H__

#include <string>
#include <text_to_speech/tts.h>
#include <vector>
#include <tr1/functional>
#include <audio_player.h>
#include <actionlib/server/simple_action_server.h>
#include <text_to_speech/txt_to_speechAction.h>

class TextToSpeechServer {

	public:

		// creates a new server
        TextToSpeechServer(std::string name);

		// initializes server. checks whether the folder
		// for the resources exists and creates it if
		// necessary
		bool init(std::string resourceDir);

		// handles query. returns true if the query was handled succesfull.
		// otherwise returns false and writes an errormessage to error
		bool handleQuery(std::string text, std::string language,
			int8_t gender, std::string *error);

		// callback on ros tts request 
		bool speak_callback(text_to_speech::tts::Request &req, 
			text_to_speech::tts::Response &res);

	private:
		//Jacob's audio player using SDL		
		audioPlayer player;
		// the path of the resources
		std::string _resourceDir;
        // create txt_to_speech action

        ros::NodeHandle nh;
        actionlib::SimpleActionServer<text_to_speech::txt_to_speechAction> as_;
        std::string action_name_;
        text_to_speech::txt_to_speechFeedback feedback_;
        text_to_speech::txt_to_speechResult result_;
        void goalCB();

		// splits a text if it is larger then maxLength. First tries
		// splitting by '. and ! and ?' then ', and ;' then ' ' and
		// then in spaces.
		std::vector<std::string> splitText(std::string text, int maxLength);

		// hash function
		std::tr1::hash<std::string> hash_fn;

		// plays the given file and returns whether the file could be played
		bool playFile(std::string path);

		// checks whether the folder exists and creates it if not. returns
		// true if the folder exists or has been created.
		bool ensureFolderExists(std::string folder);

		// downloads the text file using wget.
		// returns true if successfull
		bool downloadFile(std::string file, std::string text, std::string language,
			int8_t gender);

        // Backup festival voice just in case we don't have internet or something bad happens to the voice
        void festival_speech(std::string speech);
};

#endif
