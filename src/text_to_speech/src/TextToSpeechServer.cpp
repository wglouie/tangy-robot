#include <text_to_speech/TextToSpeechServer.h>

#include <boost/algorithm/string.hpp>
#include <sys/stat.h>
#include <ros/ros.h>

#define MAX_TEXT_LENGTH 100

TextToSpeechServer::TextToSpeechServer(std::string name) :
    as_(nh, name, false)
  , action_name_(name){
	_resourceDir = "";
    as_.registerGoalCallback(boost::bind(&TextToSpeechServer::goalCB, this));
    as_.start();
    ROS_INFO("Text to Speech action server started");

}

bool TextToSpeechServer::init(std::string resourceDir){

	// remember dir
	_resourceDir = resourceDir;

	// check whether the directory exists
	if(!ensureFolderExists(_resourceDir)){
		ROS_WARN("Creating resource folder failed!");
		return false;
	}

	// ok
	return true;
}

bool TextToSpeechServer::handleQuery(std::string text, std::string language,
		int8_t gender, std::string *error)
{
// 	ROS_INFO("Text: [%s]", text.c_str());
//         ROS_INFO("Language: %s", language.c_str());
// 	ROS_INFO("Gender: %d", gender);

	// save ressources in _resourceDir/language/f (female) /m (male)
	std::string folder = _resourceDir+"/"+language;

	if(!ensureFolderExists(folder)){
		ROS_WARN("Creating lang folder failed!");
		return false;
	}

	// gender entry
	if(gender == text_to_speech::tts::Request::MALE){
		folder = folder+"/m";
	} else {
		folder = folder+"/f";
	}

	if(!ensureFolderExists(folder)){
		ROS_WARN("Creating lang/gender folder failed!");
		return false;
	}
	
	//Check first if complete text already has a downloaded speech file before splitting
	bool fileExists=false;
	std::vector<std::string> playlist;
	std::string fname;
	boost::trim(text);
  ROS_INFO("Text: [%s]", text.c_str());
	// create hash for this file
      size_t texthash = hash_fn(text);
	// file path
	std::stringstream buff;
	buff << folder << "/" << texthash << ".mp3";
	fname = buff.str();
	ROS_INFO(fname.c_str());
	// check if the file exists
	struct stat sb0;
	if(stat(fname.c_str(), &sb0) == 0 && !S_ISDIR(sb0.st_mode)){

		// check whether the file is larger than 0 byte
 		fileExists = sb0.st_size != 0;
	}
	if(fileExists){
    playlist.push_back(fname);
	}else{
	// split text if necessary
  	std::vector<std::string> textFragments = splitText(text, MAX_TEXT_LENGTH);
  
  	// create file for each sentence and create playlist
  	std::string file;
  	std::string fragment;
  	
  
  	for(int i = 0; i<textFragments.size(); i++){
  
  		fragment = textFragments.at(i);
  		boost::trim(fragment);
      ROS_INFO("Text: [%s]", fragment.c_str());
  		// create has for this file
          size_t hash = hash_fn(fragment);
  
  		// file path
  		std::stringstream buffer_name;
  		buffer_name << folder << "/" << hash << ".mp3";
  		file = buffer_name.str();
  		ROS_INFO(file.c_str());
  		// check if the file exists. if not: download file
  		// from server
  		bool fileOk = false;
  		struct stat sb;
			ROS_INFO("Stuck before checking file exists/size");
  		if(stat(file.c_str(), &sb) == 0 && !S_ISDIR(sb.st_mode)){
  
  			// check whether the file is larger than 0 byte
   			fileOk = sb.st_size != 0;
  		}
  		if(!fileOk){
  			// does not exist or is 0 byte. try to create
          if(!downloadFile(file, fragment, language, gender)){
                  *error = "Failed to download file using backup festival voice instead";
                festival_speech(fragment);
  				return false;
  			}
  		}
  
  		playlist.push_back(file);
  	}
	}
	// play playlist
	ROS_INFO("Stuck before playing file");
	for(int i = 0; i<playlist.size(); i++){
		if(!playFile(playlist.at(i))){
			*error = "could not play file";
			return false;
		}
	}
	return true;
}

void TextToSpeechServer::goalCB(){
    bool success = true;
    text_to_speech::txt_to_speechGoalConstPtr goal = as_.acceptNewGoal();
    bool webbased = goal->webbased;
    std::string error = "ok";
    std::string speech = goal->speech;
    std::string language = goal->language;
    int8_t gender = goal->gender;
    if(webbased){
        success = handleQuery(speech, language,
            gender, &error);
    } else {
        // use festival
        std::string tmp = speech;
        boost::replace_all(tmp, "|" , " ");
        std::string text = "echo \""+tmp+"\" | festival --tts";
        bool success = (system(text.c_str()) == 0);
    }
    result_.success = success;
    result_.error = error;
    as_.setSucceeded(result_);
}

bool TextToSpeechServer::speak_callback(text_to_speech::tts::Request &req,
		text_to_speech::tts::Response &res)
{		
	if(req.webbased){
		// handle query
		std::string error = "ok";
		res.success = handleQuery(req.speak, req.language,
			req.gender, &error);
		res.error = error;
	} else {
		// use festival
		std::string tmp = req.speak;
		boost::replace_all(tmp, "|" , " ");
		std::string text = "echo \""+tmp+"\" | festival --tts";
		res.success = (system(text.c_str()) == 0);
	}
	return true;

}

std::vector<std::string> TextToSpeechServer::splitText(std::string text,
		int maxLength)
{
	// create list
	std::vector<std::string> list;

	// check whether | exist. split there and call splitText on results
	if(boost::contains(text, "|")) {
		std::vector<std::string> tmp, tmpResult;
   		boost::split(tmp,text,boost::is_any_of("|"));
		for(int i = 0; i<tmp.size(); i++){
			tmpResult = splitText(tmp.at(i), maxLength);
			for(int y = 0; y<tmpResult.size(); y++){
				list.push_back(tmpResult.at(y));
			}
		}
		return list;
	}

	int sentenceEnd = -1;
	int sentenceSeparator = -1;
	int space = -1;
	char current;

	while(text.length() > 0){

		if(text.length() < maxLength){

			// text is short enough
			list.push_back(text);
			break;
		}

		// traverse text and remember last occurence of
		// split signs.

		// no signs found yet
		sentenceEnd = -1;
		sentenceSeparator = -1;
		space = -1;
		
		for(int i = 0; i<maxLength; i++){
			current = text.at(i);

			if(current == '.' || current == '!' || current == '?' || current == ':'){
				sentenceEnd = i;
			} else if(current == ',' || current == ';'){
				sentenceSeparator = i;
			} else if(current == ' '){
				space = i;
			}
		}

		int splitPos = maxLength-1;

		// if splitsigns found, use them to split
		if(sentenceEnd != -1){
			splitPos = sentenceEnd;
		} else if(sentenceSeparator != -1){
			splitPos = sentenceSeparator;
		} else if(space != -1){
			splitPos = space;
		}

		// split
		std::string t = text.substr(0, splitPos+1);
		text = text.substr(splitPos+1);
		list.push_back(t);
	}

	return list;
}

bool TextToSpeechServer::playFile(std::string path){
	player.play_file(path);
	return 1;
}

bool TextToSpeechServer::ensureFolderExists(std::string folder){

	// try to create if not existing
	struct stat sb;
	if(!(stat(folder.c_str(), &sb) == 0) || (!S_ISDIR(sb.st_mode))){
		// does not exist. try to create
		std::stringstream buffer;
		buffer << "mkdir " << folder << "" ;
		int n = system(buffer.str().c_str());

		if(n != 0){
			// the folder could not be created
			ROS_ERROR("Could not create folder");
			return false;
		}
	}

	return true;
}

bool TextToSpeechServer::downloadFile(std::string file, std::string text, std::string language,
	int8_t gender)
{
	std::stringstream buffer1;
	buffer1 << "wget -q -U Mozilla -O \"" << file << "\" ";

	/*if(language == "uk"){
		buffer1 << "\"http://translate.google.co.uk/translate_tts?tl=en&q=";
	} else 	if(language == "en" && gender == text_to_speech::tts::Request::MALE){
		buffer1 << "\"http://tts-api.com/tts.mp3?q=";
	} else {
		buffer1 << "\"http://translate.google.com/translate_tts?tl=" << language << "&q=";
	}*/

    buffer1<< "\"https://api.voicerss.org/?key=ee7b4c54b4394d1780ab1ba31a8e9168&&hl=en-us&f=16khz_16bit_stereo&src=";
	buffer1 << text <<"\"";
	int n=system(buffer1.str().c_str());
    ROS_INFO(buffer1.str().c_str());

	if(n!=0){
		return false;
	}

	// check whether the file is larger than 0 byte
	struct stat filestatus;
  	stat(file.c_str(), &filestatus);

 	return filestatus.st_size != 0;
}

void TextToSpeechServer::festival_speech(std::string speech){
    boost::replace_all(speech, "|" , " ");
    std::string text = "echo \""+speech+"\" | festival --tts";
    system(text.c_str());
}
