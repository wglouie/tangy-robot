#include <fault_diagnosis/MonitorTemplate.h>
#include <audio_common_msgs/AudioData.h>
#include <stdlib.h>
#include <cstdio>

static std::map<std::string, std::string[FAULT_LIST_LEN]> faultList =
        {{"A1", {"No or incorrect audio ", "1", "1"}}};

static std::vector<std::string> stateList =
        {"GREETING", "CALL NUMBER", "REMOVEMARKER", "MARKNUMBER", "ENCOURAGE", "CELEBRATE", "JOKE", "VALEDICTION", "SHOWBINGOCARD"};

class FaultMonitorAudio : public FaultMonitor<audio_common_msgs::AudioData>{
public:
    FaultMonitorAudio(std::string watched_topic, std::map<std::string, std::string[FAULT_LIST_LEN]>& faultList,
                           std::vector<std::string> stateList) : FaultMonitor<audio_common_msgs::AudioData>(watched_topic, faultList, stateList){}

protected:
    bool thresholdPassed, isActive;

    virtual void GSUpdate(const std_msgs::String::ConstPtr& state){
//        //if the list is empty or the new game state is in the list we start checking the audio //TODO CHECK THIS FOR TRUTH
//        if(activeStates.empty() || std::find(activeStates.begin(), activeStates.end(), state->data) != activeStates.end()){
//            isActive = false; //we are not recording any audio
//            return;
//        }
//        else{
//            if(isActive && !thresholdPassed){ //if the audio recorder was active last state and no audio was heard
//                LogFault("A1");
//            }
//            isActive = true;
//        }
//        this->fault_sent.at("A1") = false; //refresh the flag
//        game_state = state->data; //change the state
    }

    virtual void CheckForFault(const typename audio_common_msgs::AudioData::ConstPtr& data_msg){
        for (int i = 0; i < data_msg->data.size(); i++)
            std::cout << data_msg->data[i] << " ";
        std::cout << std::endl;
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "audio");
    FaultMonitorAudio monitor("audio", faultList, stateList);

    ros::spin();

    return 0;
}
