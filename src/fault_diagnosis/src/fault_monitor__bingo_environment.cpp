#include <fault_diagnosis/MonitorTemplate.h>
#include <std_msgs/Empty.h>

static std::map<std::string, std::string[FAULT_LIST_LEN]> faultList =
        {{"S4", {"Can't see card", "1", "0"}}};

static std::vector<std::string> stateList = {"test"};

class FaultMonitorEnvironment : public FaultMonitor<std_msgs::Empty>, public TimedRefresh{
public:
    FaultMonitorEnvironment(std::string watched_topic, std::map<std::string, std::string[FAULT_LIST_LEN]>& faultList,
            std::vector<std::string> stateList) : FaultMonitor<std_msgs::Empty>(watched_topic, faultList, stateList), TimedRefresh(10.0, n){
    }

protected:
    virtual void CheckForFault(const typename std_msgs::Empty::ConstPtr& data_msg){
        if(!fault_sent.at("S4")) {
            LogFault("S4"); //for the moment the card occlusion is the only fault
        }
    }

    virtual void Refresh(const ros::TimerEvent& event){
        this->fault_sent.at("S4") = false;
        std::cout << "Refreshed" << std::endl;
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "fault_monitor_environment");
    FaultMonitorEnvironment monitor("environment_faults", faultList, stateList);

    ros::spin();

    return 0;
}
