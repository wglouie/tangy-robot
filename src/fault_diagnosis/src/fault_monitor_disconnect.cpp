#include <fault_diagnosis/MonitorTemplate.h>
#include <std_msgs/Empty.h>
#include <stdlib.h>
#include <cstdio>
#include <memory>

static std::map<std::string, std::string[FAULT_LIST_LEN]> faultList =
        {{"A1", {"Device disconnnect: ", "1", "0"}}};

static std::vector<std::string> stateList = {};

class FaultMonitorDisconnect : public FaultMonitor<std_msgs::Empty>, public TimedRefresh{
public:
    FaultMonitorDisconnect(std::string watched_topic, std::map<std::string, std::string[FAULT_LIST_LEN]>& faultList,
            std::vector<std::string> stateList) : FaultMonitor<std_msgs::Empty>(watched_topic, faultList, stateList), TimedRefresh(10.0, n){
        check_timer  = n.createTimer(ros::Duration(10.0), &FaultMonitorDisconnect::CheckForFault, this);
    }

protected:
    ros::Timer check_timer;

    virtual void CheckForFault(const typename std_msgs::Empty::ConstPtr& data_msg){}//dummy

    void CheckForFault(const ros::TimerEvent& event){
        if(!std::system(NULL)){return;} //if there is no shell, return (required?)
        if(exec("ping -c 1 192.168.0.97 | grep \"0 received\"") != ""){//look for the axis camera TODO this command takes a long time, maybe there is a better way?
            LogFault("A1", "AXIS camera"); //there is an error that runs when the camera disconnects, if we can find it we can use it instead maybe.
        }
        if(exec("lsusb | grep 046d:082d") == ""){//look for the webcam
            LogFault("A1", "Logitech webcam");
        }
        //TODO add the timeout to fault_sent
    }

    virtual void Refresh(const ros::TimerEvent& event){
        this->fault_sent.at("A1") = false;
        std::cout << "Refreshed" << std::endl;
    }

    std::string exec(const char* cmd) {
        std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
        if (!pipe) return "ERROR";
        char buffer[128];
        std::string result = "";
        while (!feof(pipe.get())) {
            if (fgets(buffer, 128, pipe.get()) != NULL)
                result += buffer;
        }
        return result;
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "fault_monitor_disconnect");
    FaultMonitorDisconnect monitor("dummy", faultList, stateList);

    ros::spin();

    return 0;
}