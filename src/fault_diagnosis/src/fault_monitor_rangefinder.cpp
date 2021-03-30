#include <fault_diagnosis/MonitorTemplate.h>
#include <diagnostic_msgs/DiagnosticArray.h>

static std::map<std::string, std::string[FAULT_LIST_LEN]> faultList =
        {{"E1", {"Rangfinder fault. Message: ", "1", "1"}}};

static std::vector<std::string> stateList = {};

class FaultMonitorRangefinder : public FaultMonitor<diagnostic_msgs::DiagnosticArray>, public TimedRefresh{
public:
    FaultMonitorRangefinder(std::string watched_topic, std::map<std::string, std::string[FAULT_LIST_LEN]>& faultList,
            std::vector<std::string> stateList) : FaultMonitor<diagnostic_msgs::DiagnosticArray>(watched_topic, faultList, stateList), TimedRefresh(10.0, n){
    }

protected:
    virtual void CheckForFault(const typename diagnostic_msgs::DiagnosticArray::ConstPtr& data_msg){
        diagnostic_msgs::DiagnosticStatus msg = data_msg->status.front();
        if(msg.level != 0 && !fault_sent.at("E1")) { //this doesn't check for hokuyo but we probably don't need it since it is the only one on diagnostics
            LogFault("E1", msg.message); //there is only one fault type for the hokuyo, but we add the message on
        }
        //TODO add the timeout to fault_sent
    }

    virtual void Refresh(const ros::TimerEvent& event){
        this->fault_sent.at("E1") = false;
        std::cout << "Refreshed" << std::endl;
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "fault_monitor_rangefinder");
    FaultMonitorRangefinder monitor("diagnostics", faultList, stateList); //ir_data_pcl = topic with range data in PointCloud2 message format

    ros::spin();

    return 0;
}
