#include <fault_diagnosis/MonitorTemplate.h>
#include <actionlib_msgs/GoalStatusArray.h>

static std::map<std::string, std::string[FAULT_LIST_LEN]> faultList =
        {{"S2", {"Navigation goal aborted", "1", "0"}},
         {"S3", {"Navigation goal rejected", "1", "0"}}};

static std::vector<std::string> stateList = {"Moving"};

class FaultMonitorNavigation : public FaultMonitor<actionlib_msgs::GoalStatusArray>, public TimedRefresh{
public:
    FaultMonitorNavigation(std::string watched_topic, std::map<std::string, std::string[FAULT_LIST_LEN]>& faultList,
            std::vector<std::string> stateList) : FaultMonitor<actionlib_msgs::GoalStatusArray>(watched_topic, faultList, stateList), TimedRefresh(10.0, n){
    }

protected:
    virtual void CheckForFault(const typename actionlib_msgs::GoalStatusArray::ConstPtr& data_msg){
        if(data_msg->status_list.size() < 1){return;}
        int firstGoalStatus = data_msg->status_list[0].status;
        if(firstGoalStatus == 4 && !fault_sent.at("S2")){
            LogFault("S2");
        }
        else if (firstGoalStatus == 5 && !fault_sent.at("S3")){
            LogFault("S3");
        }
        //TODO add the timeout to fault_sent
    }

    virtual void Refresh(const ros::TimerEvent& event){
        this->fault_sent.at("S2") = false;
        this->fault_sent.at("S3") = false;
        std::cout << "Refreshed" << std::endl;
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "fault_monitor_navigation");
    FaultMonitorNavigation monitor("move_base/status", faultList, stateList);

    ros::spin();

    return 0;
}
