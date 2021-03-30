#include <fault_diagnosis/MonitorTemplate.h>
#include <keyboard/Key.h>
#include <fault_diagnosis/fault_return.h>

static std::map<std::string, std::string[FAULT_LIST_LEN]> faultList =
        {{"A1", {"Pressed: ", "1", "1"}}};

static std::vector<std::string> stateList = {};

class TestMonitorKeyboard : public FaultMonitor<keyboard::Key>, public TimedRefresh{
public:
    TestMonitorKeyboard(std::string watched_topic, std::map<std::string, std::string[FAULT_LIST_LEN]>& faultList,
            std::vector<std::string> stateList) : FaultMonitor<keyboard::Key>(watched_topic, faultList, stateList), TimedRefresh(10.0, n){
        client = n.serviceClient<fault_diagnosis::fault_return>("fault_return");
    }

protected:
    ros::ServiceClient client;
    fault_diagnosis::fault_return srv;

    virtual void CheckForFault(const typename keyboard::Key::ConstPtr& data_msg){
        if(data_msg->modifiers == 1 || data_msg->modifiers == 2){
            if (!fault_sent.at("A1")){
                LogFault("A1", std::to_string(data_msg->code));
            }
        }
        else if(data_msg->code == 97){ //this is the a key
            if(client.call(srv)){
                std::cout << srv.response.fault.error_code << " " << srv.response.fault.fault_msg << std::endl;
            }
            else{
                std::cout << "No fault or error" << std::endl;
            }
        }
    }

    virtual void Refresh(const ros::TimerEvent& event){
        this->fault_sent.at("A1") = false;
        std::cout << "Refreshed" << std::endl;
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "test_monitor_keyboard");
    TestMonitorKeyboard monitor("keyboard/keydown", faultList, stateList);

    ros::spin();

    return 0;
}