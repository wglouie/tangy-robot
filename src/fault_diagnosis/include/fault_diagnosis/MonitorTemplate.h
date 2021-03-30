/*This node listens on a communication channel and detects anomalous behaviour
 *When something is found, it sends a message to the core node via the general
 *fault publisher with the information in Fault.msg
 *Defined in header because http://stackoverflow.com/questions/495021/why-can-templates-only-be-implemented-in-the-header-file
 *see same link for other options
 */

#ifndef FAULT_MONITOR
#define FAULT_MONITOR

#include <fault_diagnosis/FaultInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

template <class MsgType> //using variation of Curiously Recurring Template Pattern http://stackoverflow.com/questions/7181361/enforcing-correct-parameter-types-in-derived-virtual-function
class FaultMonitor{
public:
    FaultMonitor(std::string watched_topic, std::map<std::string, std::string[FAULT_LIST_LEN]>& faultList,
            std::vector<std::string> states) : faults(faultList), activeStates(states), topic(watched_topic){
        std::cout << "Setting up " << ros::this_node::getName() << std::endl;
        fault_pub = n.advertise<fault_diagnosis::Fault>("faults", 10);
        gssub = n.subscribe("game_state", 1, &FaultMonitor::GSUpdate, this);
        for(auto x : faults) {//initialize the sent flags to the same set of keys and set them all to false
            fault_sent.emplace(x.first, false);
            //std::cout << "Copy " << x.first << " <> Num: " << fault_sent.count(x.first) << " Val: " << fault_sent.at(x.first) << std::endl;
        }
    }

protected:
    ros::NodeHandle n;
    ros::Publisher fault_pub;
    ros::Subscriber sub, gssub;
    std::map<std::string, std::string[FAULT_LIST_LEN]>& faults;
    std::map<std::string, bool> fault_sent;
    std::vector<std::string> activeStates;
    std::string game_state, topic;

    virtual void CheckForFault(const typename MsgType::ConstPtr& data_msg){
        //check the incoming data for faults
        //add the timeout to fault_sent
        //if(fault_found && !fault_sent.at(fault code))
        //    LogFault(fault code);
    }

    void LogFault(std::string fault){ //Takes the fault error, checks the table for the correct information, and publishes it
        fault_diagnosis::Fault msg = AssembleFault(fault);
        fault_pub.publish(msg);
        fault_sent.at(fault) = true;
        //std::cout << "Fault sent." << std::endl;
    }

    void LogFault(std::string fault, std::string additional){ //As LogFault, with additional info in the message
        fault_diagnosis::Fault msg = AssembleFault(fault);
        msg.fault_msg = faults.at(fault)[0] + additional;
        fault_pub.publish(msg);
        fault_sent.at(fault) = true;
        //std::cout << "Fault sent." << std::endl;
    }

    fault_diagnosis::Fault AssembleFault(std::string fault){
        fault_diagnosis::Fault msg;
        std::cout << "Fault found, code " << fault << ", sending to core." << std::endl;
        msg.monitor_name = ros::this_node::getName();
        msg.behaviour = game_state;
        msg.error_code = fault;
        msg.fault_msg = faults.at(fault)[0];
        msg.criticality = std::stoi(faults.at(fault)[1]);
        msg.technicality = std::stoi(faults.at(fault)[2]);
        return msg;
    }

    virtual void GSUpdate(const std_msgs::String::ConstPtr& state){
        game_state = state->data;
        //if the list is empty or the current game state is in the list, activate the subscriber
        if(activeStates.empty() || std::find(activeStates.begin(), activeStates.end(), game_state) != activeStates.end()){
            sub = n.subscribe(topic, 10, &FaultMonitor::CheckForFault, this);
        }
        else {
            sub.shutdown(); //else turn off the subscriber for the moment
        }
    }
};

class FaultMonitorImg : public FaultMonitor<sensor_msgs::Image>{
public:
    FaultMonitorImg(std::string watched_topic, std::map<std::string, std::string[FAULT_LIST_LEN]>& faultList,
            std::vector<std::string> states) : FaultMonitor<sensor_msgs::Image>(watched_topic, faultList, states), it(n){
        imgsub = it.subscribe(watched_topic, 1, &FaultMonitorImg::CheckForFaultImg, this);
    }

protected:
    image_transport::ImageTransport it;
    image_transport::Subscriber imgsub;

    virtual void CheckForFault(const typename sensor_msgs::Image::ConstPtr& data_msg){} //dummy for superclass

    virtual void CheckForFaultImg(const typename sensor_msgs::Image::ConstPtr& data_msg){} //Different callback function due to image_transport subscriber

    virtual void GSUpdate(const std_msgs::String::ConstPtr& state){
        game_state = state->data;
        //if the list is empty or the current game state is in the list, activate the subscriber
        if(activeStates.empty() || std::find(activeStates.begin(), activeStates.end(), game_state) != activeStates.end()){
            imgsub = it.subscribe(topic, 1, &FaultMonitorImg::CheckForFaultImg, this);
        }
        else {
            imgsub.shutdown(); //else shut down the subscriber for the moment
        }
    }
};

class TimedRefresh{ //Temporary refresh class until the behaviours are added in
public:
    TimedRefresh(double duration, ros::NodeHandle n){
        timer = n.createTimer(ros::Duration(duration), &TimedRefresh::Refresh, this);
    }

private:
    ros::Timer timer;
    virtual void Refresh(const ros::TimerEvent& event) = 0;
};

/* Example main file, faultList should be defined at the top of the file
int main(int argc, char **argv) {
    ros::init(argc, argv, "fault_monitor_template");
    FaultMonitor monitor<std::string>("temp", faultList); //insert watched topic name

    ros::spin();

    return 0;
}
*/
//TODO see about importing a templated main function here
#endif //FAULT_MONITOR