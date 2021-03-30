/*Fault diagnosis module for Tangy robot
 *Collects the possible faults from the monitors, and keeps them in a list
 *when a fault is detected, searches through the collection to find the most
 *likely candidate fault and sends it out.
 */

#include <fault_diagnosis/FaultInfo.h>
#include "fault_diagnosis/fault_return.h"
#include "fault_diagnosis/print_faults.h"
#include <vector>

class FaultDiagnosisCore{
public:
    FaultDiagnosisCore(){
        faultsub = n.subscribe("faults", 100, &FaultDiagnosisCore::logFault, this);
        gssub = n.subscribe("game_state", 10, &FaultDiagnosisCore::updateGS, this);
        faultreturnsrv = n.advertiseService("fault_return", &FaultDiagnosisCore::FaultReturn, this);
        printsrv = n.advertiseService("print_faults", &FaultDiagnosisCore::PrintFaults, this);
        list.reserve(10); //reserve room for 10 errors initially
    }

    void logFault(const fault_diagnosis::Fault::ConstPtr& msg){ //When a message comes in on the topic, add it to the list
        //TODO If this is a proactive fault, send it out instead
        list.push_back(CopyFault(msg));
        std::cout << "Received fault " << msg->error_code << " from " << msg->monitor_name << std::endl;
        //There should be a timeout on each publisher to keep from continual reporting of the same error
    }

    void updateGS(const std_msgs::String::ConstPtr& msg){
        game_state = msg->data;
        //TODO turn monitors on and off according to some config file
    }

    bool FaultReturn(fault_diagnosis::fault_return::Request &req, fault_diagnosis::fault_return::Response &res){ //When a service call is made, check the list according to algorithm and return the proper fault
        if(list.size() < 1){ //If there are no errors we return the default learning fault
            res.fault = CreateFault("S1", "Learning Error, the robot needs a new demonstration of the activity", 1, 1);
            return true;
        }
        //TODO search the list in a behaviour dependent manner to find the most likely fault
        //int index = 0; //temp, for now returns last member
        res.fault = list.back();
        return true;
    }

    bool PrintFaults(fault_diagnosis::print_faults::Request &req, fault_diagnosis::print_faults::Response &res){
        int last = list.size() - 1;
        for(int i = last; i > std::max(last - req.no, -1); i--){
            std::cout << list[i].header.stamp << " - " << list[i].error_code << ": " << list[i].fault_msg << std::endl;
        }
        return true;
    }

private:
    ros::NodeHandle n;
    ros::Subscriber faultsub, gssub;
    ros::ServiceServer printsrv, faultreturnsrv;
    std::vector<fault_diagnosis::Fault> list;
    std::string game_state;
    //TODO add default fault

    fault_diagnosis::Fault CopyFault(const fault_diagnosis::Fault::ConstPtr& msg){
        fault_diagnosis::Fault data = *msg;
        data.header.stamp = ros::Time::now();
        //data.monitor_name = msg->monitor_name;
        //data.behaviour = msg->behaviour;
        //data.error_code = msg->error_code;
        //data.fault_msg = msg->fault_msg;
        //data.criticality = msg->criticality;
        //data.technicality = msg->technicality;
        return data;
    }

    fault_diagnosis::Fault CreateFault(std::string code, std::string message, int criticality, int technicality){
        fault_diagnosis::Fault fault;
        fault.header.stamp = ros::Time::now();
        fault.monitor_name = ros::this_node::getName();
        fault.behaviour = "";
        fault.error_code = code;
        fault.fault_msg = message;
        fault.criticality = criticality;
        fault.technicality = technicality;
        return fault;
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "fault_diagnosis_core");
    FaultDiagnosisCore diagnoser;

    ros::spin();

    return 0;
}

