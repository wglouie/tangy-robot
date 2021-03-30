#include <fault_diagnosis/FaultInfo.h>
#include <fault_diagnosis/print_faults.h>

class FaultDiagnosisPrintRequester{
public:
    FaultDiagnosisPrintRequester(){
        client = n.serviceClient<fault_diagnosis::print_faults>("print_faults");
    }

    void PrintFaults(int no){
        srv.request.no = no;
        if(!client.call(srv)){
            std::cout << "There was an error printing the faults" << std::endl;
        }
    }

private:
    ros::NodeHandle n;
    ros::ServiceClient client;
    fault_diagnosis::print_faults srv;
};


int main(int argc, char **argv){
    ros::init(argc, argv, "fault_diagnosis_print_requester");
    FaultDiagnosisPrintRequester node;
    int no = 0;

    while(ros::ok()) {
        std::cout << "Input number of faults to print" << std::endl;
        std::cin >> no;
        node.PrintFaults(no);
        ros::spinOnce();
    }

    return 0;
}