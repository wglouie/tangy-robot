#include <fault_diagnosis/MonitorTemplate.h>
#include <vector>
#include <cmath>

static std::map<std::string, std::string[FAULT_LIST_LEN]> faultList =
        {{"B2", {"Near zero depth", "1", "1"}}};

class FaultMonitorDepth : public FaultMonitorImg, public TimedRefresh{
public:
    FaultMonitorDepth(std::string watched_topic, std::map<std::string, std::string[FAULT_LIST_LEN]>& faultList)
            : FaultMonitorImg(watched_topic, faultList), TimedRefresh(10.0, n){
    }

protected:
    int THRESHOLD = 65;

    virtual void CheckForFaultImg(const typename sensor_msgs::Image::ConstPtr& data_msg){
        std::cout << "Checking" << std::endl;
        bool fault_found = false;
        int length = data_msg->data.size();
        int step = 500;
        double average = 0;
        std::vector<int> stuff;
        std::vector<double> differences;
        for(int i = 0; i < length; i += step){
            stuff.push_back(data_msg->data[i]);
        }
        std::cout << "Got " << stuff.size() << " pixels" << std::endl;
        //check difference between every pixel
        for(int i = 0; i < stuff.size(); i++){
            for(int j = 0; j < stuff.size(); j++){
                if(i != j){
                    differences.push_back(stuff[i] - stuff[j]);
                }
            }
        }
        //if a majority of pixels are dark report an error
        for(int i = 0; i < differences.size(); i++){
            average += std::abs(differences[i]/differences.size());
        }
        std::cout << "Average: " << average << std:: endl;
        if(average < THRESHOLD && !fault_sent.at("B2"))
            LogFault("B2");//TODO add refresh
    }

    virtual void Refresh(const ros::TimerEvent& event) {
        this->fault_sent.at("B2") = false;
        std::cout << "Refreshed" << std::endl;
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "fault_monitor_depth");
    FaultMonitorDepth monitor("bottom_camera/depth/image", faultList);

    ros::spin();

    return 0;
}
