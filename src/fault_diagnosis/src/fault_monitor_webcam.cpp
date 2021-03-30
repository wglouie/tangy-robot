#include <fault_diagnosis/MonitorTemplate.h>
#include <vector>
#include <cmath>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

static std::map<std::string, std::string[FAULT_LIST_LEN]> faultList =
        {{"E1", {"Camera impeded", "1", "1"}},
         {"E2", {"Camera partially impeded", "1", "1"}},
         {"E3", {"Camera vision blocked by environmental conditions", "1", "0"}}};

static std::vector<std::string> stateList = {"Checking card"};

class FaultMonitorWebcam : public FaultMonitorImg, public TimedRefresh{
public:
    FaultMonitorWebcam(std::string watched_topic, std::map<std::string, std::string[FAULT_LIST_LEN]>& faultList,
            std::vector<std::string> stateList) : FaultMonitorImg(watched_topic, faultList, stateList), TimedRefresh(10.0, n){
    }

protected:
    int THRESHOLD = 20;

    virtual void CheckForFaultImg(const typename sensor_msgs::Image::ConstPtr& data_msg){
        /*
        //get monochrome image and downsize
        try{
        //    cv::Mat img = cv_bridge::toCvCopy(msg, "mono8")->image;
        } catch (cv_bridge::Exception error) {
            ROS_ERROR("Error converting image message");
            return;
        }
        //cv::Mat dst = cv::Mat::zeros(180, 320, img.type());
        resize(img, dst, dst.size(), 0, 0, cv::INTER_AREA);
        //run image through trained network and classify

        //log fault if abnormal and fault not already sent



        for(int i = 0; i < differences.size(); i++){
            average += std::abs(differences[i]/differences.size());
        }
        std::cout << "Average: " << average << std:: endl;
        if(average < THRESHOLD && !fault_sent.at("E1"))
            LogFault("E1");//TODO add refresh
            */
    }

    virtual void Refresh(const ros::TimerEvent& event) {
        this->fault_sent.at("E1") = false;
        this->fault_sent.at("E2") = false;
        this->fault_sent.at("E3") = false;
        std::cout << "Refreshed" << std::endl;
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "fault_monitor_webcam");
    FaultMonitorWebcam monitor("camera/image_raw", faultList, stateList);

    ros::spin();

    return 0;
}
