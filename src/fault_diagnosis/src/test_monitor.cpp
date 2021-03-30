#include <fault_diagnosis/MonitorTemplate.h>
#include <drrobot_h20_player/StandardSensor.h>

class TestMonitor{
public:
    TestMonitor(std::string watched_topic){
	sub = n.subscribe(watched_topic, 10, &TestMonitor::CheckForFault, this); 
	}

protected:
ros::Subscriber sub;
ros::NodeHandle n;

    virtual void CheckForFault(const drrobot_h20_player::StandardSensor::ConstPtr& data_msg){
	for(int i = 0; i < data_msg->humanSensorData.size(); i++){
		std::cout << data_msg->humanSensorData[i] << "\t\t";
	}
	std::cout << std::endl;
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "test_monitor");
    TestMonitor monitor("drrobot_standardsensor");

    ros::spin();

    return 0;
}
