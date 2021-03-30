#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <keyboard/Key.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


class ImageSaver{
public:
    ImageSaver(int dummy) : it(n){
        count = 0;
        key = n.subscribe("/keyboard/keydown", 1, &ImageSaver::Save, this);
        imgsub = it.subscribe("/camera/camera/image_raw", 1, &ImageSaver::ImgCb, this);
        myfile.open("/home/tangy/classification.txt"); //TODO should have a try
    }

private:
    ros::NodeHandle n;
    cv_bridge::CvImagePtr latestimg;
    ros::Subscriber key;
    std::ofstream myfile;
    image_transport::ImageTransport it;
    image_transport::Subscriber imgsub;
    int count;

    void Save(const keyboard::Key::ConstPtr& msg){
        if(msg->code == 48 || msg->code == 49){
            std::string filename = std::to_string(count) + ".png";
            try {
                cv::Mat temp = latestimg->image;
                //cv::Mat dst = cv::Mat::zeros(180, 320, temp.type());
                //cv::resize(temp, dst, dst.size(), 0, 0, cv::INTER_AREA);
                cv::imwrite(filename, temp);
                myfile << filename << (msg->code == 48 ? " 0" : " 1") << std::endl;
                std::cout << "Saved image " << filename << std::endl;
                count++;
            } catch (cv_bridge::Exception error) {
                ROS_ERROR("error converting image message");
            }
        }
    }

    void ImgCb(const sensor_msgs::Image::ConstPtr& msg){
        std::cout << "." << std::endl;
        latestimg = cv_bridge::toCvCopy(msg, "mono8");
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "image_saver");
    ImageSaver saver(0);

    ros::spin();
    
    return 0;
}
