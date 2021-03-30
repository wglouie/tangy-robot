#include <BingoGameFullServer/BingoGameFullServer.h>
#include <iostream>
std::string BingoGameFullServer::detect_face() {
	ROS_INFO("Trying to detect any faces...");
//	faceDetectionClient.sendAndWait();

	faceDetectionClient.send(10);
	double begin=ros::Time::now().toSec();
	bool timeout=false;

	while(!faceDetectionClient.goal_completed()&& timeout!=true) {
		double end=ros::Time::now().toSec();
		if(end>(begin+3)){
		  timeout=true;
		}
	}


    //the line below no longer makes sense
    //std::string name=faceDetectionClient.person_of_interest();
    //instead i am doing the line below which in this current implementation will always return the name "POI"
    std::string name=faceDetectionClient.get_POIname();
	ROS_INFO("Face detection client returning: [%s]",name.c_str());
	if(name.compare("No Users")==0||name.compare("Unknown")==0) {
        return "";
	} else {
		return name;
	}

}

///////////////////////////////////////////////
void BingoGameFullServer::look_at_face(){
	ROS_INFO("Beginning tracking face!");
	ros::Rate loop_rate(1);
	std_msgs::String msg;
	std::stringstream ss;
        ss << "start";
	msg.data = ss.str();
	start_track_face_pub.publish(msg);
        ros::spinOnce();

/*
	while (ros::ok())
	{
		start_track_face_pub.publish(msg);			
		ros::spinOnce();
		loop_rate.sleep();
	}
  
    std_msgs::String msg;
    msg.data="start";
    start_track_face_pub.publish(msg);
    sleep(1);
    start_track_face_pub.publish(msg);
*/
}
///////////////////////////////////////////////

//////////////////////////////////////////////
//the changePOI function is meant to be used in the "look_at_face" function based on what ever trigger you use, a timer.....
//the publisher is defined in Bingogamefullserver.h and the advertising of the publisher is done int the Bingogamefullserver.cpp
void BingoGameFullServer::changePOI()
{
    ROS_INFO("Intentionally changing POI!");
    std_msgs::String msg;
    std::stringstream ss;
    ss << "change";
    msg.data = ss.str();
    pick_a_person_pub.publish(msg);
    ros::spinOnce();
}
//////////////////////////////////////////////

void BingoGameFullServer::stopNeckMovement(bool stopneckmove)
{
  string neckmove;
  if(stopneckmove)
	neckmove="trueess";
  if(!stopneckmove)
	neckmove="falseee";

  ROS_INFO("stop_neck_movement");
  std_msgs::String msg;
  std::stringstream ss(neckmove);
  msg.data = ss.str();
  stop_neck_movement.publish(msg);
  ros::spinOnce();  
}

void BingoGameFullServer::stop_look_at_face(){
  ROS_INFO("Stopping tracking face!");
  std_msgs::String msg;
  msg.data="stop";
  stop_track_face_pub.publish(msg);
  sleep(1);
  stop_track_face_pub.publish(msg);
  stop_track_face_pub.publish(msg);
}
