//============================================================================
// Name        : face_tracker.h
// Author      : Jacob Li
// Version     : 1.0
// Copyright   :
// Description : Helper class for text to speech; play any mp3 audio file by using the function play_file(string filePath)
//============================================================================

#ifndef face_tracker_H_
#define face_tracker_H_
#include <ros/ros.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <math.h>
#include <std_msgs/String.h>
#include "neck_handler.h"
#include <FaceDetectionClient.h>

class faceTracker {
  
  ros::NodeHandle nh_;
  neck_handler neckHandler;
  std::vector<int> previous_face_location;
  
/////////////////////////////////////////////////
  bool stoptheneck;
  int pickcenterperson;
  std::string centerPerson;
  int firsttime;
  bool findANewPOI;
  bool tracking_permission;
  std::string POIname;
  int timeofpreviousPOIpositionupdate;
  int prevPOIx;
  int prevPOIy;
//////////////////////////////////////////////////

  	/*Action library variables*/

  public:
  	FaceDetectionClient faceDetectionClient;
    bool already_tracking;
    //bool stop_tracking;
    faceTracker();
    ~faceTracker();
    
    void setPOInameCb(const std_msgs::String::ConstPtr& str);
    void trackFaceCB(const std_msgs::String::ConstPtr& str);
    void stopTrackCB(const std_msgs::String::ConstPtr& str);
    void pickAPersonCB(const std_msgs::String::ConstPtr& str);

///////////////////////////////////////////////////////////////////////////////////////////
    void stopNeckCB(const std_msgs::String::ConstPtr& str);
///////////////////////////////////////////////////////////////////////////////////////////

};
#endif
