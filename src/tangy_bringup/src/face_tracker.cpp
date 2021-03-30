#include "face_tracker.h"


faceTracker::faceTracker():faceDetectionClient("FT_face_detection_client"), neckHandler("FT_neck_handler"){
  //stop_tracking=false;
  already_tracking=false;
  pickcenterperson=0;
  firsttime=0;
  findANewPOI=false;
  tracking_permission=false;
  timeofpreviousPOIpositionupdate=ros::Time::now().toSec()-1;
  stoptheneck=false;
}

faceTracker::~faceTracker(){
  ROS_INFO("\n\n tracker destroyed. \n\n");
}

void faceTracker::setPOInameCb(const std_msgs::String::ConstPtr& str)
{
    POIname=str->data;
}

void faceTracker::trackFaceCB(const std_msgs::String::ConstPtr& str)
{
  std::vector<int> pos_of_face;
  int pos_of_face_center_base [2];

  //initialize the POIname to No Users so that the person of interest loop is executed intitially
  tracking_permission=true;
  stoptheneck=false;
  if(already_tracking==false)
  {
      already_tracking=true;
      //stop_tracking=false;
      faceDetectionClient.send(10);
      double begin=ros::Time::now().toSec();
      bool timeout=false;
      while(timeout!=true)
      {
          double end=ros::Time::now().toSec();
          if(end>(begin+1))
          {
               timeout=true;
          }
      }
      //the outer while loop (not do while loop!!!!) always occurs until the breaking signal is given vial topic. in every outer loop a new POI is chosen.
      //the iterations of the inner while loop (not do while!!!) update the POI x and y coordinates until the signal is given via topic to choose a new POI where the inner loop is left
      //into the outer loop again where a new poi is chosen.
      while(true && tracking_permission)
      {
	  //loop until a personofinterest is found. this occurs when there is no one on the screen.
          //you must wait for a person to appear in an empty screen where that person becomes the POI.
          do
          {
	      //ROS_INFO(" \n yooo %s \n",POIname.c_str());
              //get the person of interest and store in the server. it can also return the name of that person
              POIname = faceDetectionClient.person_of_interest_client();
	      //ROS_INFO("after the poiclient func in face tracker");
          }while(POIname.compare("No Users") == 0);

          //loop until told to stop
          while(tracking_permission)
          {
	      //what is this for???????? coment it if it does not work
              //faceDetectionClient.send(2);

              //give back the person of interest x and y position
              pos_of_face = faceDetectionClient.get_person_of_interest_position_follow_client();

              //trigger to find new POI causes the loop to be left.
              //(can be done by the get person of interest realising your POI has left, or by the function stop look at face).
              if(findANewPOI)
              {
                  //reset the new POI trigger to false
                  findANewPOI=false;
                  break;
              }

              //based on the pos_of_face you calculate the increment needed wont increment if the current position is the same as the previous position
  	      pos_of_face_center_base[0] = 320-pos_of_face[0];
  	      pos_of_face_center_base[1] = 240-pos_of_face[1];

	      pos_of_face_center_base[0] -= 10;
	      //pos_of_face_center_base[1] -= 10;

          //bool closex=false;
          //bool closey=false;
	      bool close=false;

          close=sqrt((prevPOIx-pos_of_face_center_base[0])*(prevPOIx-pos_of_face_center_base[0])+(prevPOIy-pos_of_face_center_base[1])*(prevPOIy-pos_of_face_center_base[1]))<4;
          
          if((ros::Time::now().toSec()>timeofpreviousPOIpositionupdate+0.9) && !close)
          {
              timeofpreviousPOIpositionupdate=ros::Time::now().toSec();
              if((pos_of_face_center_base[0]>2 || pos_of_face_center_base[0]<-2) && (pos_of_face_center_base[1]>2 || pos_of_face_center_base[1]<-2))
              {
                if(POIname.compare("No Users")!=0){
		    if(!stoptheneck){
                    //ROS_INFO("LEFT RIGHT INCREMENT: %d up down increment %d",left_right,up_down);
                    neckHandler.increment_dir(-pos_of_face_center_base[0],pos_of_face_center_base[1]);
		    }
		    //else if(stoptheneck)
                    //neckHandler.increment_dir(0,0);
                }
                else
                     break;
              }
          }
          prevPOIx=pos_of_face_center_base[0];
          prevPOIy=pos_of_face_center_base[1];

              //timeofpreviousPOIpositionupdate=ros::Time::now().toSec();


              //one in the screen would be anyone that appears first.
          }
      }
      already_tracking=false;
  }
}

void faceTracker::pickAPersonCB(const std_msgs::String::ConstPtr& str)
{
    findANewPOI=true;
    ROS_INFO("Picking a new POI!");
}

void faceTracker::stopTrackCB(const std_msgs::String::ConstPtr& str)
{
  tracking_permission=false;
  stoptheneck=true;
  ROS_INFO("Stopping tracking!");
}

///////////////////////////////////////////////////////////////////////////////////////
void faceTracker::stopNeckCB(const std_msgs::String::ConstPtr& str)
{
  if((str->data).compare("trueess")==0)
  {
     stoptheneck=true;
  }
  else if((str->data).compare("falseee")==0)
  {
     stoptheneck=false;
  }
}
/////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv)
{
	ros::init(argc,argv,"face_tracker");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(0);
	spinner.start();
	faceTracker tracker;
	//This node gets its instructions to start or stop by subscribing to two topics:
	// -'start_track_face'
	// -'stop_track_face'
	ros::Subscriber start_sub=n.subscribe<std_msgs::String>("start_track_face",100, &faceTracker::trackFaceCB, &tracker);
   	ros::Subscriber stop_sub=n.subscribe<std_msgs::String>("stop_track_face",100, &faceTracker::stopTrackCB, &tracker);
        ros::Subscriber pick_a_person_sub=n.subscribe<std_msgs::String>("pick_a_person",1, &faceTracker::pickAPersonCB, &tracker);
        
///////////////////////////////////////////////////////////////////////////////////////////
        ros::Subscriber stop_neck_movement=n.subscribe<std_msgs::String>("stop_neck_movement",1,&faceTracker::stopNeckCB, &tracker);
///////////////////////////////////////////////////////////////////////////////////////////
	ros::waitForShutdown();
 	return 0;
}
