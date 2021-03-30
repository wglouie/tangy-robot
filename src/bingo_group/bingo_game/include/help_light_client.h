////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  Frank Despond

  ---How to---

  start help light with start();
  then
  query is_help_required() method

    is_help_required() will return 0 for nothing, 1 for help, and -1 for error;

    when is_help_required() == 1 query getHelpLocations() to get the help locations
    Triangle Locations are in a struct


*/
////////////////////////////////////////////////////////////////////////////////////////////////////////



#include <unistd.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <help_indicators/helpLightAction.h>
#include <vector>
#include <cmath>
#include <fstream>
#include <drrobot_h20_player/HeadCmd.h>
#define PI 3.14159265


using namespace std;

class HelpLightClient{

public:

    //Client's attributes
    actionlib::SimpleActionClient<help_indicators::helpLightAction> ac;
    //states
    bool done;
    bool running;
    bool ready;

    ros::NodeHandle nh_;
    ros::Publisher pub_head;


    enum state{nothing=0, help=1, error =-1};
    state helpState;


    /*Important*/

    struct Triangle {
        double locationX;
        double locationY;
        double locationZ;
    };

    vector< Triangle > resultTriangles;


    /**
     * ConstructorS
     * @param name
     */
    HelpLightClient(std::string name):
        //Set up the client. It's publishing to topic "test_action", and is set to auto-spin
        ac("help_light", true),
        //Stores the name
        action_name_(name)
    {

        //Get connection to a server
        ROS_INFO("[%s] Waiting For Server...", action_name_.c_str());
        //Wait for the connection to be valid
        ac.waitForServer();
        ROS_INFO("[%s] Got a Server...", action_name_.c_str());
        done = false;
        running = false;
        ready = true;
        helpState =nothing;
        pub_head= nh_.advertise<drrobot_h20_player::HeadCmd>("/cmd_pose_head", 1);
    }

    /**
     * Destructor
     */
    ~HelpLightClient(){
        //ac.cancelAllGoals();
    }



    /**
     * Called once when the goal completes
     * @param state
     * @param result
     */
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const help_indicators::helpLightResultConstPtr& result)
    {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());

        done = true;
        stop();
        if (result->numberOfLights >= 1)
        {
            helpState = help;
            for(int i =0; i < result->numberOfLights; i++)
            {
                resultTriangles.push_back(Triangle());
                //orderedGoodTriangles goes x,y,z,numbertimes seen in order of apperance
                resultTriangles[i].locationX = result->lightLocationX[i];//locaton x
                resultTriangles[i].locationY =  result->lightLocationY[i];//location y
                resultTriangles[i].locationZ = result->lightLocationZ[i];//location z
            }
            acknowledge_help();
            stop();

        }
        else if (result->numberOfLights == 0)
        {
            helpState = nothing;
            //if this doesnt find anything restart the goal and keep looking....
            start();
        }
        else if (result->numberOfLights == -1) helpState = error;

    }


    /**
     * Called once when the goal becomes active
     */
    void activeCb()
    {
        ROS_INFO("[%s] Goal just went active...", action_name_.c_str());

        done = false;
        running = true;
        ready = false;
    }


    /**
     * Called every time feedback is received for the goal
     * @param feedback
     */
    void feedbackCb(const help_indicators::helpLightFeedbackConstPtr& feedback)
    {
    }


    /**
     * Send a goal to the server
     * @param newGoal
     */
    void sendGoal()
    {
        if(ready){
          help_indicators::helpLightGoal newGoal;
          newGoal.start = 1;
          //Once again, have to used boost::bind because you are inside a class
          ac.sendGoal(newGoal, boost::bind(&HelpLightClient::doneCb, this, _1, _2),
                      boost::bind(&HelpLightClient::activeCb, this),
                      boost::bind(&HelpLightClient::feedbackCb, this, _1));
        }

    }

    /**
     * Called for reset the action client (it cannot be running)
     */
    void reset()
    {
        done = false;
        running = false;
        ready = true;
        resultTriangles.clear();
        helpState=nothing;

    }
    void start()
    {
        reset();
        sendGoal();
    }


    void waitForResult(){
        ROS_INFO("[%s] Waiting for result...", action_name_.c_str());
        ac.waitForResult();
    }

    int is_help_required()
    {
        if(helpState == nothing) return 0;
        else if(helpState == help) return 1;
        else if(helpState == error) return -1;

    }

    vector<Triangle> getHelpLocations()
    {
        return resultTriangles;
    }

    void stop()
    {
        ready=false;
        running=false;
        ac.cancelAllGoals();
    }

    void acknowledge_help(){
      
        drrobot_h20_player::HeadCmd cmdhead_;

        int neck_x_rotation_ = 3800;
        int neck_z_rotation_= 3450;
        int mouth_= 3500;
        int upper_head_ = 3500;
        int left_eye_ = 3600;
        int right_eye_ = 3350;
        cmdhead_.neck_x_rotation = neck_x_rotation_; //Head Tilt
        cmdhead_.neck_z_rotation = neck_z_rotation_; //Head Pan
        cmdhead_.mouth = mouth_;
        cmdhead_.upper_head = upper_head_;
        cmdhead_.left_eye = left_eye_;
        cmdhead_.right_eye = right_eye_;
        cmdhead_.flag = 1000;
    
        for(int i =0; i < resultTriangles.size(); i++)
        {
            double x = resultTriangles[i].locationX;
            double y = resultTriangles[i].locationY;
            
            double ang = atan ( y/x );
            ROS_INFO("Triangle.locationX= %f", x);
            ROS_INFO("Triangle.locationY= %f", y);
            ROS_INFO("Head moving to angle %f", ang);
            //Assuming that the head pan servo has max/rest/min servo values (4550/3450/2350) corresponding to (+90deg/0deg/-90deg)
            //Can get that 1 deg ~= 12.222 servo increments; use 18 to exaggerate difference
            for(int i =0; i<10; i++){
                cmdhead_.neck_z_rotation= int((-ang*180/PI)/10*i*18)+3450;
                pub_head.publish(cmdhead_);
                usleep(100000);
            }
            pub_head.publish(cmdhead_);
            usleep(500000);
            //Nod action
            //Assuming that the head tilt servo has rest/min servo values (3800/3050) corresponding to (0deg/-45deg)
            //Can get that 1 deg ~= 23.333 servo increments
            for(int i=0;i<10;i++){
              cmdhead_.neck_x_rotation= 3800 - int(2*i*23.333);
              
              pub_head.publish(cmdhead_);
              usleep(50000);
            }
            pub_head.publish(cmdhead_);
            for(int i=0;i<10;i++){
              cmdhead_.neck_x_rotation= 3300 + int(2*i*23.333);
              
              pub_head.publish(cmdhead_);
              usleep(50000);
            }
            pub_head.publish(cmdhead_);
            cmdhead_.neck_x_rotation=3800;
            usleep(500000);
            double curr_ang=cmdhead_.neck_z_rotation;
            for(int i =0; i<10; i++){
                cmdhead_.neck_z_rotation= curr_ang-int((ang*180/PI)/10*i*12.222);
                
                pub_head.publish(cmdhead_);
                usleep(100000);
            }
            pub_head.publish(cmdhead_);
        }
      
    }


private:
    string action_name_;

};
