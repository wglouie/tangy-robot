#include <tangyRobot.h>

tangyRobot::tangyRobot(std::string name):
    robotGuiClient(name + "RobotGuiClient"),
    armMovementClient(name + "ArmMovementClient"),
    moveBaseClient(name + "MoveBaseClient"),
    faceDetectionClient(name +"FaceDetectionClient"),
    navClient(),
    musicPlayer(),
    neckHandler(name + "robot"){
    init(); //CHECK TO SEE IF THIS IS IN THE RIGHT PLACE
}

tangyRobot::tangyRobot(float wait_time, std::string name):
    robotGuiClient(name + "RobotGuiClient",wait_time),
    armMovementClient(name + "ArmMovementClient",wait_time),
    moveBaseClient(name + "MoveBaseClient",wait_time),
    faceDetectionClient(name + "FaceDetectionClient",wait_time),
    navClient(),
    musicPlayer(),
    neckHandler(name + "robot"){
    init(); //CHECK TO SEE IF THIS IS IN THE RIGHT PLACE
}

tangyRobot::~tangyRobot(){

}

bool tangyRobot::init(){
    navClient.initialize(nh_);
    pause=false;
    wave_plan_num=0;
    point_at_screen_plan_num=0;
    celebrate_plan_num=0;
    convo_gesture_num=0;
    laugh_gesture_num=0;

    nod_pub=nh_.advertise<std_msgs::Bool>("head_ready",1000);
    start_track_face_pub=nh_.advertise<std_msgs::String>("start_track_face",1000);
    stop_track_face_pub=nh_.advertise<std_msgs::String>("stop_track_face",1000);
    stop_neck_movement = nh_.advertise<std_msgs::String>("stop_neck_movement",1);
    pick_a_person_pub = nh_.advertise<std_msgs::String>("pick_a_person",1);
    // pause_sub=nh_.subscribe<std_msgs::String>("control_command",10, &tangyRobot::pauseCallback, this);
    plan_clap();
    plan_wave();
    plan_point_at_screen();
    plan_celebrate();
    plan_laugh_gesture();
    plan_convo_gesture();
  
  
    ROS_INFO("---------ROBOT INITIALIZED------------");
    nod();

/*  while(1){
    moveNeck(25,-5);
    sleep(5);
    moveNeck(-25, -5);
    sleep(5);
    moveNeck(0,0);
  }*/
}

