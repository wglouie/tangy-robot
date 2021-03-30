#include <help_indicators/help_indicators.h>

/*
void Help::acknowledge_help(float x, float y) {
  	const float PI = 3.14159265359;
  
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

    double ang = atan ( y/x );
    //Assuming that the head pan servo has max/rest/min servo values (4550/3450/2350) corresponding to (+90deg/0deg/-90deg)
    //Can get that 1 deg ~= 12.222 servo increments; use 18 to exaggerate difference
    for(int i =0; i<10; i++){
        cmdhead_.neck_z_rotation= int((-ang*180/PI)/10*i*13)+3450;
        pub_head.publish(cmdhead_);
        usleep(150000);
    }
    pub_head.publish(cmdhead_);
    usleep(500000);
    //Nod action
    //Assuming that the head tilt servo has rest/min servo values (3800/3050) corresponding to (0deg/-45deg)
    //Can get that 1 deg ~= 23.333 servo increments
    for(int i=0;i<15;i++) {
      cmdhead_.neck_x_rotation= 3800 - int(2*i*23.333);
      
      pub_head.publish(cmdhead_);
      usleep(60000);
    }
    pub_head.publish(cmdhead_);
    for(int i=0;i<15;i++) {
      cmdhead_.neck_x_rotation= 3100 + int(2*i*23.333);
      
      pub_head.publish(cmdhead_);
      usleep(60000);
    }
    pub_head.publish(cmdhead_);
    cmdhead_.neck_x_rotation=3800;
    usleep(500000);
    double curr_ang=cmdhead_.neck_z_rotation;
    for(int i =0; i<10; i++) {
        cmdhead_.neck_z_rotation= curr_ang-int((ang*180/PI)/10*i*12.222);
        
        pub_head.publish(cmdhead_);
        usleep(150000);
    }
    pub_head.publish(cmdhead_);
  
}
*/
