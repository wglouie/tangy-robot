#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <collision_avoider/check_collision.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <vector>
#include <algorithm>
#include "collision_avoider/geometry_math.h"

double PI = 3.1415926;
int R_arm = 50;
int R_torso = 90;
int R_head = 130;
int threshold_arm_sphere_new_usb = R_arm + 95;
int threshold_arm_torso = R_arm + R_torso;
int threshold_arm_head = R_arm + R_head;
int threshold_arm_cylinder_1 = R_arm+85;
int threshold_arm_cylinder_2 = 2*R_arm;
int threshold_arm_cylinder_3 = 2*R_arm;
int threshold_arm_cylinder_4 = 2*R_arm;
int threshold_arm_cylinder_5 = R_arm+85;
int threshold_arm_cylinder_6 = 2*R_arm;



RobotArm fwd_kin_right (double t5, double t6, double t7, double t8) {
    int shdt = 170; // shoulder to torso center
    int d1 = 239;
    int d2 = 400;


    RobotArm robot_right_arm;
    // Forward kinematics for robot elbow
    robot_right_arm.elbow.x = shdt - d1*cos(t6 + PI/2);
    robot_right_arm.elbow.y = d1*sin(t6 + PI/2)*cos(t5 - PI/2);
    robot_right_arm.elbow.z = d1*sin(t6 + PI/2)*sin(t5 - PI/2);
    // Forward kinematics for robot hand
    robot_right_arm.hand.x = shdt - d1*cos(t6 + PI/2) +
                            d2*cos(t6 + PI/2)*sin(t8 - PI/2) +
                            d2*cos(t7 + PI/2)*sin(t6 + PI/2)*cos(t8 - PI/2);
    robot_right_arm.hand.y = d1*sin(t6 + PI/2)*cos(t5 - PI/2) +
                            d2*cos(t8 - PI/2)*(sin(t7 + PI/2)*sin(t5 - PI/2) +
                            cos(t6 + PI/2)*cos(t7 + PI/2)*cos(t5 - PI/2)) -
                            d2*sin(t6 + PI/2)*cos(t5 - PI/2)*sin(t8 - PI/2);
    robot_right_arm.hand.z = d1*sin(t6 + PI/2)*sin(t5 - PI/2) -
                            d2*cos(t8 - PI/2)*(sin(t7 + PI/2)*cos(t5 - PI/2) -
                            cos(t6 + PI/2)*cos(t7 + PI/2)*sin(t5 - PI/2)) -
                            d2*sin(t6 + PI/2)*sin(t5 - PI/2)*sin(t8 - PI/2);
    // return value
    return robot_right_arm;
}

RobotArm fwd_kin_left (double t1, double t2, double t3, double t4) {
    int shdt = 170; // shoulder to torso center
    int d1 = 239;
    int d2 = 400;

    RobotArm robot_left_arm;
    // Forward kinematics for robot elbow
    robot_left_arm.elbow.x = - shdt - d1*cos(t2 - PI/2);
    robot_left_arm.elbow.y = -d1*cos(t1 - PI/2)*sin(t2 - PI/2);
    robot_left_arm.elbow.z = -d1*sin(t1 - PI/2)*sin(t2 - PI/2);
    // Forward kinematics for robot hand
    robot_left_arm.hand.x = d2*cos(t2 - PI/2)*sin(t4 - PI/2) -
                           d1*cos(t2 - PI/2) - shdt -
                           d2*cos(t3 - PI/2)*cos(t4 - PI/2)*
                           sin(t2 - PI/2);
    robot_left_arm.hand.y = d2*cos(t1 - PI/2)*sin(t2 - PI/2)*
                           sin(t4 - PI/2) - d2*cos(t4 - PI/2)*
                           (sin(t1 - PI/2)*sin(t3 - PI/2) -
                           cos(t1 - PI/2)*cos(t2 - PI/2)*
                           cos(t3 - PI/2)) - d1*cos(t1 - PI/2)*
                           sin(t2 - PI/2);
    robot_left_arm.hand.z = d2*cos(t4 - PI/2)*(cos(t1 - PI/2)*
                           sin(t3 - PI/2) + cos(t2 - PI/2)*
                           cos(t3 - PI/2)*sin(t1 - PI/2)) -
                           d1*sin(t1 - PI/2)*sin(t2 - PI/2) +
                           d2*sin(t1 - PI/2)*sin(t2 - PI/2)*
                           sin(t4 - PI/2);
    // return value
    return robot_left_arm;
}

double cons1 (double angle1, double angle2, double angle3, double angle4, Point hand, Point elbow)
{
    const Point torso_top = {.x=0, .y=0, .z=250};
    const Point torso_bottom = {.x=0, .y=0, .z=-750};
    double arm_torso = SegToSegDist (torso_top, torso_bottom, hand, elbow) -
                                threshold_arm_torso;
    return arm_torso;
}


double cons2 (double angle1, double angle2, double angle3, double angle4, Point hand, Point elbow)
{
    const Point head_center = {.x=0, .y=0, .z=250};
    double arm_head = PointToSegDist(head_center, hand, elbow) -
                                threshold_arm_head;
    return arm_head;
}


//USB
double cons3 (double angle1, double angle2, double angle3, double angle4, Point hand, Point elbow)
{
    const Point sphere4_center = {.x=150, .y=200, .z=-150};
    double arm_sphere4 = PointToSegDist(sphere4_center, hand, elbow) -
                                threshold_arm_sphere_new_usb;
    return arm_sphere4;
}

//USB
double cons4 (double angle1, double angle2, double angle3, double angle4, Point hand, Point elbow)
{
    const Point sphere24_center = {.x=-150, .y=200, .z=-150};
    double arm_sphere24 = PointToSegDist(sphere24_center, hand, elbow) -
                                threshold_arm_sphere_new_usb;
    return arm_sphere24;
}

double cons5 (double angle1, double angle2, double angle3, double angle4)
{
    return angle2;
}

// cylinder 1

double cons_c1 (double angle1, double angle2, double angle3, double angle4, Point hand, Point elbow)
{
    const Point body_cylinder_top = {.x=150, .y=120, .z=95};
    const Point body_cylinder_bottom = {.x=150, .y=200, .z=-150};
    double arm_torso = SegToSegDist (body_cylinder_top, body_cylinder_bottom, hand, elbow) -
                                threshold_arm_cylinder_1;
    return arm_torso;
}

double cons_c2 (double angle1, double angle2, double angle3, double angle4, Point hand, Point elbow)
{
    const Point body_cylinder_top = {.x=75, .y=120, .z=95};
    const Point body_cylinder_bottom = {.x=75, .y=200, .z=-150};
    double arm_torso = SegToSegDist (body_cylinder_top, body_cylinder_bottom, hand, elbow) -
                                threshold_arm_cylinder_2;
    return arm_torso;
}


double cons_c3 (double angle1, double angle2, double angle3, double angle4, Point hand, Point elbow)
{
    const Point body_cylinder_top = {.x=0, .y=120, .z=95};
    const Point body_cylinder_bottom = {.x=0, .y=200, .z=-150};
    double arm_torso = SegToSegDist (body_cylinder_top, body_cylinder_bottom, hand, elbow) -
                                threshold_arm_cylinder_3;
    return arm_torso;
}


double cons_c4 (double angle1, double angle2, double angle3, double angle4, Point hand, Point elbow)
{
    const Point body_cylinder_top = {.x=-75, .y=120, .z=95};
    const Point body_cylinder_bottom = {.x=-75, .y=200, .z=-150};
    double arm_torso = SegToSegDist (body_cylinder_top, body_cylinder_bottom, hand, elbow) -
                                threshold_arm_cylinder_4;
    return arm_torso;
}

double cons_c5 (double angle1, double angle2, double angle3, double angle4, Point hand, Point elbow)
{
    const Point body_cylinder_top = {.x=-150, .y=120, .z=95};
    const Point body_cylinder_bottom = {.x=-150, .y=200, .z=-150};
    double arm_torso = SegToSegDist (body_cylinder_top, body_cylinder_bottom, hand, elbow) -
                                threshold_arm_cylinder_5;
    return arm_torso;
}

double cons_c6 (double angle1, double angle2, double angle3, double angle4, Point hand, Point elbow)
{
    const Point body_cylinder_top = {.x=150, .y=145, .z=-220};
    const Point body_cylinder_bottom = {.x=-150, .y=145, .z=-220};
    double arm_torso = SegToSegDist (body_cylinder_top, body_cylinder_bottom, hand, elbow) -
                                threshold_arm_cylinder_6;
    return arm_torso;
}


bool is_there_collision(float angle1, float angle2, float angle3, float angle4, RobotArm arm){
    double cons_value_1 = cons1(angle1, angle2, angle3, angle4, arm.hand, arm.elbow);
    double cons_value_2 = cons2(angle1, angle2, angle3, angle4, arm.hand, arm.elbow);
    double cons_value_3 = cons3(angle1, angle2, angle3, angle4, arm.hand, arm.elbow);
    double cons_value_4 = cons4(angle1, angle2, angle3, angle4, arm.hand, arm.elbow);
    double cons_value_5 = cons5(angle1, angle2, angle3, angle4);
    double cons_value_6 = cons_c1(angle1, angle2, angle3, angle4, arm.hand, arm.elbow);
    double cons_value_7 = cons_c2(angle1, angle2, angle3, angle4, arm.hand, arm.elbow);
    double cons_value_8 = cons_c3(angle1, angle2, angle3, angle4, arm.hand, arm.elbow);
    double cons_value_9 = cons_c4(angle1, angle2, angle3, angle4, arm.hand, arm.elbow);
    double cons_value_10 = cons_c5(angle1, angle2, angle3, angle4, arm.hand, arm.elbow);
    double cons_value_11 = cons_c6(angle1, angle2, angle3, angle4, arm.hand, arm.elbow);
    if(cons_value_1 < 0 ||  cons_value_2 < 0 || cons_value_3 < 0  || cons_value_4 < 0
            || cons_value_5 < 0 ||  cons_value_6 < 0 || cons_value_7 < 0  || cons_value_8 < 0
            || cons_value_9 < 0 ||  cons_value_10 < 0 || cons_value_11 < 0){
        return true;
    } else {
       return false;
    }
}


class CollisionAvoider{
    ros::NodeHandle nh;
    ros::ServiceServer check_collision_srv;

public:
    CollisionAvoider(){
        check_collision_srv = nh.advertiseService("collision_avoider/check_collision", &CollisionAvoider::check_collision_cb, this);
    }

private:
    bool check_collision_cb(collision_avoider::check_collision::Request &req,
                     collision_avoider::check_collision::Response &res){
        RobotArm right_arm = fwd_kin_right(req.right_arm_angles[0],req.right_arm_angles[1], req.right_arm_angles[2], req.right_arm_angles[3]);
        RobotArm left_arm = fwd_kin_left(req.left_arm_angles[0],req.left_arm_angles[1], req.left_arm_angles[2], req.left_arm_angles[3]);
        res.is_there_collision_right = is_there_collision(req.right_arm_angles[0],req.right_arm_angles[1], req.right_arm_angles[2], req.right_arm_angles[3], right_arm);
        res.is_there_collision_left = is_there_collision(req.left_arm_angles[0],req.left_arm_angles[1], req.left_arm_angles[2], req.left_arm_angles[3], left_arm);
//up two times
        return true;
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "collision_avoider");
    CollisionAvoider collision_avoider;
    ros::spin();
    return 0;
}
