#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <vector>
#include <algorithm>
#include "coblya_test_package/geometry_math.h"
#include <nlopt.hpp>
#include <coblya_test_package/optimize_angles.h>


double PI = 3.1415926;
int R_arm = 50;
int R_torso = 90;
int R_head = 100;
int threshold_arm_sphere_new_usb = R_arm + 95;
int threshold_arm_torso = R_arm + R_torso;
int threshold_arm_head = R_arm+R_head;
int threshold_arm_cylinder_1 = 2*R_arm;
int threshold_arm_cylinder_2 = 2*R_arm;
int threshold_arm_cylinder_3 = 2*R_arm;
int threshold_arm_cylinder_4 = 2*R_arm;
int threshold_arm_cylinder_5 = 2*R_arm;
int threshold_arm_cylinder_6 = 2*R_arm;

static const double lower_bounds_arr[] = {-1*PI,0,-1*PI,-1*PI};
static const double upper_bounds_arr[] = {PI,PI,PI,PI};

enum Arm{LEFT,RIGHT};

struct OldAngleData{
		Arm arm;
        double old_angle1;
        double old_angle2;
        double old_angle3;
        double old_angle4;
};


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


double objective_func(const std::vector<double> &x, std::vector<double> &grad, void* data)
{
    OldAngleData *d = (OldAngleData *) data;
    double angle1_cost = (x[0]-d->old_angle1)*(x[0]-d->old_angle1);
    double angle2_cost = (x[1]-d->old_angle2)*(x[1]-d->old_angle2);
    double angle3_cost = (x[2]-d->old_angle3)*(x[2]-d->old_angle3);
    double angle4_cost = (x[3]-d->old_angle4)*(x[3]-d->old_angle4);

    double cost = angle1_cost + angle2_cost + angle3_cost + angle4_cost;
    return cost;
}



double cons1(const std::vector<double> &x, std::vector<double> &grad, void* data)
{
    OldAngleData *d = (OldAngleData *) data;
    const Point torso_top = {.x=0, .y=0, .z=250};
    const Point torso_bottom = {.x=0, .y=0, .z=-750};
	RobotArm arm;
	if(d->arm == RIGHT)
		arm = fwd_kin_right(x[0],x[1],x[2],x[3]);
	else
		arm = fwd_kin_left(x[0],x[1],x[2],x[3]);
		
    double arm_torso = SegToSegDist (torso_top, torso_bottom, arm.hand, arm.elbow) -
                                threshold_arm_torso;
    return  arm_torso;
}


double cons2(const std::vector<double> &x, std::vector<double> &grad, void* data)
{
	OldAngleData *d = (OldAngleData *) data;
    const Point head_center = {.x=0, .y=0, .z=250};
	RobotArm arm;
	if(d->arm == RIGHT)
		arm = fwd_kin_right(x[0],x[1],x[2],x[3]);
	else
		arm = fwd_kin_left(x[0],x[1],x[2],x[3]);
    double arm_head = PointToSegDist(head_center, arm.hand, arm.elbow) -
                                threshold_arm_head;
    return arm_head;
}


//USB
double cons3 (const std::vector<double> &x, std::vector<double> &grad, void* data)
{
	OldAngleData *d = (OldAngleData *) data;
    const Point sphere4_center = {.x=150, .y=200, .z=-150};
	RobotArm arm;
	if(d->arm == RIGHT)
		arm = fwd_kin_right(x[0],x[1],x[2],x[3]);
	else
		arm = fwd_kin_left(x[0],x[1],x[2],x[3]);
    double arm_sphere4 = PointToSegDist(sphere4_center, arm.hand, arm.elbow) -
                                threshold_arm_sphere_new_usb;
    return arm_sphere4;
}

//USB
double cons4 (const std::vector<double> &x, std::vector<double> &grad, void* data)
{
	OldAngleData *d = (OldAngleData *) data;
    const Point sphere24_center = {.x=-75, .y=200, .z=-150};
	RobotArm arm;
	if(d->arm == RIGHT)
		arm = fwd_kin_right(x[0],x[1],x[2],x[3]);
	else
		arm = fwd_kin_left(x[0],x[1],x[2],x[3]);
    double arm_sphere24 = PointToSegDist(sphere24_center, arm.hand, arm.elbow) -
                                threshold_arm_sphere_new_usb;
    return arm_sphere24;
}

// cylinder 1

double cons_c1(const std::vector<double> &x, std::vector<double> &grad, void* data)
{
	OldAngleData *d = (OldAngleData *) data;
    const Point body_cylinder_top = {.x=150, .y=120, .z=95};
    const Point body_cylinder_bottom = {.x=150, .y=200, .z=-220};
	RobotArm arm;
	if(d->arm == RIGHT)
		arm = fwd_kin_right(x[0],x[1],x[2],x[3]);
	else
		arm = fwd_kin_left(x[0],x[1],x[2],x[3]);
    double arm_torso = SegToSegDist (body_cylinder_top, body_cylinder_bottom, arm.hand, arm.elbow) -  threshold_arm_cylinder_1;
    return arm_torso;
}

double cons_c2(const std::vector<double> &x, std::vector<double> &grad, void* data)
{
	OldAngleData *d = (OldAngleData *) data;
    const Point body_cylinder_top = {.x=75, .y=100, .z=95};
    const Point body_cylinder_bottom = {.x=75, .y=180, .z=-220};
	RobotArm arm;
	if(d->arm == RIGHT)
		arm = fwd_kin_right(x[0],x[1],x[2],x[3]);
	else
		arm = fwd_kin_left(x[0],x[1],x[2],x[3]);
    double arm_torso = SegToSegDist (body_cylinder_top, body_cylinder_bottom, arm.hand, arm.elbow) - threshold_arm_cylinder_2;
    return arm_torso;
}

double cons_c3(const std::vector<double> &x, std::vector<double> &grad, void* data)
{
	OldAngleData *d = (OldAngleData *) data;
    const Point body_cylinder_top = {.x=0, .y=100, .z=95};
    const Point body_cylinder_bottom = {.x=0, .y=180, .z=-220};
	RobotArm arm;
	if(d->arm == RIGHT)
		arm = fwd_kin_right(x[0],x[1],x[2],x[3]);
	else
		arm = fwd_kin_left(x[0],x[1],x[2],x[3]);
    double arm_torso = SegToSegDist (body_cylinder_top, body_cylinder_bottom, arm.hand, arm.elbow) -threshold_arm_cylinder_3;
    return arm_torso;
}


double cons_c4(const std::vector<double> &x, std::vector<double> &grad, void* data)
{
	OldAngleData *d = (OldAngleData *) data;
    const Point body_cylinder_top = {.x=-75, .y=100, .z=95};
    const Point body_cylinder_bottom = {.x=-75, .y=180, .z=-220};
	RobotArm arm;
	if(d->arm == RIGHT)
		arm = fwd_kin_right(x[0],x[1],x[2],x[3]);
	else
		arm = fwd_kin_left(x[0],x[1],x[2],x[3]);
    double arm_torso = SegToSegDist (body_cylinder_top, body_cylinder_bottom, arm.hand, arm.elbow) -threshold_arm_cylinder_4;
    return arm_torso;
}

double cons_c5(const std::vector<double> &x, std::vector<double> &grad, void* data)
{
	OldAngleData *d = (OldAngleData *) data;
    const Point body_cylinder_top = {.x=-150, .y=100, .z=95};
    const Point body_cylinder_bottom = {.x=-150, .y=180, .z=-220};
	RobotArm arm;
	if(d->arm == RIGHT)
		arm = fwd_kin_right(x[0],x[1],x[2],x[3]);
	else
		arm = fwd_kin_left(x[0],x[1],x[2],x[3]);
    double arm_torso = SegToSegDist (body_cylinder_top, body_cylinder_bottom, arm.hand, arm.elbow) - threshold_arm_cylinder_5;
    return arm_torso;
}

double cons_c6(const std::vector<double> &x, std::vector<double> &grad, void* data)
{
	OldAngleData *d = (OldAngleData *) data;
    const Point body_cylinder_top = {.x=150, .y=145, .z=-220};
    const Point body_cylinder_bottom = {.x=-150, .y=145, .z=-220};
	RobotArm arm;
	if(d->arm == RIGHT)
		arm = fwd_kin_right(x[0],x[1],x[2],x[3]);
	else
		arm = fwd_kin_left(x[0],x[1],x[2],x[3]);
    double arm_torso = SegToSegDist (body_cylinder_top, body_cylinder_bottom, arm.hand,arm.elbow) - threshold_arm_cylinder_6;
    return arm_torso;
}

void optimization(std::vector<double> &input_angles, Arm arm){
    nlopt::opt opt(nlopt::LN_COBYLA, 4);

    OldAngleData old_angle_data = {arm, input_angles[0],input_angles[1],input_angles[2],input_angles[3]};
    opt.set_min_objective(objective_func,&old_angle_data);

    std::vector<double> lower_bounds (lower_bounds_arr, lower_bounds_arr + sizeof(lower_bounds_arr) / sizeof(lower_bounds_arr[0]) );
    std::vector<double> upper_bounds (upper_bounds_arr, upper_bounds_arr + sizeof(upper_bounds_arr) / sizeof(upper_bounds_arr[0]) );
    opt.set_lower_bounds(lower_bounds);
    opt.set_upper_bounds(upper_bounds);
    opt.add_inequality_constraint(cons1, &old_angle_data, 1e-8);
    opt.add_inequality_constraint(cons2, &old_angle_data, 1e-8);
    opt.add_inequality_constraint(cons3, &old_angle_data, 1e-8);
    opt.add_inequality_constraint(cons4, &old_angle_data, 1e-8);
    opt.add_inequality_constraint(cons_c1, &old_angle_data, 1e-8);
    opt.add_inequality_constraint(cons_c2, &old_angle_data, 1e-8);
    opt.add_inequality_constraint(cons_c3, &old_angle_data, 1e-8);
    opt.add_inequality_constraint(cons_c4, &old_angle_data, 1e-8);
    opt.add_inequality_constraint(cons_c5, &old_angle_data, 1e-8);
    opt.add_inequality_constraint(cons_c6, &old_angle_data, 1e-8);
    opt.set_ftol_rel(1e-4);
    double minf;
    if (opt.optimize(input_angles, minf) < 0) {
        ROS_INFO("Optimization failed!\n");
    }
    else {
        //ROS_INFO("found minimum at f(%f,%f,%f,%f) = %0.10f\n", input_angles[0], input_angles[1], input_angles[2], input_angles[3], minf);
    }
}




class CollisionAvoider{
    ros::NodeHandle nh;
    ros::ServiceServer optimize_angle_srv;

public:
    CollisionAvoider(){
        optimize_angle_srv = nh.advertiseService("cobyla_test_package/optimize_angle", &CollisionAvoider::optimize_angle_cb, this);
    }

private:
    bool optimize_angle_cb(coblya_test_package::optimize_angles::Request &req,
                     coblya_test_package::optimize_angles::Response &res){
        std::vector<double> right_arm_input_angles;
        right_arm_input_angles.push_back(req.right_arm_angles[0]);
        right_arm_input_angles.push_back(req.right_arm_angles[1]);
        right_arm_input_angles.push_back(req.right_arm_angles[2]);
        right_arm_input_angles.push_back(req.right_arm_angles[3]);
		ROS_INFO("RIGHT ARM Original angles at f(%lf,%lf,%lf,%lf)", right_arm_input_angles[0], right_arm_input_angles[1], right_arm_input_angles[2], right_arm_input_angles[3]);
        optimization(right_arm_input_angles, RIGHT);
        ROS_INFO("RIGHT ARM Found minimum at f(%lf,%lf,%lf,%lf)", right_arm_input_angles[0], right_arm_input_angles[1], right_arm_input_angles[2], right_arm_input_angles[3]);
        res.optimized_right_arm_angles[0] = right_arm_input_angles[0];
        res.optimized_right_arm_angles[1] = right_arm_input_angles[1];
        res.optimized_right_arm_angles[2] = right_arm_input_angles[2];
        res.optimized_right_arm_angles[3] = right_arm_input_angles[3];

        std::vector<double> left_arm_input_angles;
        left_arm_input_angles.push_back(req.right_arm_angles[0]);
        left_arm_input_angles.push_back(req.right_arm_angles[1]);
        left_arm_input_angles.push_back(req.right_arm_angles[2]);
        left_arm_input_angles.push_back(req.right_arm_angles[3]);
		ROS_INFO("LEFT ARM Original angles at f(%lf,%lf,%lf,%lf)", left_arm_input_angles[0], left_arm_input_angles[1], left_arm_input_angles[2], left_arm_input_angles[3]);
        optimization(left_arm_input_angles, LEFT);
        ROS_INFO("LEFT ARM Found minimum at f(%lf,%lf,%lf,%lf)", left_arm_input_angles[0], left_arm_input_angles[1], left_arm_input_angles[2], left_arm_input_angles[3]);
        res.optimized_left_arm_angles[0] = left_arm_input_angles[0];
        res.optimized_left_arm_angles[1] = left_arm_input_angles[1];
        res.optimized_left_arm_angles[2] = left_arm_input_angles[2];
        res.optimized_left_arm_angles[3] = left_arm_input_angles[3];

        return true;
    }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "cobyla_optimizer");
    CollisionAvoider test;
    ros::spin();
    return 0;
}
