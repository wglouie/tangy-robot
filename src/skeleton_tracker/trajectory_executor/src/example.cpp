#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <drrobot_h20_arm_player/joint_cmd.h>
#include <drrobot_h20_arm_player/arm_cmd.h>
#include <trajectory_executor/arm_trajectory.h>
#include <trajectory_executor/both_arm_trajectory.h>
#include <trajectory_executor/execute_fileAction.h>
#include <trajectory_executor/execute_both_arms.h>
#include <actionlib/server/simple_action_server.h>
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>

/*	
rest_pos_s0     //Right finger
rest_pos_s1     //Right finger
rest_pos_s2	//Right wrist pitch joint
rest_pos_s3	//Right wrist yaw roint
rest_pos_s4	//Right elbow
rest_pos_s5	//Right shoulder yaw joint
rest_pos_s6	//Right shoulder pitch joint
rest_pos_s7	//Right shoulder roll
rest_pos_s8     //Left finger
rest_pos_s9     //Left finger
rest_pos_s10	//Left wrist pitch joint
rest_pos_s11	//Left wrist yaw roint
rest_pos_s12	//Left elbow
rest_pos_s13	//Left shoulder yaw joint
rest_pos_s14	//Left shoulder pitch joint
rest_pos_s15	//Left shoulder roll
*/
class TrajectoryExecutor{
    ros::NodeHandle nh;
    ros::Subscriber right_arm_traj_sub;
    ros::Subscriber left_arm_traj_sub;
    ros::Subscriber go_to_neutral_sub;
    ros::Publisher right_arm_joint_pub;
    ros::Publisher left_arm_joint_pub;
    ros::ServiceServer execute_both_arms_srv;

    actionlib::SimpleActionServer<trajectory_executor::execute_fileAction> as_;
    std::string action_name_;
    trajectory_executor::execute_fileFeedback feedback_;
    trajectory_executor::execute_fileResult result_;
public:
    TrajectoryExecutor(std::string name) :
        as_(nh, name, false)
        , action_name_(name)
    {
       	right_arm_traj_sub = nh.subscribe("trajectory_executor/right_arm_traj",10, &TrajectoryExecutor::right_arm_cb, this);
        left_arm_traj_sub = nh.subscribe("trajectory_executor/left_arm_traj",10, &TrajectoryExecutor::left_arm_cb, this);
        go_to_neutral_sub = nh.subscribe("trajectory_executor/go_to_neutral", 10, &TrajectoryExecutor::go_to_neutral_cb, this);
        execute_both_arms_srv = nh.advertiseService("trajectory_executor/both_arm_traj", &TrajectoryExecutor::both_arm_cb, this);

        right_arm_joint_pub = nh.advertise<drrobot_h20_arm_player::arm_cmd>("r_arm_joint_cmds", 1000);
        left_arm_joint_pub = nh.advertise<drrobot_h20_arm_player::arm_cmd>("l_arm_joint_cmds", 1000);

        //register the goal and feeback callbacks

        as_.registerGoalCallback(boost::bind(&TrajectoryExecutor::goalCB, this));
        as_.start();
        ROS_INFO("Trajectory Executor action server started");
    }

private:
    bool both_arm_cb(trajectory_executor::execute_both_arms::Request &req,
                     trajectory_executor::execute_both_arms::Response &res){

        std::vector<drrobot_h20_arm_player::arm_cmd> left_trajectory;
        std::vector<drrobot_h20_arm_player::arm_cmd> right_trajectory;
        left_trajectory = req.trajectory.left_trajectory;
        right_trajectory = req.trajectory.right_trajectory;
        ros::Time start = ros::Time::now();
        ros::Time end = ros::Time::now();
        for(int point=0; point<left_trajectory.size(); point++){
            double time_difference = 0;
            while(time_difference < left_trajectory[point].velocity){
                time_difference = (end - start).toNSec() * 1e-6;
                end = ros::Time::now();
            }
            drrobot_h20_arm_player::arm_cmd goal;
            goal = left_trajectory[point];
            left_arm_joint_pub.publish(goal);
            goal = right_trajectory[point];
            right_arm_joint_pub.publish(goal);
            usleep(goal.velocity*1000);
            //ROS_INFO("Send new right arm point");
            start = ros::Time::now();
        }

        ROS_INFO("Completed both arm trajectories");
        res.success = true;
        return true;
    }

    void right_arm_cb(const trajectory_executor::arm_trajectory::ConstPtr& msg){
        std::vector<drrobot_h20_arm_player::arm_cmd> trajectory;
        trajectory = msg->trajectory;
				ros::Time start = ros::Time::now();
				ros::Time end = ros::Time::now();
        for(int point=0; point<trajectory.size(); point++){
						double time_difference = 0;
                        while(time_difference < trajectory[point].velocity){
							time_difference = (end - start).toNSec() * 1e-6;
							end = ros::Time::now();
		        }
            drrobot_h20_arm_player::arm_cmd goal;
            goal = trajectory[point];
            right_arm_joint_pub.publish(goal);
            usleep(goal.velocity*1000);
            //ROS_INFO("Send new right arm point");
						start = ros::Time::now();
				}
        ROS_INFO("Completed right arm trajectory");
    }

    void left_arm_cb(const trajectory_executor::arm_trajectory::ConstPtr& msg){
        std::vector<drrobot_h20_arm_player::arm_cmd> trajectory;
        trajectory = msg->trajectory;
        ros::Time start = ros::Time::now();
        ros::Time end = ros::Time::now();
        for(int point=0; point<trajectory.size(); point++){
            double time_difference = 0;
            while(time_difference < trajectory[point].velocity){
                time_difference = (end - start).toNSec() * 1e-6;
                end = ros::Time::now();
            }
            drrobot_h20_arm_player::arm_cmd goal;
            goal = trajectory[point];
            left_arm_joint_pub.publish(goal);
            //usleep(goal.velocity*1000);
            ROS_INFO("Send new left arm point");
						start = ros::Time::now();
        }
        ROS_INFO("Completed left arm trajectory");
    }
    void go_to_neutral_cb(const std_msgs::String::ConstPtr& msg){
        drrobot_h20_arm_player::arm_cmd goal;
        goal.joint_commands[0].joint_angle = 0.0;
        goal.joint_commands[1].joint_angle = 0.0;
        goal.joint_commands[2].joint_angle = 0.0;
        goal.joint_commands[3].joint_angle = 0.0;
        goal.joint_commands[4].joint_angle = 0.0;
        goal.joint_commands[5].joint_angle = 0.0;
        goal.velocity = 2000;
        left_arm_joint_pub.publish(goal);
        right_arm_joint_pub.publish(goal);
        ROS_INFO("Moving arms to neutral position");
    }
    void goalCB()
    {
        bool success = true;
        // accept the new goal
        std::string file_name = as_.acceptNewGoal()->file_name;
        if(!boost::filesystem::exists(file_name + "_left_arm.gesture")){
            ROS_INFO("%s: Aborted. Gesture file does not exist", action_name_.c_str());
            as_.setAborted(result_,"Trajectory Executor aborted. File does not exist!");
        }

        std::vector<drrobot_h20_arm_player::arm_cmd> left_trajectory;
        std::vector<drrobot_h20_arm_player::arm_cmd> right_trajectory;
        left_trajectory = read_gesture_file(file_name + "_left_arm.gesture");
        right_trajectory = read_gesture_file(file_name + "_right_arm.gesture");
        ros::Time start = ros::Time::now();
        ros::Time end = ros::Time::now();

        for(int point=0; point<left_trajectory.size(); point++){
            double time_difference = 0;
            while(time_difference < left_trajectory[point].velocity){
                time_difference = (end - start).toNSec() * 1e-6;
                end = ros::Time::now();
            }
            drrobot_h20_arm_player::arm_cmd goal;
            goal = left_trajectory[point];
            left_arm_joint_pub.publish(goal);
            goal = right_trajectory[point];
            right_arm_joint_pub.publish(goal);
            /*ROS_INFO("Right arm joint angles read were: [%f] [%f] [%f] [%f]", goal.joint_commands[5].joint_angle,
                                                                            goal.joint_commands[4].joint_angle,
                                                                            goal.joint_commands[3].joint_angle,
                                                                            goal.joint_commands[2].joint_angle);
            */
            usleep(goal.velocity*1000);
            start = ros::Time::now();
            if(as_.isPreemptRequested() || !ros::ok()){
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }
            //ROS_INFO("Time left: [%f]", (left_trajectory.size()-point)*goal.velocity*.001);
            feedback_.time_left = (left_trajectory.size()-point)*goal.velocity*.001; //Time left for gesture
            as_.publishFeedback(feedback_);
        }


        // set the action state to succeeded
        if(success){
            //ROS_INFO("Completed both arm trajectories");
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            result_.done = true;
            as_.setSucceeded(result_);
        } else {
            result_.done = false;
            as_.setAborted(result_,"Trajectory Executor was pre-empted by something!");
        }
        return;
    }

    std::vector<drrobot_h20_arm_player::arm_cmd> read_gesture_file(std::string file_name){
        std::vector<drrobot_h20_arm_player::arm_cmd> trajectory;
        std::fstream in(file_name.c_str());
        std::string line;

        while (std::getline(in, line)){
            drrobot_h20_arm_player::arm_cmd arm_cmd;
            std::stringstream ss(line);
            ss >> arm_cmd.joint_commands[5].joint_angle;
            ss >> arm_cmd.joint_commands[4].joint_angle;
            ss >> arm_cmd.joint_commands[3].joint_angle;
            ss >> arm_cmd.joint_commands[2].joint_angle;
            arm_cmd.velocity = 60;
            trajectory.push_back(arm_cmd);
        }
        return trajectory;
    }

};


int main(int argc, char **argv){
    ros::init(argc, argv, "trajectory_executor_server");
    TrajectoryExecutor trajectory_executor("trajectory_executor_server");
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}
