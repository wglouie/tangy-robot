#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <drrobot_h20_arm_player/armAction.h>
#include <string>
#include <ArmMovementClient.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "classtest");

  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
    ArmMovementClient client(ros::this_node::getName());

  /* Sending a command to the server:
  Send either a x,y,z coordinate using send(double x, double y, double z)
  Or send a behavior using send(string behavior).
  List of behaviors: "plan point at screen", "plan celebrate", "plan wave goodbye"
  and "execute point at screen", "execute celebrate", "execute wave goodbye", "execute point to"*/
  // if(client.state==0){
  //   client.send("test");
  //   client.state=1;
  // }

//    client.send("plan celebrate");
  //client.send("execute celebrate");
//  sleep(30);
//   client.send("plan wave goodbye");
//   sleep(5);
//     client.send("execute wave goodbye");
// //     sleep(20);
//   sleep(1);
//     client.send("plan point at screen");
//     sleep(5);
// //     sleep(30);
//     client.send("execute point at screen");
//   sleep(1);
    client.send("plan celebrate");
    sleep(5);
    client.send("execute celebrate");

    sleep(20);

    client.send("plan celebrate");
    sleep(5);
    client.send("execute celebrate");

    sleep(20);

    client.send("plan celebrate");
    sleep(5);
    client.send("execute celebrate");
//     client.send(-0.15,-0.23,0.84);
  ros::spin();
    return 0;
}
