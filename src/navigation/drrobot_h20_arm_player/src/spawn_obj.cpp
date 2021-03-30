//============================================================================
// Name        : spawn_obj.cpp
// Author      : Ananias Paiva
// Copyright   :
// Description : Spawn objects on rviz for tests
//============================================================================

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
//#include <solid_primitive_dims.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "collision");
  ros::NodeHandle nh;
  
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

  moveit_msgs::CollisionObject co;
  ros::Rate r(20);
  
  while(nh.ok())
  {
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "/world"; //change this for yours

  // add a box
  co.operation = moveit_msgs::CollisionObject::ADD;
  
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  
  co.primitives[0].dimensions.resize((static_cast<int>(shape_msgs::SolidPrimitive::BOX_X) >= static_cast<int>(shape_msgs::SolidPrimitive::BOX_Y) && static_cast<int>(shape_msgs::SolidPrimitive::BOX_X) >= static_cast<int>(shape_msgs::SolidPrimitive::BOX_Z)) ?
static_cast<int>(shape_msgs::SolidPrimitive::BOX_X) :
(((static_cast<int>(shape_msgs::SolidPrimitive::BOX_Y) >= static_cast<int>(shape_msgs::SolidPrimitive::BOX_X) &&
static_cast<int>(shape_msgs::SolidPrimitive::BOX_Y) >= static_cast<int>(shape_msgs::SolidPrimitive::BOX_Z))) ?
static_cast<int>(shape_msgs::SolidPrimitive::BOX_Y) :
((static_cast<int>(shape_msgs::SolidPrimitive::BOX_Z) >= static_cast<int>(shape_msgs::SolidPrimitive::BOX_X) && static_cast<int>(shape_msgs::SolidPrimitive::BOX_Z) >= static_cast<int>(shape_msgs::SolidPrimitive::BOX_Y)) ? static_cast<int>(shape_msgs::SolidPrimitive::BOX_Z) : 0)) + 1);
  
  //box dimensions
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.2;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.2;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.2;
  //box position relative to frame_id
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.2;
  co.primitive_poses[0].position.y = -0.2;
  co.primitive_poses[0].position.z = 0.2;
  co.primitive_poses[0].orientation.w = 1.0;
  pub_co.publish(co);
  
  ros::spinOnce();
  r.sleep();
  
  }

}
