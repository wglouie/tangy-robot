#include<new_layer/new_layer.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <fstream>

PLUGINLIB_EXPORT_CLASS(new_layer_namespace::NewLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace new_layer_namespace
{

NewLayer::NewLayer() {}

NewLayer::~NewLayer() {
  goalService.shutdown();
}

void NewLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  goalService = nh.advertiseService("check_nav_goal", &NewLayer::checkNavigationGoalService, this);
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &NewLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void NewLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void NewLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                             double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  mark_x_ = origin_x + cos(origin_yaw);
  mark_y_ = origin_y + sin(origin_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void NewLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}

//////////////////CHECK NAVIGATION GOAL///////////////////////////////////
bool NewLayer::checkNavigationGoalService(navigation_experiment::CheckNavigationGoal::Request &req, navigation_experiment::CheckNavigationGoal::Response &res) {
  //get the costmap
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();

  //bounds stuff
  res.min_x = costmap->getOriginX();
  res.max_x = costmap->getSizeInMetersX();
  res.min_y = costmap->getOriginY();
  res.max_y = costmap->getSizeInMetersY();

  //check goal stuff
  bool *bPtr;
  bPtr = ((bool *) &res.safe);
  return checkNavigationGoal(req.x, req.y, *bPtr);
}

bool NewLayer::checkNavigationGoal(float x, float y, bool &safe) {
  //get the costmap
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();

  //resolution = meters/cell
  float resolution =  costmap->getResolution();

  //origin of our robots map (RVIZ) relative to the costmap (in meters)
  float origin_x = costmap->getOriginX();
  float origin_y = costmap->getOriginY();

  //shift our points (RVIZ) to be relative to the costmap
  x = x - origin_x;
  y = y - origin_y;

  //scale our points to be in cells instead of meters
  x = x / resolution;
  y = y / resolution;

  //determine if this spot on the costmap is free (0)
  int goal_value = costmap->getCost(x, y);

  if(goal_value == 0) { //space if free return that this point is safe
    safe = true;
    return true;
  }    
  //point is not safe
  safe = false;
  return true;
}
///////////////////////////////////////////////////////////////////////////

} // end namespace
