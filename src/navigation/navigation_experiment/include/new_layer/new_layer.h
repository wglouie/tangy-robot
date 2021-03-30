#ifndef NEW_LAYER_H_
#define NEW_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <navigation_experiment/CheckNavigationGoal.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetPlan.h>
#include <ros/callback_queue_interface.h>

namespace new_layer_namespace
{

class NewLayer : public costmap_2d::Layer
{
public:
  NewLayer();
  ~NewLayer();

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
protected:   
  bool checkNavigationGoalService(navigation_experiment::CheckNavigationGoal::Request &req, navigation_experiment::CheckNavigationGoal::Response &res);
  bool checkNavigationGoal(float x, float y, bool &safe);
private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  double mark_x_, mark_y_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  ros::ServiceServer goalService;
};
}
#endif

