// Instructions:
// In order to use this class, add plans to the ArmOptimizer object and then use
// the functions scoreTime() and scoreEnergy(). Retrieve the best plan based on
// these scores with the function getBestPlan()

#include <vector>
#include <sstream>
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

using namespace std;
//Defines the Optimizer class used to score a series of arm trajectory plans
//from the MoveIt Planner based on each of their time and energy consumption costs
class ArmOptimizer{
  
    static const int SINGLE_ARM=6;
    static const int DOUBLE_ARM=12;
    int curr_group;
  
public:

    ArmOptimizer(std::string name);
    ~ArmOptimizer(void);
    
    void add_plan(moveit::planning_interface::MoveGroup::Plan plan);
    vector<double> scoreTime();
    vector<double> scoreEnergy();
    
    vector< vector<double> > dynaMatM(vector<double> j, vector<double> dj);
    vector< vector<double> > dynaMatC(vector<double> j, vector<double> dj);
    vector<double> dynaMatG(vector<double> j, vector<double> dj);
    
    moveit::planning_interface::MoveGroup::Plan getBestPlan();

private:

    vector<moveit::planning_interface::MoveGroup::Plan> plans;

};
