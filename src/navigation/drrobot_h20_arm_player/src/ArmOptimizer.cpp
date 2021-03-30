#include <ArmOptimizer.h>

ArmOptimizer::ArmOptimizer(std::string name){
  curr_group=0;
}
ArmOptimizer::~ArmOptimizer(){
  
}

void ArmOptimizer::add_plan(moveit::planning_interface::MoveGroup::Plan plan){
  if(curr_group==0){
    curr_group=plan.trajectory_.joint_trajectory.joint_names.size();
  }else if(plan.trajectory_.joint_trajectory.joint_names.size()!=curr_group){
    ROS_ERROR("Cannot optimize differently sized kinematic chains!");
  }else{
    plans.push_back(plan);
  }
}

void ArmOptimizer::clearPlans(){
  plans.clear();
}

vector<double> ArmOptimizer::scoreTime(){
  ROS_INFO("Beginning to score %d plans for time!", plans.size());
  vector<double> scores;
  for(int i=0; i<plans.size(); i++){
    moveit::planning_interface::MoveGroup::Plan curr_plan=plans.at(i);
    ros::Duration ending;
    ending=curr_plan.trajectory_.joint_trajectory.points.back().time_from_start;
    scores.push_back(ending.toSec());
  }
  return scores;
}


//void ArmOptimizer::deriv(){
//}

moveit::planning_interface::MoveGroup::Plan ArmOptimizer::getBestPlan(){
  vector<double> timeScores;
  vector<double> enScores;
  
  timeScores=scoreTime();
  for(int s=0;s<timeScores.size();s++){
    ROS_INFO("Time score=%f",timeScores.at(s));
  }

  if(timeScores.size()<2){
    return plans.front();
  }else{
    int iter_lowest=0;
    int iter_lower=0;
    double lowestT=9999999999;
    double lowerT=9999999999;
    for(int i=0; i<timeScores.size();i++){
      if(timeScores[i]<lowestT || timeScores[i]==lowestT){
        lowerT=lowestT;
        lowestT=timeScores[i];
        iter_lower=iter_lowest;
        iter_lowest=i;
      }else if(timeScores[i]<lowerT && timeScores[i]>lowestT){
        iter_lower=i;
        lowerT=timeScores[i];
      }
    }
    
    ROS_INFO("Lowest time = %f", lowestT);
    ROS_INFO("Second lowest time = %f", lowerT);
    if(abs(lowestT-lowerT)<0.5){
      //Get rid of all plans which we don't need to consider
      moveit::planning_interface::MoveGroup::Plan placeholder1;
      moveit::planning_interface::MoveGroup::Plan placeholder2;
      placeholder1=plans.at(iter_lowest);
      placeholder2=plans.at(iter_lower);
      plans.clear();
      
      //Repopulate plans with the lowest time score plans
      plans.push_back(placeholder1);
      plans.push_back(placeholder2);
      
      //Score energy of the remaining plans
      enScores=ArmOptimizer::scoreEnergy();
      
      if(enScores[0]>enScores[1]){
        return plans.at(1);
      }else{
        return plans.at(0);
      }
    }else{
      return plans.at(iter_lowest);
    }
  }
}
