#include <ArmOptimizer.h>

vector<double> ArmOptimizer::scoreEnergy(){
  ROS_INFO("Beginning to score plans for energy!");
  vector<double> motorConsumption;
  motorConsumption.push_back(3.02);
  motorConsumption.push_back(1.0);
  motorConsumption.push_back(3.02);
  motorConsumption.push_back(1.0);
  motorConsumption.push_back(1.58);
  motorConsumption.push_back(1.58);
  vector<double> energyScores;
  
  for(int p=0; p<plans.size(); p++){
    vector< vector<double> > theta;
    vector< vector<double> > dtheta;
    vector< vector<double> > ddtheta;
    vector<ros::Duration> durations;
    vector< vector<double> > M;
    vector< vector<double> > C;
    vector<double> G;
    double scorePerPlan=0;
    //Populate angle position vectors in order to find their derivatives
    for(int pt=1; pt<plans[p].trajectory_.joint_trajectory.points.size(); pt++){
        ROS_INFO("Scoring point %d", pt);
        double scorePerPoint=0;
        
        if(pt==1){
          theta.push_back(plans[p].trajectory_.joint_trajectory.points[0].positions);
          dtheta.push_back(plans[p].trajectory_.joint_trajectory.points[0].velocities);
          ddtheta.push_back(plans[p].trajectory_.joint_trajectory.points[0].accelerations);
          durations.push_back(plans[p].trajectory_.joint_trajectory.points[0].time_from_start);
        }
        theta.push_back(plans[p].trajectory_.joint_trajectory.points[pt].positions);
        dtheta.push_back(plans[p].trajectory_.joint_trajectory.points[pt].velocities);
        ddtheta.push_back(plans[p].trajectory_.joint_trajectory.points[pt].accelerations);
        durations.push_back(plans[p].trajectory_.joint_trajectory.points[pt].time_from_start);

        M= dynaMatM(theta[pt], dtheta[pt]);
        C= dynaMatC(theta[pt], dtheta[pt]);
        G= dynaMatG(theta[pt], dtheta[pt]);
        ROS_INFO("Matrices populated!");
        vector<double> tau(6,0);
        for(int row=0; row<M.size(); row++){
          double sum1=0;
          double sum2=0;
          for(int col=0; col<M.size(); col++){
            sum1=M[row][col]*ddtheta[pt][col]+sum1;
            sum2=C[row][col]*dtheta[pt][col]+sum2;
          }
          tau[row]=sum1+sum2+G[row];
          
        }
        ROS_INFO("tau generated! tau=%f,%f,%f,%f,%f,%f", tau[0], tau[1], tau[2], tau[3], tau[4], tau[5]);
        for(int row=0; row<tau.size(); row++){
          scorePerPoint=scorePerPoint+abs(tau[row]*motorConsumption[row]*dtheta[pt][row])*(durations[pt].toSec()-durations[pt-1].toSec());
        }
        
        scorePerPlan=scorePerPoint+scorePerPlan;
    }
    
    energyScores.push_back(scorePerPlan);
  }
  
  for(int i=0; i<energyScores.size(); i++){
    ROS_INFO("Energy score of plan %d is: %f", i, energyScores.at(i));
  }
  return energyScores;
}