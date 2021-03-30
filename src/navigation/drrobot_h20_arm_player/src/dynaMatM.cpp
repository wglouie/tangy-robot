#include <ArmOptimizer.h>

vector< vector<double> > ArmOptimizer::dynaMatM(vector<double> j, vector<double> dj){
  vector< vector<double> > M;
  for (int i=0; i<6; i++){
    vector<double> vec(6,0);
    M.push_back(vec);
  }
/*  for (int i=0; i<6; i++){
    M[i].push_back(0);
  }*/
  
  try{
    double j1=j[0];
    double j2=j[1];
    double j3=j[2];
    double j4=j[3];
    double j5=j[4];
    double j6=j[5];
    double dj1=dj[0];
    double dj2=dj[1];
    double dj3=dj[2];
    double dj4=dj[3];
    double dj5=dj[4];
    double dj6=dj[5];
    
    M[0][0]=(9*cos(j5))/2 - (9*pow(cos(j2),2))/2 + (9*pow(cos(j4),2))/4 + (9*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*cos(j5))/2 + (27*pow(cos(j2),2)*pow(cos(j3),2))/4 - (9*pow(cos(j2),2)*pow(cos(j4),2))/4 - (9*pow(cos(j2),2)*pow(cos(j5),2))/2 - (9*pow(cos(j4),2)*pow(cos(j5),2))/4 + (9*pow(cos(j2),2)*pow(cos(j3),2)*cos(j5))/2 - (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j4),2))/4 + (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j5),2))/4 + (9*pow(cos(j2),2)*pow(cos(j4),2)*pow(cos(j5),2))/4 + (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j4),2)*pow(cos(j5),2))/4 + (9*cos(j2)*cos(j3)*cos(j4)*sin(j2)*sin(j4))/2 - (9*cos(j2)*sin(j2)*sin(j3)*sin(j4)*sin(j5))/2 + (9*pow(cos(j2),2)*cos(j3)*cos(j4)*sin(j3)*sin(j5))/2 - (9*cos(j2)*cos(j5)*sin(j2)*sin(j3)*sin(j4)*sin(j5))/2 - (9*cos(j2)*cos(j3)*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j4))/2 + (9*pow(cos(j2),2)*cos(j3)*cos(j4)*cos(j5)*sin(j3)*sin(j5))/2 + 27/4;
    
    M[1][0]=(9*cos(j2)*cos(j3)*pow(cos(j4),2)*sin(j3))/4 - (9*cos(j2)*cos(j4)*sin(j5))/4 - (27*cos(j2)*cos(j3)*sin(j3))/4 - (9*cos(j2)*cos(j3)*pow(cos(j5),2)*sin(j3))/4 + (9*cos(j2)*pow(cos(j3),2)*cos(j4)*sin(j5))/2 - (9*cos(j2)*cos(j3)*cos(j5)*sin(j3))/2 - (9*cos(j2)*cos(j4)*cos(j5)*sin(j5))/4 - (9*cos(j4)*sin(j2)*sin(j3)*sin(j4))/4 - (9*cos(j3)*sin(j2)*sin(j4)*sin(j5))/4 - (9*cos(j2)*cos(j3)*pow(cos(j4),2)*pow(cos(j5),2)*sin(j3))/4 - (9*cos(j3)*cos(j5)*sin(j2)*sin(j4)*sin(j5))/4 + (9*cos(j2)*pow(cos(j3),2)*cos(j4)*cos(j5)*sin(j5))/2 + (9*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j3)*sin(j4))/4;
    
    
    M[2][0]=(9*pow(cos(j4),2)*pow(cos(j5),2)*sin(j2))/4 - (9*cos(j5)*sin(j2))/2 - (9*pow(cos(j4),2)*sin(j2))/4 - (9*pow(cos(j5),2)*sin(j2))/4 - (27*sin(j2))/4 - (9*cos(j2)*cos(j3)*cos(j4)*sin(j4))/4 + (9*cos(j2)*sin(j3)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j5)*sin(j3)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j3)*cos(j4)*pow(cos(j5),2)*sin(j4))/4;
    M[3][0]=(9*cos(j2)*sin(j3))/4 - (9*cos(j2)*pow(cos(j5),2)*sin(j3))/4 - (9*sin(j2)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j3)*cos(j4)*sin(j5))/4 - (9*cos(j5)*sin(j2)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j3)*cos(j4)*cos(j5)*sin(j5))/4;
    M[4][0]=(9*cos(j4)*sin(j2))/4 + (9*cos(j2)*cos(j3)*sin(j4))/4 + (9*cos(j4)*cos(j5)*sin(j2))/4 + (9*cos(j2)*cos(j3)*cos(j5)*sin(j4))/4;
    M[5][0]=0;
    M[0][1]=(9*cos(j2)*cos(j3)*pow(cos(j4),2)*sin(j3))/4 - (9*cos(j2)*cos(j4)*sin(j5))/4 - (27*cos(j2)*cos(j3)*sin(j3))/4 - (9*cos(j2)*cos(j3)*pow(cos(j5),2)*sin(j3))/4 + (9*cos(j2)*pow(cos(j3),2)*cos(j4)*sin(j5))/2 - (9*cos(j2)*cos(j3)*cos(j5)*sin(j3))/2 - (9*cos(j2)*cos(j4)*cos(j5)*sin(j5))/4 - (9*cos(j4)*sin(j2)*sin(j3)*sin(j4))/4 - (9*cos(j3)*sin(j2)*sin(j4)*sin(j5))/4 - (9*cos(j2)*cos(j3)*pow(cos(j4),2)*pow(cos(j5),2)*sin(j3))/4 - (9*cos(j3)*cos(j5)*sin(j2)*sin(j4)*sin(j5))/4 + (9*cos(j2)*pow(cos(j3),2)*cos(j4)*cos(j5)*sin(j5))/2 + (9*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j3)*sin(j4))/4;
    M[1][1]=(9*pow(cos(j2),2))/2 - (9*pow(cos(j4),2))/4 - (9*pow(cos(j5),2))/4 + (9*pow(cos(j2),2)*cos(j5))/2 - (27*pow(cos(j2),2)*pow(cos(j3),2))/4 + (9*pow(cos(j2),2)*pow(cos(j4),2))/4 + (9*pow(cos(j2),2)*pow(cos(j5),2))/2 + (9*pow(cos(j4),2)*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*cos(j5))/2 + (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j4),2))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*pow(cos(j4),2)*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j4),2)*pow(cos(j5),2))/4 - (9*cos(j2)*cos(j3)*cos(j4)*sin(j2)*sin(j4))/2 + (9*cos(j2)*sin(j2)*sin(j3)*sin(j4)*sin(j5))/2 - (9*pow(cos(j2),2)*cos(j3)*cos(j4)*sin(j3)*sin(j5))/2 + (9*cos(j2)*cos(j5)*sin(j2)*sin(j3)*sin(j4)*sin(j5))/2 + (9*cos(j2)*cos(j3)*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j4))/2 - (9*pow(cos(j2),2)*cos(j3)*cos(j4)*cos(j5)*sin(j3)*sin(j5))/2 + 9/4;
    M[2][1]=(9*cos(j4)*sin(j3)*sin(j4))/4 + (9*cos(j3)*sin(j4)*sin(j5))/4 - (9*pow(cos(j2),2)*cos(j4)*sin(j3)*sin(j4))/4 - (9*pow(cos(j2),2)*cos(j3)*sin(j4)*sin(j5))/4 - (9*cos(j4)*pow(cos(j5),2)*sin(j3)*sin(j4))/4 + (27*cos(j2)*cos(j3)*sin(j2)*sin(j3))/4 + (9*cos(j2)*cos(j4)*sin(j2)*sin(j5))/4 + (9*cos(j3)*cos(j5)*sin(j4)*sin(j5))/4 + (9*pow(cos(j2),2)*cos(j4)*pow(cos(j5),2)*sin(j3)*sin(j4))/4 + (9*cos(j2)*cos(j3)*cos(j5)*sin(j2)*sin(j3))/2 + (9*cos(j2)*cos(j4)*cos(j5)*sin(j2)*sin(j5))/4 - (9*cos(j2)*cos(j3)*pow(cos(j4),2)*sin(j2)*sin(j3))/4 + (9*cos(j2)*cos(j3)*pow(cos(j5),2)*sin(j2)*sin(j3))/4 - (9*cos(j2)*pow(cos(j3),2)*cos(j4)*sin(j2)*sin(j5))/2 - (9*pow(cos(j2),2)*cos(j3)*cos(j5)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j3)*pow(cos(j4),2)*pow(cos(j5),2)*sin(j2)*sin(j3))/4 - (9*cos(j2)*pow(cos(j3),2)*cos(j4)*cos(j5)*sin(j2)*sin(j5))/2;
    M[3][1]=(9*cos(j3))/4 - (9*pow(cos(j2),2)*cos(j3))/4 - (9*cos(j3)*pow(cos(j4),2))/4 - (9*cos(j3)*pow(cos(j5),2))/4 + (9*pow(cos(j2),2)*cos(j3)*pow(cos(j4),2))/2 + (9*pow(cos(j2),2)*cos(j3)*pow(cos(j5),2))/4 + (9*cos(j3)*pow(cos(j4),2)*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*cos(j4)*sin(j3)*sin(j5))/4 - (9*pow(cos(j2),2)*cos(j3)*pow(cos(j4),2)*pow(cos(j5),2))/2 - (9*cos(j2)*cos(j4)*sin(j2)*sin(j4))/4 - (9*cos(j2)*pow(cos(j3),2)*cos(j4)*sin(j2)*sin(j4))/4 + (9*cos(j2)*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j4))/4 - (9*pow(cos(j2),2)*cos(j4)*cos(j5)*sin(j3)*sin(j5))/4 + (9*cos(j2)*pow(cos(j3),2)*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j4))/4 + (9*cos(j2)*cos(j3)*sin(j2)*sin(j3)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j3)*cos(j5)*sin(j2)*sin(j3)*sin(j4)*sin(j5))/4;
    M[4][1]=(9*pow(cos(j2),2)*sin(j3)*sin(j4))/4 - (9*sin(j3)*sin(j4))/4 + (9*pow(cos(j5),2)*sin(j3)*sin(j4))/4 - (9*cos(j2)*sin(j2)*sin(j5))/4 + (9*cos(j2)*pow(cos(j3),2)*sin(j2)*sin(j5))/4 - (9*pow(cos(j2),2)*cos(j5)*sin(j3)*sin(j4))/4 - (9*pow(cos(j2),2)*pow(cos(j5),2)*sin(j3)*sin(j4))/2 - (9*cos(j2)*cos(j5)*sin(j2)*sin(j5))/2 + (9*cos(j2)*cos(j3)*cos(j4)*sin(j2)*sin(j3))/4 - (9*cos(j3)*cos(j4)*cos(j5)*sin(j4)*sin(j5))/4 + (9*cos(j2)*pow(cos(j3),2)*cos(j5)*sin(j2)*sin(j5))/4 + (9*cos(j2)*pow(cos(j4),2)*cos(j5)*sin(j2)*sin(j5))/4 + (9*cos(j2)*pow(cos(j3),2)*pow(cos(j4),2)*cos(j5)*sin(j2)*sin(j5))/4 - (9*cos(j2)*cos(j3)*cos(j4)*cos(j5)*sin(j2)*sin(j3))/4 - (9*cos(j2)*cos(j3)*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j3))/2 + (9*pow(cos(j2),2)*cos(j3)*cos(j4)*cos(j5)*sin(j4)*sin(j5))/2;
    M[5][1]=0;
    M[0][2]=(9*pow(cos(j4),2)*pow(cos(j5),2)*sin(j2))/4 - (9*cos(j5)*sin(j2))/2 - (9*pow(cos(j4),2)*sin(j2))/4 - (9*pow(cos(j5),2)*sin(j2))/4 - (27*sin(j2))/4 - (9*cos(j2)*cos(j3)*cos(j4)*sin(j4))/4 + (9*cos(j2)*sin(j3)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j5)*sin(j3)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j3)*cos(j4)*pow(cos(j5),2)*sin(j4))/4;
    M[1][2]=(9*cos(j4)*sin(j3)*sin(j4))/4 + (9*cos(j3)*sin(j4)*sin(j5))/4 - (9*pow(cos(j2),2)*cos(j4)*sin(j3)*sin(j4))/4 - (9*pow(cos(j2),2)*cos(j3)*sin(j4)*sin(j5))/4 - (9*cos(j4)*pow(cos(j5),2)*sin(j3)*sin(j4))/4 + (27*cos(j2)*cos(j3)*sin(j2)*sin(j3))/4 + (9*cos(j2)*cos(j4)*sin(j2)*sin(j5))/4 + (9*cos(j3)*cos(j5)*sin(j4)*sin(j5))/4 + (9*pow(cos(j2),2)*cos(j4)*pow(cos(j5),2)*sin(j3)*sin(j4))/4 + (9*cos(j2)*cos(j3)*cos(j5)*sin(j2)*sin(j3))/2 + (9*cos(j2)*cos(j4)*cos(j5)*sin(j2)*sin(j5))/4 - (9*cos(j2)*cos(j3)*pow(cos(j4),2)*sin(j2)*sin(j3))/4 + (9*cos(j2)*cos(j3)*pow(cos(j5),2)*sin(j2)*sin(j3))/4 - (9*cos(j2)*pow(cos(j3),2)*cos(j4)*sin(j2)*sin(j5))/2 - (9*pow(cos(j2),2)*cos(j3)*cos(j5)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j3)*pow(cos(j4),2)*pow(cos(j5),2)*sin(j2)*sin(j3))/4 - (9*cos(j2)*pow(cos(j3),2)*cos(j4)*cos(j5)*sin(j2)*sin(j5))/2;
    M[2][2]=(9*cos(j5))/2 + (9*pow(cos(j4),2))/4 + (9*pow(cos(j5),2))/4 - (27*pow(cos(j2),2)*pow(cos(j3),2))/4 - (9*pow(cos(j2),2)*pow(cos(j4),2))/4 - (9*pow(cos(j4),2)*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*cos(j5))/2 + (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j4),2))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j5),2))/4 + (9*pow(cos(j2),2)*pow(cos(j4),2)*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j4),2)*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*cos(j3)*cos(j4)*sin(j3)*sin(j5))/2 - (9*pow(cos(j2),2)*cos(j3)*cos(j4)*cos(j5)*sin(j3)*sin(j5))/2 + 27/4;
    
    M[3][2]=(9*sin(j4)*sin(j5))/4 + (9*cos(j5)*sin(j4)*sin(j5))/4 - (9*cos(j2)*pow(cos(j4),2)*sin(j2)*sin(j3))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*sin(j4)*sin(j5))/4 + (9*cos(j2)*pow(cos(j4),2)*pow(cos(j5),2)*sin(j2)*sin(j3))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*cos(j5)*sin(j4)*sin(j5))/4 - (9*cos(j2)*cos(j3)*cos(j4)*sin(j2)*sin(j5))/4 - (9*pow(cos(j2),2)*cos(j3)*cos(j4)*sin(j3)*sin(j4))/4 + (9*pow(cos(j2),2)*cos(j3)*cos(j4)*pow(cos(j5),2)*sin(j3)*sin(j4))/4 - (9*cos(j2)*cos(j3)*cos(j4)*cos(j5)*sin(j2)*sin(j5))/4;
    M[4][2]=(9*pow(cos(j2),2)*cos(j4))/4 - (9*cos(j4))/4 - (9*cos(j4)*cos(j5))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*cos(j4))/4 - (9*pow(cos(j2),2)*cos(j4)*pow(cos(j5),2))/4 + (9*pow(cos(j2),2)*cos(j3)*sin(j3)*sin(j5))/4 + (9*pow(cos(j2),2)*pow(cos(j3),2)*cos(j4)*cos(j5))/4 + (9*pow(cos(j2),2)*pow(cos(j3),2)*cos(j4)*pow(cos(j5),2))/2 - (9*cos(j2)*cos(j3)*cos(j5)*sin(j2)*sin(j4))/4 - (9*cos(j2)*cos(j3)*pow(cos(j5),2)*sin(j2)*sin(j4))/4 + (9*pow(cos(j2),2)*cos(j3)*cos(j5)*sin(j3)*sin(j5))/4 + (9*pow(cos(j2),2)*cos(j3)*pow(cos(j4),2)*cos(j5)*sin(j3)*sin(j5))/4 - (9*cos(j2)*cos(j4)*cos(j5)*sin(j2)*sin(j3)*sin(j4)*sin(j5))/4;
    M[5][2]=0;
    M[0][3]=(9*cos(j2)*sin(j3))/4 - (9*cos(j2)*pow(cos(j5),2)*sin(j3))/4 - (9*sin(j2)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j3)*cos(j4)*sin(j5))/4 - (9*cos(j5)*sin(j2)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j3)*cos(j4)*cos(j5)*sin(j5))/4;
    M[1][3]=(9*cos(j3))/4 - (9*pow(cos(j2),2)*cos(j3))/4 - (9*cos(j3)*pow(cos(j4),2))/4 - (9*cos(j3)*pow(cos(j5),2))/4 + (9*pow(cos(j2),2)*cos(j3)*pow(cos(j4),2))/2 + (9*pow(cos(j2),2)*cos(j3)*pow(cos(j5),2))/4 + (9*cos(j3)*pow(cos(j4),2)*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*cos(j4)*sin(j3)*sin(j5))/4 - (9*pow(cos(j2),2)*cos(j3)*pow(cos(j4),2)*pow(cos(j5),2))/2 - (9*cos(j2)*cos(j4)*sin(j2)*sin(j4))/4 - (9*cos(j2)*pow(cos(j3),2)*cos(j4)*sin(j2)*sin(j4))/4 + (9*cos(j2)*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j4))/4 - (9*pow(cos(j2),2)*cos(j4)*cos(j5)*sin(j3)*sin(j5))/4 + (9*cos(j2)*pow(cos(j3),2)*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j4))/4 + (9*cos(j2)*cos(j3)*sin(j2)*sin(j3)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j3)*cos(j5)*sin(j2)*sin(j3)*sin(j4)*sin(j5))/4;
    M[2][3]=(9*sin(j4)*sin(j5))/4 + (9*cos(j5)*sin(j4)*sin(j5))/4 - (9*cos(j2)*pow(cos(j4),2)*sin(j2)*sin(j3))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*sin(j4)*sin(j5))/4 + (9*cos(j2)*pow(cos(j4),2)*pow(cos(j5),2)*sin(j2)*sin(j3))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*cos(j5)*sin(j4)*sin(j5))/4 - (9*cos(j2)*cos(j3)*cos(j4)*sin(j2)*sin(j5))/4 - (9*pow(cos(j2),2)*cos(j3)*cos(j4)*sin(j3)*sin(j4))/4 + (9*pow(cos(j2),2)*cos(j3)*cos(j4)*pow(cos(j5),2)*sin(j3)*sin(j4))/4 - (9*cos(j2)*cos(j3)*cos(j4)*cos(j5)*sin(j2)*sin(j5))/4;
    M[3][3]=(9*pow(cos(j2),2)*pow(cos(j4),2))/4 - (9*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2))/4 - (9*pow(cos(j4),2))/4 + (9*pow(cos(j4),2)*pow(cos(j5),2))/4 + (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j4),2))/4 + (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*pow(cos(j4),2)*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j4),2)*pow(cos(j5),2))/4 - (9*cos(j2)*cos(j3)*cos(j4)*sin(j2)*sin(j4))/2 + (9*cos(j2)*cos(j3)*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j4))/2 + 9/4;
    M[4][3]=(9*pow(cos(j2),2)*cos(j3)*sin(j3)*sin(j4))/4 + (9*cos(j2)*cos(j4)*sin(j2)*sin(j3))/4 - (9*cos(j4)*cos(j5)*sin(j4)*sin(j5))/4 - (9*pow(cos(j2),2)*cos(j3)*pow(cos(j5),2)*sin(j3)*sin(j4))/4 - (9*cos(j2)*cos(j3)*cos(j5)*sin(j2)*sin(j5))/4 - (9*cos(j2)*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j3))/4 + (9*pow(cos(j2),2)*cos(j4)*cos(j5)*sin(j4)*sin(j5))/4 + (9*pow(cos(j2),2)*pow(cos(j3),2)*cos(j4)*cos(j5)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j3)*pow(cos(j4),2)*cos(j5)*sin(j2)*sin(j5))/2;
    M[5][3]=0;
    M[0][4]=(9*cos(j4)*sin(j2))/4 + (9*cos(j2)*cos(j3)*sin(j4))/4 + (9*cos(j4)*cos(j5)*sin(j2))/4 + (9*cos(j2)*cos(j3)*cos(j5)*sin(j4))/4;
    M[1][4]=(9*pow(cos(j2),2)*sin(j3)*sin(j4))/4 - (9*sin(j3)*sin(j4))/4 + (9*pow(cos(j5),2)*sin(j3)*sin(j4))/4 - (9*cos(j2)*sin(j2)*sin(j5))/4 + (9*cos(j2)*pow(cos(j3),2)*sin(j2)*sin(j5))/4 - (9*pow(cos(j2),2)*cos(j5)*sin(j3)*sin(j4))/4 - (9*pow(cos(j2),2)*pow(cos(j5),2)*sin(j3)*sin(j4))/2 - (9*cos(j2)*cos(j5)*sin(j2)*sin(j5))/2 + (9*cos(j2)*cos(j3)*cos(j4)*sin(j2)*sin(j3))/4 - (9*cos(j3)*cos(j4)*cos(j5)*sin(j4)*sin(j5))/4 + (9*cos(j2)*pow(cos(j3),2)*cos(j5)*sin(j2)*sin(j5))/4 + (9*cos(j2)*pow(cos(j4),2)*cos(j5)*sin(j2)*sin(j5))/4 + (9*cos(j2)*pow(cos(j3),2)*pow(cos(j4),2)*cos(j5)*sin(j2)*sin(j5))/4 - (9*cos(j2)*cos(j3)*cos(j4)*cos(j5)*sin(j2)*sin(j3))/4 - (9*cos(j2)*cos(j3)*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j3))/2 + (9*pow(cos(j2),2)*cos(j3)*cos(j4)*cos(j5)*sin(j4)*sin(j5))/2;
    M[2][4]=(9*pow(cos(j2),2)*cos(j4))/4 - (9*cos(j4))/4 - (9*cos(j4)*cos(j5))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*cos(j4))/4 - (9*pow(cos(j2),2)*cos(j4)*pow(cos(j5),2))/4 + (9*pow(cos(j2),2)*cos(j3)*sin(j3)*sin(j5))/4 + (9*pow(cos(j2),2)*pow(cos(j3),2)*cos(j4)*cos(j5))/4 + (9*pow(cos(j2),2)*pow(cos(j3),2)*cos(j4)*pow(cos(j5),2))/2 - (9*cos(j2)*cos(j3)*cos(j5)*sin(j2)*sin(j4))/4 - (9*cos(j2)*cos(j3)*pow(cos(j5),2)*sin(j2)*sin(j4))/4 + (9*pow(cos(j2),2)*cos(j3)*cos(j5)*sin(j3)*sin(j5))/4 + (9*pow(cos(j2),2)*cos(j3)*pow(cos(j4),2)*cos(j5)*sin(j3)*sin(j5))/4 - (9*cos(j2)*cos(j4)*cos(j5)*sin(j2)*sin(j3)*sin(j4)*sin(j5))/4;
    M[3][4]=(9*pow(cos(j2),2)*cos(j3)*sin(j3)*sin(j4))/4 + (9*cos(j2)*cos(j4)*sin(j2)*sin(j3))/4 - (9*cos(j4)*cos(j5)*sin(j4)*sin(j5))/4 - (9*pow(cos(j2),2)*cos(j3)*pow(cos(j5),2)*sin(j3)*sin(j4))/4 - (9*cos(j2)*cos(j3)*cos(j5)*sin(j2)*sin(j5))/4 - (9*cos(j2)*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j3))/4 + (9*pow(cos(j2),2)*cos(j4)*cos(j5)*sin(j4)*sin(j5))/4 + (9*pow(cos(j2),2)*pow(cos(j3),2)*cos(j4)*cos(j5)*sin(j4)*sin(j5))/4 + (9*cos(j2)*cos(j3)*pow(cos(j4),2)*cos(j5)*sin(j2)*sin(j5))/2;
    M[4][4]=(9*pow(cos(j2),2)*pow(cos(j3),2))/4 - (9*pow(cos(j5),2))/4 - (9*pow(cos(j2),2))/4 + (9*pow(cos(j2),2)*pow(cos(j5),2))/2 + (9*pow(cos(j4),2)*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*pow(cos(j4),2)*pow(cos(j5),2))/4 - (9*pow(cos(j2),2)*pow(cos(j3),2)*pow(cos(j4),2)*pow(cos(j5),2))/4 + (9*cos(j2)*cos(j5)*sin(j2)*sin(j3)*sin(j4)*sin(j5))/2 + (9*cos(j2)*cos(j3)*cos(j4)*pow(cos(j5),2)*sin(j2)*sin(j4))/2 - (9*pow(cos(j2),2)*cos(j3)*cos(j4)*cos(j5)*sin(j3)*sin(j5))/2 + 9/4;
    M[5][4]=0;
    M[0][5]=0;
    M[1][5]=0;
    M[2][5]=0;
    M[3][5]=0;
    M[4][5]=0;
    M[5][5]=0;
    
    return M;
  }
  catch (const std::out_of_range& oor) {
    std::cerr << "Out of range while unpacking angles: " << oor.what() << '\n';
  }
  return M;
}
