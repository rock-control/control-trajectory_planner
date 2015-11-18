#include<iostream>
#include<fstream>
#include<stdio.h>

//#include <boost/test/unit_test.hpp>
#include "trajectory_planner/joints_traj/joints_trajectory.hpp"
#include "trajectory_planner/bezier_curve/BezierCurve.hpp"
//#include <base/logging/logging_printf_style.h>
#include <Eigen/Core>
#include <boost/concept_check.hpp>
#include <Eigen/Dense>

using namespace trajectory_planner;




int main(int argc, char* argv[])
{
  
  
  
  base::JointLimits joints_limits;
  base::JointTrajectory sample;
  sample.resize(3);
  base::JointLimitRange limit;

  int ndata=0;
  float data1, data2, data3,data4;
  base::VectorXd j1(100),j2(100),j3(100);
  std::cout<<argc<<std::endl;
  
  FILE * fp_in=fopen("joints_traj.txt", "r") ;        // Opening the input file to read
  
  if (fp_in == NULL)
  {
    std::cout << "The inputdata file cannot be opened to read" << std::endl;
  }
  do
  {
    //fscanf(fp_in, "%f %f %f \n", &data1, &data2, &data3);
    fscanf(fp_in, "%f %f %f\n", &data1, &data2,&data3);
    //cout<<data1<<data2<<data3<<endl;
    j1(ndata)=data1;
    j2(ndata)=data2;
    j3(ndata)=data3;
    ndata = ndata + 1;
  }while (!feof(fp_in));
  
  base::JointsTrajectory joints_in;
  
  joints_in.resize(3,ndata);
  std::cout<<"ndata= "<<ndata<<std::endl;
  for(int i=0;i<ndata;i++)
  {
    joints_in.elements[0][i].position=j1(i);
    joints_in.elements[1][i].position=j2(i);
    joints_in.elements[2][i].position=j3(i);
  }
  
  FILE * fp_in1=fopen("joints_limits.txt", "r") ;        // Opening the input file to read
  
  if (fp_in1 == NULL)
  {
    std::cout << "The inputdata file cannot be opened to read" << std::endl;
    std::cout << std::endl;
  }
  do
  {
    fscanf(fp_in1, "%f %f %f %f\n", &data1, &data2, &data3, &data4);
    //cout<<data1<<data2<<data3<<endl;
    limit.min.position=data1;
    limit.max.position=data2;
    limit.min.speed=data3;
    limit.max.speed=data4;
    joints_limits.elements.push_back(limit);
    ndata = ndata + 1;
  }while (!feof(fp_in1));	
  
  std::cout<<"No of joints= "<<joints_in.getNumberOfJoints()<<std::endl;
  std::cout<<"No of timesteps= "<<joints_in.elements[0].size()<<std::endl;
  std::cout<<"limits= "<<joints_limits.size()<<std::endl;
  for(int i=0; i<joints_in.elements[0].size(); i++)
  {
    for(int j=0; j<joints_in.getNumberOfJoints();j++)
    {
      std::cout<<joints_in.elements[j][i].position<<"   ";
    }
    std::cout<<" "<<std::endl;
  }
  
  for(int i=0; i<1; i++)
  {
    std::cout<<"max speed= "<<joints_limits.elements[i].max.speed<<std::endl;
  }
  
  trajectory_planner::joints_trajectory traj(joints_in, joints_limits, 0.01);
  traj.calc_joint_traj_bezier(joints_in, joints_limits, 0.01, 10, 0,100);
  return 0;
}