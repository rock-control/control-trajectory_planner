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
#include<utils/MyMath.hpp>
#include<utils/ReadJointLimits.hpp>

using namespace trajectory_planner;
using namespace std;



int main(int argc, char* argv[])
{
  base:: JointLimits limits=utils::initFromURDF("/home/dfki.uni-bremen.de/rmenon/software/rock_traj/bundles/kuka_lbr/data/urdf/kuka_lbr_left_arm.urdf");
  base::JointsTrajectory jointsTrajOut;
  
  jointsTrajOut.resize(limits.size(),10);
  jointsTrajOut.names=limits.names;
  
  int noPoints;
  cout<<"Enter number of points "<<endl;
  cin>>noPoints;
  noPoints = min(noPoints,10);
  noPoints = max(noPoints, 3);
  
  jointsTrajOut.resize(limits.size(), noPoints);
  for(unsigned i=0; i<jointsTrajOut.getNumberOfJoints();i++)
  {
    for(unsigned j=0; j<noPoints;j++)
    {
      jointsTrajOut.elements[i][j].position=utils::getRandom(limits.elements[i].min.position, limits.elements[i].max.position);
    }
  }
  
  joints_trajectory jointsTrajCurve;
  base::JointsTrajectory trajIn, trajOut;
  
  trajIn.resize(limits.size());
  trajIn.names=limits.names;
  
  trajOut.resize(limits.size());
  trajOut.names =limits.names;
  
  jointsTrajCurve.initialise_data(limits,0.01,5,75, 0);
  trajOut=jointsTrajCurve.calc_joint_traj_bezier(trajIn);
  
  return 0;
}