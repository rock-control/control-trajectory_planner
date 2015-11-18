#ifndef _JOINTS_TRAJECTORY_HPP_
#define _JOINTS_TRAJECTORY_HPP_
#define EIGEN_MATRIXBASE_PLUGIN "MatrixBaseAddons.h"


#include <iostream>
#include<fstream>
#include<stdio.h>
#include <boost/shared_ptr.hpp>
#include <base/Eigen.hpp>
#include "boost/multi_array.hpp"
#include <cassert>
#include <Eigen/Dense>
#include <math.h>
#include <eigen3/Eigen/src/Core/EigenBase.h>
#include "../bezier_curve/BezierCurve.hpp"
#include <base/JointLimits.hpp>
#include <trajectory_planner/AbstractInterpolator.hpp>
#include<base/JointsTrajectory.hpp>

namespace trajectory_planner
{
  
  typedef boost::multi_array<double, 3> array_3D;
  typedef array_3D::index index1;
  
   
  class joints_trajectory 
  {
  public:
    int m_const, m_dim,hasTime, m_noJts, m_noTrajPts, m_order;
    double m_speedLevel,m_period, m_timeDesired, m_accMax, m_accMin, m_velMax, m_velMin;
    base::JointLimits m_jointsLimits;
    BezierCurve m_jointsPos_curve;
    BezierCurve m_jointsVel_curve;
    BezierCurve m_jointsAcc_curve;
    base::VectorXd m_timeSteps, m_numIntervals;
    
    joints_trajectory();
    joints_trajectory(base::JointsTrajectory, base::JointLimits, double);
    joints_trajectory(base::JointsTrajectory, base::JointLimits, double, double);
    ~joints_trajectory();
    
    base::JointsTrajectory calc_joint_traj_bezier(const base::JointsTrajectory& joints_traj_in, const base::JointLimits& limits,  double period=0.01, double acc_max=10.0, int end_condition=0, double time_desired=-1);
    void initialise_data(const base::JointsTrajectory& joints_traj_in, const base::JointLimits& limits,  double period=0.01);
    void map_jointsTraj2baseMatrix(const base::JointsTrajectory& joints_traj_in);
    void map_baseMatrix2jointsTraj(base::JointsTrajectory&);
    void calc_traj_time();
    double synchronise_time(const base::MatrixXd& bCurve1, const base::MatrixXd& bCurve2, double time_interval);
    void write_data(const base::MatrixXd& bCurve, const char*filename);
    int check_time_reduction(base::VectorXi& newPoints);
    int check_position(base::VectorXi& newPoints);
   // void add_points(const base::VectorXi& newPoints);
    void correct_knotacc();
    //void check_position_limits();
    
  };
}
  
#endif