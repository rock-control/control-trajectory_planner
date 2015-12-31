#ifndef _BEZIERCURVE_HPP_
#define _BEZIERCURVE_HPP_

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <base/Eigen.hpp>
#include "boost/multi_array.hpp"
#include <cassert>
#include <Eigen/Dense>
#include <math.h>
#include <trajectory_planner/AbstractInterpolator.hpp>

namespace base
{
  typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> 
  MatrixXi;
  
  typedef Eigen::Matrix<int, Eigen::Dynamic, 1, Eigen::DontAlign> 
  VectorXi;
}

namespace trajectory_planner
{

    typedef boost::multi_array<double, 3> array_3D;
    typedef array_3D::index index1;
    
    void check_bezier_curve(base::MatrixXd & pts);
    unsigned factorial(unsigned);
    
    
    class BezierCurve : public AbstractInterpolator
    {
        public:
            BezierCurve();
	    BezierCurve(int, int);
	    BezierCurve(int, int, int);
	    BezierCurve(int, int, int, int);
	    ~BezierCurve();
	    
	    
	   // Eigen::MatrixXd m_deBoorPoints, m_trajPoints;
	    int m_const, m_condition, m_noPts, m_dim, m_numIntervals;
	    array_3D m_bCoeffs, x11;
	    base::MatrixXd m_trajPoints,m_deBoorPoints,m_bCurve;
	    base::VectorXd m_numIntervals_vec;
	    //base::MatrixXd m_bCoeffStart, m_bCoeffEnd;
	   // void initialise_data();
	    
            base::MatrixXd calc_bezier_curve(const base::MatrixXd & pts, const int& num_int);
	    
	    //Checks the no of points and takes action accordingly
	    friend void check_bezier_curve(base::MatrixXd & pts);

            //computes the spline point for a given bezier coefficient. this is an internal func called in comp_bezier_pt
            base::MatrixXd comp_cubic_spline_pt(const base::MatrixXd& b_coeff_set, double t);
	    base::MatrixXd comp_spline_pt(const base::MatrixXd& b_coeff_set, double t);
            //computes the dim dimensional bezier point for a given index and the percentage of time between the 2 control points for that portion of the spline
            base::MatrixXd comp_bezier_pt(const int& b_index, double t);

            //computes the enitre bezier curve points
	    void comp_bezier_curve();
            base::MatrixXd comp_bezier_curve(int num_intervals);
	    base::MatrixXd comp_bezier_curve(base::VectorXd& num_intervals);
	    void comp_bezier_curve(const array_3D& bCoeffs, base::VectorXd& num_intervals,base::MatrixXd& b_curve, int pt);
	    base::MatrixXd comp_bezier_curve(base::VectorXd& num_intervals, base::MatrixXd bCoeffStart_pos, base::MatrixXd bCoeffEnd_pos, int curveType);
	    void comp_bezier_curve_deriv(const base::MatrixXd& bezierCurve, int interval, base::MatrixXd& bezierCurve_deriv);
	   
	    void comp_deBoor_controlpts();
	    void read_input_data(const base::MatrixXd & pts_in, const base::VectorXd num_int, int condition);
	    void initialise_data(int noTrajPts, int dim, double period, double order, double end_condition);
	    void read_input_data(const base::MatrixXd & pts_in, int num_int, int condition);

	    void comp_bezier_coeff();
            array_3D comp_bezier_coeff_deriv(array_3D bCoeffs);
	    base::MatrixXd comp_bezier_coeff_start(array_3D bCoeffs, base::MatrixXd bCoeff_extreme, int curveType);
	    base::MatrixXd comp_bezier_coeff_end(array_3D bCoeffs, base::MatrixXd bCoeff_extreme, int curveType);
	    void add_points(const base::VectorXi& newPoints);
           
	   

    };

} // end namespace trajectory_planner

#endif // _BEZIERCURVE_HPP_
