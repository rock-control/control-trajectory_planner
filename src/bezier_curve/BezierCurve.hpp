#ifndef _BEZIERCURVE_HPP_
#define _BEZIERCURVE_HPP_

#include <iostream>

#include <boost/shared_ptr.hpp>
#include <base/Eigen.hpp>
#include "boost/multi_array.hpp"
#include <cassert>
#include <Eigen/Dense>
#include <iostream>
#include <math.h>

#include <trajectory_planner/AbstractInterpolator.hpp>


namespace trajectory_planner
{

    typedef boost::multi_array<double, 3> array_3D;
    typedef array_3D::index index1;

    class BezierCurve : public AbstractInterpolator
    {
        public:
            BezierCurve();
            ~BezierCurve();

            base::MatrixXd calc_bezier_curve(const base::MatrixXd & pts, const int& num_int);
	    
	    //Checks the no of points and takes action accordingly
	    void check_bezier_curve(base::MatrixXd & pts);

            //computes the spline point for a given bezier coefficient. this is an internal func called in comp_bezier_pt
            template <typename Derived>
            Eigen::MatrixXd comp_cubic_spline_pt(const Eigen::EigenBase< Derived >& b_coeff, double t);

            //computes the dim dimensional bezier point for a given index and the percentage of time between the 2 control points for that portion of the spline
            Eigen::MatrixXd comp_bezier_pt(array_3D b_coeffs, const int& b_index, double t);

            //computes the enitre bezier curve points

            Eigen::MatrixXd comp_bezier_curve(array_3D b_coeffs, const int& num_intervals);

            Eigen::MatrixXd comp_deBoor_controlpts(const base::MatrixXd & pts, const int& condition);

            template <typename Derived>
            array_3D comp_bezier_coeff(const Eigen::EigenBase< Derived > & pts, const Eigen::EigenBase< Derived >& d_points);





            int no_pts, dim, condition, num_intervals;

    };

} // end namespace trajectory_planner

#endif // _BEZIERCURVE_HPP_
