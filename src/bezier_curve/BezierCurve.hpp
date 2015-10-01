#ifndef _BEZIERCURVE_HPP_
#define _BEZIERCURVE_HPP_

#include <iostream>

#include <boost/shared_ptr.hpp>


#include <trajectory_planner/AbstractInterpolator.hpp>


namespace trajectory_planner
{
    class BezierCurve : public AbstractInterpolator
    {
        public:
            BezierCurve();
            ~BezierCurve();

    };

} // end namespace trajectory_planner

#endif // _BEZIERCURVE_HPP_
