#ifndef _TRAJECTORYPLANNERCONFIG_HPP_
#define _TRAJECTORYPLANNERCONFIG_HPP_

#include <string>

namespace trajectory_planner
{

enum InterpolatorType
{
    BEZIER
};

/**
* @struct ConstraintStruct
* @brief This struct contains constraints parameter to be imposed on the interpolator.
*/
struct ConstraintStruct
{

};


struct TrajectoryPlannerConfig
{
    TrajectoryPlannerConfig() : mInterpolatorType(BEZIER){}

    // interpolator type: currently 1 type of interpolator is available
    enum InterpolatorType mInterpolatorType;
};

struct TrajectoryPlannerStatus
{
    enum StatusCode
    {
        SOLUTION_FOUND
    }statuscode;

};

} // end namespace trajectory_planner

#endif
