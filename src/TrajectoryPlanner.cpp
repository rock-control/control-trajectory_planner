#include "TrajectoryPlanner.hpp"
#include <stdio.h>
#include <stdlib.h>

namespace trajectory_planner
{

    TrajectoryPlanner::TrajectoryPlanner(TrajectoryPlannerConfig _trajectoryPlannerConfig):
                                         mTrajectoryPlannerConfig(_trajectoryPlannerConfig)
    {

        switch(mTrajectoryPlannerConfig.mInterpolatorType)
        {
            case BEZIER:
            {
                mInterpolator = boost::shared_ptr<AbstractInterpolator> (new BezierCurve());
                break;
            }
            
	   
            default:
            {
                /*LOG_ERROR("This TrajectoryInterpolator is not available");
                throw new std::runtime_error("This TrajectoryInterpolator is not available");*/
		std::cout<<"This TrajectoryInterpolator is not available"<<std::endl;
            }
        }

    }

    TrajectoryPlanner::~TrajectoryPlanner()
    {
        mInterpolator.reset();
    }



}
