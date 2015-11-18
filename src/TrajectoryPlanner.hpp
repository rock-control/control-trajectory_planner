#ifndef _TRAJECTORYPLANNER_HPP_
#define _TRAJECTORYPLANNER_HPP_

#include <iostream>

#include <boost/shared_ptr.hpp>

#include <base/JointsTrajectory.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/logging/logging_printf_style.h>

#include "AbstractInterpolator.hpp"
#include "TrajectoryPlannerConfig.hpp"
#include <trajectory_planner/bezier_curve/BezierCurve.hpp>
#include <trajectory_planner/joints_traj/joints_trajectory.hpp>

namespace trajectory_planner
{
    class TrajectoryPlanner
    {
        public:
            TrajectoryPlanner(TrajectoryPlannerConfig _trajectoryPlannerConfig);
            ~TrajectoryPlanner();

            bool interpolateCartesianSpace( const base::samples::RigidBodyState &target_pose,
                                            base::JointsTrajectory &solution,
                                            TrajectoryPlannerStatus &solver_status);

            void interpolateJointSpace( const base::JointsTrajectory &target_trajectory,
                                        base::JointsTrajectory &solution,
                                        TrajectoryPlannerStatus &solver_status);

        protected:
            boost::shared_ptr<AbstractInterpolator> mInterpolator;


        private:
            TrajectoryPlannerConfig mTrajectoryPlannerConfig;


    };

} // end namespace trajectory_planner

#endif // _TRAJECTORYPLANNER_HPP_
