rock_library(trajectory_planner
    SOURCES AbstractInterpolator.cpp
            TrajectoryPlanner.cpp
            bezier_curve/BezierCurve.cpp
            joints_traj/joints_trajectory.cpp
            
    HEADERS AbstractInterpolator.hpp
            TrajectoryPlanner.hpp
            TrajectoryPlannerConfig.hpp
            bezier_curve/BezierCurve.hpp
            joints_traj/joints_trajectory.hpp
            

    DEPS_PKGCONFIG eigen3 base-types utils
)

#rock_executable(test_dummy test_interpol.cpp DEPS trajectory_planner)


