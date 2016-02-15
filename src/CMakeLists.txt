rock_library(trajectory_planner
    SOURCES AbstractInterpolator.cpp
            TrajectoryPlanner.cpp
            bezier_curve/BezierCurve.cpp
            joints_traj/joints_trajectory.cpp
            joints_traj/Utils.cpp
            
    HEADERS AbstractInterpolator.hpp
            TrajectoryPlanner.hpp
            TrajectoryPlannerConfig.hpp
            bezier_curve/BezierCurve.hpp
            joints_traj/joints_trajectory.hpp
            joints_traj/Utils.hpp
            

    DEPS_PKGCONFIG eigen3 base-types kdl_parser urdfdom urdfdom_headers yaml-cpp
)

#rock_executable(test_dummy test_interpol.cpp DEPS trajectory_planner)


