rock_library(trajectory_planner
    SOURCES AbstractInterpolator.cpp
            TrajectoryPlanner.cpp
            bezier_curve/BezierCurve.cpp
    HEADERS AbstractInterpolator.hpp
            TrajectoryPlanner.hpp
            TrajectoryPlannerConfig.hpp
            bezier_curve/BezierCurve.hpp
    DEPS_PKGCONFIG base-types
)


