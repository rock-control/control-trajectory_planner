#rock_testsuite(testb test.cpp
#  DEPS trajectory_planner)

#include_directories(${PROJECT_SOURCE_DIR}/src)

find_package(PkgConfig)
pkg_check_modules(BASE REQUIRED base-lib)
INCLUDE_DIRECTORIES(${BASE_INCLUDE_DIRS})
LINK_DIRECTORIES(${BASE_LIBRARY_DIRS})

rock_executable(test_interpolator test.cpp DEPS trajectory_planner)

target_link_libraries(test_interpolator base)
