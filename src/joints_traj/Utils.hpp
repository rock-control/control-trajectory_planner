#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <stdlib.h>     /* srand, rand */
#include <time.h>   
#include<math.h>
#include<iostream>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <base/JointLimits.hpp>
#include <base/commands/Joints.hpp>

namespace utils{
  
  double getRandom(double min, double max);
  base::JointLimits initFromURDF(const std::string& filepath);
  base::JointLimits initFromYaml(const std::string&  filepath);
  
}

#endif