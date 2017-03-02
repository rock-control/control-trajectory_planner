#include"Utils.hpp"
#include <base/Logging.hpp>
#include <boost/iterator/iterator_concepts.hpp>
using namespace std;
using namespace utils;

namespace utils{
  
  base::JointLimits initFromURDF(const std::string& filepath)
  {
    std::ifstream file(filepath.c_str());
    
    if (!file.is_open())
    {  
      std::cout<<"cannot open the urdf file" <<std::endl; 
     
    }
    std::string xml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(xml);
    
    std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it;
    base::JointLimits limits;
    for (it=urdf_model->joints_.begin(); it!=urdf_model->joints_.end(); ++it)
    {
      boost::shared_ptr<urdf::Joint> joint = it->second;
      base::JointLimitRange range;
      
      if(joint->type != urdf::Joint::FIXED && !joint->mimic){
	if(joint->limits)
	{
	  if (joint->type == urdf::Joint::CONTINUOUS) 
	  {
	    range.max.position = 3.14;
	    range.min.position = -3.14;
	  } 
	  else 
	  {
	    range.max.position = joint->limits->upper;
	    range.min.position = joint->limits->lower;
	  }
	  range.max.speed = joint->limits->velocity;
	  range.min.speed = -joint->limits->velocity;
	  range.max.effort = joint->limits->effort;
	  range.min.effort = -joint->limits->effort;
	  limits.names.push_back(it->first);
	  limits.elements.push_back(range);
	}
      }
    }
    
    return limits;
  }
  
  base::JointLimits initFromYaml(const std::string&  filepath)
  {
    
    std::ifstream file(filepath.c_str());
    YAML::Parser parser(file);
    YAML::Node doc;
    parser.GetNextDocument(doc);
    
    try{
      doc["limits"];
    }
    catch(std::exception e){
      std::stringstream ss;
      ss<<"Yaml parsing error: File "<<filepath<<" either doesn't exist or is not a valid joint limits file"<<std::endl;
      throw std::invalid_argument(ss.str());
    }
    
    const YAML::Node &names_node = doc["limits"]["names"];
    const YAML::Node &elements_node = doc["limits"]["elements"];
    
    if(elements_node.size() != names_node.size()){
      LOG_ERROR("Invalid limits yaml file. Size of names is different than size of elements");
      throw std::invalid_argument("Invalid yaml file");
    }
    
    base::JointLimits limits;
    for(uint i = 0; i < names_node.size(); i++){
      std::string name;
      names_node[i] >> name;
      limits.names.push_back(name);
      
      base::JointLimitRange range;
      elements_node[i]["max"]["position"] >> range.max.position;
      elements_node[i]["min"]["position"] >> range.min.position;
      elements_node[i]["max"]["speed"] >> range.max.speed;
      try{
	elements_node[i]["min"]["speed"] >> range.min.speed;
      }catch(...){
	range.min.speed = -range.max.speed;
      }
      elements_node[i]["max"]["effort"] >> range.max.effort;
      try{
	elements_node[i]["min"]["effort"] >> range.min.effort;
      }catch(...){
	range.min.effort = -range.max.effort;
      }
      
      limits.elements.push_back(range);
    }
    
    return limits;
  }
  
  double getRandom(double min, double max) 
  {
    /* Returns a random double between min and max */
    srand (time(NULL));
    
    return (max - min) * ( (double)rand() / (double)RAND_MAX ) + min;
  }
  
}