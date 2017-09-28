#ifndef PARAMETER_SERVER_H
#define PARAMETER_SERVER_H

#include <opencv2/core/core.hpp>

#include <map>
#include <string>

namespace MIN_RGBD_SLAM
{
class ParameterServer
{
public:
  static ParameterServer* instance();
  
  bool setPath(const std::string filename);
  
  inline float getParam(const std::string param){return config[param];}
  
private:
  ParameterServer();
  
  inline void setParam(const std::string param,float value){config[param]=value;}
  
  std::map<std::string,float> config;
  
  static ParameterServer* _instance;
};
}
#endif