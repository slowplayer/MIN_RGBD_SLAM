#include "ParameterServer.h"

namespace MIN_RGBD_SLAM
{
ParameterServer* ParameterServer::_instance=NULL;
ParameterServer::ParameterServer()
{

}
ParameterServer* ParameterServer::instance()
{
  if(_instance==NULL)
    _instance=new ParameterServer();
  return _instance;
}
bool ParameterServer::setPath(const std::string filename)
{
  cv::FileStorage fs(filename);
  if(!fs.isOpened())
    return false;
  //TODO:add param
  
  
  
  return true;
}
}
