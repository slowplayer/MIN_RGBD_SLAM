#ifndef VIEWER_H
#define VIEWER_H

#include "ParameterServer.h"
#include "ColorOctomapServer.h"

#include <pangolin/pangolin.h>
#include <mutex>

namespace MIN_RGBD_SLAM
{
class ColorOctomapServer;
class Viewer
{
public:
  Viewer(ColorOctomapServer* octree);
  
  void Run();
  
  void RequestFinish();
  void RequestStop();
  bool isFinished();
  bool isStopped();
  void Release();
private:
  bool Stop();
  
  double mT;
  float mImageWidth,mImageHeight;
  
  float mViewpointX,mViewpointY,mViewpointZ,mViewpointF;
  
  bool CheckFinish();
  void SetFinish();
  
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;
  
  bool mbStopped;
  bool mbStopRequested;
  std::mutex mMutexStop;
  
  ColorOctomapServer* octree_server;
};
}
#endif