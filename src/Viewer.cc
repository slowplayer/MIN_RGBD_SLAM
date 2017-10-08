#include "Viewer.h"

namespace MIN_RGBD_SLAM
{
Viewer::Viewer(ColorOctomapServer* octree)
:mbFinishRequested(false),mbFinished(true),
mbStopped(false),mbStopRequested(false),octree_server(octree)
{
  ParameterServer* ps=ParameterServer::instance();
  float fps=ps->getParam("Camera.fps");
  if(fps<1)
    fps=30;
  mT=1e3/fps;
  
  mImageWidth=ps->getParam("Camera.width");
  mImageHeight=ps->getParam("Camera.height");
  if(mImageHeight<1||mImageWidth<1)
  {
    mImageWidth=640;
    mImageHeight=480;
  }
  
  mViewpointX=ps->getParam("Viewer.ViewpointX");
  mViewpointY=ps->getParam("Viewer.ViewpointY");
  mViewpointZ=ps->getParam("Viewer.ViewpointZ");
  mViewpointF=ps->getParam("Viewer.ViewpointF");
}
void Viewer::Run()
{
  mbFinished=false;
  
  pangolin::CreateWindowAndBind("MIN_RGBD_SLAM",1024,768);
  
  glEnable(GL_DEPTH_TEST);
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
    pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ,
      0,0,0,0.0,-1.0,0.0)
    );
  pangolin::View& d_cam=pangolin::CreateDisplay()
  .SetBounds(0.0,1.0,0.0,1.0,-1024.0f/768.0f)
  .SetHandlere(new pangolin::Handler3D(s_cam));
  
  while(1)
  {
     glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
     glClearColor(1.0f,1.0f,1.0f,1.0f);
     //TODO:render the octree
     octree_server->render();
    
     cv::waitKey(mT);
      
     if(Stop())
     {
	while(isStopped())
	{
	  usleep(3000);
        }
     }
     if(CheckFinish())
	break;
  }
  SetFinish();
}
void Viewer::RequestFinish()
{
  std::unique_lock<std::mutex> lock(mMutexFinish);
  mbFinishRequested=true;
}
bool Viewer::CheckFinish()
{
  std::unique_lock<std::mutex> lock(mMutexFinish);
  return mbFinishRequested;
}
void Viewer::SetFinish()
{
  std::unique_lock<std::mutex> lock(mMutexFinish);
  mbFinished=true;
}
bool Viewer::isFinished()
{
  std::unique_lock<std::mutex> lock(mMutexFinish);
  return mbFinished;
}
void Viewer::RequestStop()
{
  std::unique_lock<std::mutex> lock(mMutexStop);
  if(!mbStopped)
    mbStopRequested=true;
}
bool Viewer::isStopped()
{
  std::unique_lock<std::mutex> lock(mMutexStop);
  return mbStopped;
}
bool Viewer::Stop()
{
  std::unique_lock<std::mutex> lock(mMutexStop);
  std::unique_lock<std::mutex> lock2(mMutexFinish);
  if(mbFinishRequested)
    return false;
  else if(mbFinishRequested)
  {
    mbStopped=true;
    mbStopRequested=false;
    return true;
  }
  return false;
}
void Viewer::Release()
{
  std::unique_lock<std::mutex> lock(mMutexStop);
  mbStopped=false;
}






  
  
  
  
}