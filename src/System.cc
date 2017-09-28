#include "System.h"

namespace MIN_RGBD_SLAM 
{
System::System(const std::string paramFile)
{
  ParameterServer::instance()->setPath(paramFile);
  
  detector_=createDetector();
  extractor_=createDescriptorExtractor();
  
  //TODO: fix GraphManager constructor
  graph_mgr=new GraphManager();
  
}
void System::TrackRGBD(const cv::Mat& imRGB, const cv::Mat& imDepth, double timestamp)
{
  cv::Mat depth_mono8_img;
  
  //TODO:make sure the type of imDepth is CV_32FC1
  depthToCV8UC1(imDepth,depth_mono8_img);
  
  Node* node_ptr=new Node(imRGB,imDepth,depth_mono8_img,timestamp,detector_,extractor_);
  
  processNode(node_ptr);
}
void System::depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img)
{
  if(depth_img.type() == CV_32FC1)
  {
    depth_img.convertTo(mono8_img, CV_8UC1, 100,0); //milimeter (scale of mono8_img does not matter)
  }
  else if(depth_img.type() == CV_16UC1)
  {
    mono8_img = cv::Mat(depth_img.size(), CV_8UC1);
    cv::Mat float_img;
    depth_img.convertTo(mono8_img, CV_8UC1, 0.05, -25); //scale to 2cm
    depth_img.convertTo(float_img, CV_32FC1, 0.001, 0);//From mm to m(scale of depth_img matters)
    depth_img = float_img;
  }
}
void System::processNode(Node* new_node)
{
  bool has_been_added=graph_mgr->addNode(new_node);
}

}