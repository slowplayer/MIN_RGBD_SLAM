#ifndef SYSTEM_H
#define SYSTEM_H

#include "ParameterServer.h"

#include "features.h"

#include "Node.h"
#include "GraphManager.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace MIN_RGBD_SLAM
{
class Node;
class GraphManager;
class System
{
public:
  System(const std::string paramFile);

  void TrackRGBD(const cv::Mat& imRGB,const cv::Mat& imDepth,double timestamp);
  
private:
  void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img);
  void processNode(Node* new_node);
  
  
  GraphManager* graph_mgr;
  
  cv::Ptr<cv::Feature2D> detector_;
  cv::cv::Ptr<cv::DescriptorExtractor> extractor_;
};
}
#endif