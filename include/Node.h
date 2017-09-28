#ifndef NODE_H
#define NODE_H

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "features.h"
#include "misc.h"
#include "ParameterServer.h"

namespace MIN_RGBD_SLAM
{
typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > std_vector_of_eigen_vector4f;
class Node
{
public:
  Node(const cv::Mat& imRGB,const cv::Mat& imDepth,
       const cv::Mat& detector_mask,double timestamp,
       cv::Ptr<cv::Feature2D> detector,
       cv::Ptr<cv::DescriptorExtractor> extractor);
  
  Node(){};
  ~Node(){};
  
  //2d-features,
  std::vector<cv::KeyPoint> feature_location_2d_;
  //3d-coordinates
  std_vector_of_eigen_vector4f feature_location_3d_;
  //features-descriptor
  cv::Mat features_descriptors_;
  //3d-colors
  std::vector<cv::Vec3b> feature_color_3d_;
  //3d-pointcloud
  pointcloud_type* pc_col;
  
  int id_,seq_id_,vertex_id_;
  double timestamp_;
private:
  pointcloud_type* createXYZRGBPointCloud (const cv::Mat& depth_msg, const cv::Mat& rgb_msg); 
  //void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img);
  void removeDepthless(std::vector<cv::KeyPoint>& feature_location_2d,const cv::Mat& depth);
  void projectTo3D(std::vector<cv::KeyPoint>& feature_location_2d,
    std_vector_of_eigen_vector4f& feature_location_3d,std::vector<cv::Vec3b>& feature_color_3d,const cv::Mat& depth,const cv::Mat& color);
 
};  
}
#endif