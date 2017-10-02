#ifndef NODE_H
#define NODE_H

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <pcl/common/transformation_from_correspondences.h>

#include "features.h"
#include "misc.h"
#include "ParameterServer.h"
#include "MatchingResult.h"
#include "EmmModel.h"

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
  
  //Compare the features of two nodes and compute the transformation
  MatchingResult matchNodePair(const Node* older_node);
  
  int id_;//number of camera nodes in the graph when node was added
  int seq_id_;//number of images that have been processed
  int vertex_id_;//id of the corresponding vertex in the g2o graph
  bool valid_tf_estimate_;//flags whether the data of this node should be considered for postprocessing steps
  bool matchable_;//flags whether the data for matching is available
  double timestamp_;//time of the data belonging to the node
  
  //2d-features,
  std::vector<cv::KeyPoint> feature_location_2d_;
  //3d-coordinates
  std_vector_of_eigen_vector4f feature_location_3d_;
  //features-descriptor
  cv::Mat features_descriptors_;
  //3d-pointcloud
  pointcloud_type::Ptr pc_col;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  pointcloud_type* createXYZRGBPointCloud (const cv::Mat& depth_msg, const cv::Mat& rgb_msg); 
  void removeDepthless(std::vector<cv::KeyPoint>& feature_location_2d,const cv::Mat& depth);
  void projectTo3D(std::vector<cv::KeyPoint>& feature_location_2d,
    std_vector_of_eigen_vector4f& feature_location_3d,std::vector<cv::Vec3b>& feature_color_3d,const cv::Mat& depth,const cv::Mat& color);
  
 
  unsigned int featureMatching(const Node* other,std::vector<cv::DMatch>* matches)const;
  //Compute the relative transformation between the nodes
  bool getRelativeTransformationTo(const Node* earlier_node,
				  std::vector<cv::DMatch>* initial_matches,
				   Eigen::Matrix4f& resulting_transformation,
				   float& rmse,std::vector<cv::DMatch>& matches)const;
  std::vector<cv::DMatch> sample_matches_prefer_by_distance(unsigned int sample_size,
					std::vector<cv::DMatch>& matches_with_depth);
  Eigen::Matrix4f getTransformFromMatches(const Node* newer_node,const Node* earlier_node,
					  const std::vector<cv::DMatch>& matches,
					  bool& valid,const float max_dist_m);
  
  void computeInliersAndError(const std::vector<cv::DMatch> & all_matches,
                                  const Eigen::Matrix4f& transformation4f,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
                                  size_t min_inliers, //break if this can't be reached
                                  std::vector<cv::DMatch>& inliers, //pure output var
                                  double& return_mean_error,//pure output var: rms-mahalanobis-distance
                                  //std::vector<double>& errors,
                                  double squaredMaxInlierDistInM) const;

  int initial_node_matches_;
  
  
};  
}
#endif