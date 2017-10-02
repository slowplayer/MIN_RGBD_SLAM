#ifndef MATCHING_RESULT_H
#define MATCHING_RESULT_H

#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
#include <set>
namespace MIN_RGBD_SLAM
{
struct LoadedEdge3D
{
  int id1, id2;
  Eigen::Isometry3d transform;
  Eigen::Matrix<double, 6,6> informationMatrix;
};

struct LoadedEdgeComparator3D
{
  inline bool operator()(const LoadedEdge3D& e1, const LoadedEdge3D& e2){
    int i11=e1.id1, i12=e1.id2;
    if (i11>i12){
      i11=e1.id2;
      i12=e1.id1;
    }
    int i21=e2.id1, i22=e2.id2;
    if (i21>i22){
      i21=e2.id2;
      i22=e2.id1;
    }
    if (i12<i22) return true;
    if (i12>i22) return false;
    return (i11<i21);
  }
};

typedef std::set<LoadedEdge3D, LoadedEdgeComparator3D> LoadedEdgeSet3D;
class MatchingResult 
{
    public:
        MatchingResult() : 
          rmse(0.0), 
          ransac_trafo(Eigen::Matrix4f::Identity()), 
          final_trafo(Eigen::Matrix4f::Identity()), 
          icp_trafo(Eigen::Matrix4f::Identity()),
          inlier_points(0), outlier_points(0), occluded_points(0)
        {
            edge.id1 = edge.id2 = -1;
        }
        std::vector<cv::DMatch> inlier_matches;
        std::vector<cv::DMatch> all_matches;
        LoadedEdge3D edge;
        float rmse;
        Eigen::Matrix4f ransac_trafo;
        Eigen::Matrix4f final_trafo;
        Eigen::Matrix4f icp_trafo;
        unsigned int inlier_points, outlier_points, occluded_points, all_points;
        const char* toString();
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
#endif