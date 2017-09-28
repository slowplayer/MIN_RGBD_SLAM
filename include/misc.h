#ifndef MISC_H
#define MISC_H

#include <cmath>
#include <Eigen/Core>
#include "g2o/types/slam3d/se3quat.h"
#include "ParameterServer.h"
#include "pcl/point_types.h"

namespace MIN_RGBD_SLAM
{
typedef pcl::PointXYZRGB point_type;
typedef pcl::PointCloud<point_type> pointcloud_type;
inline g2o::SE3Quat eigen2G2O(const Eigen::Matrix4d& eigen_mat) 
{
  Eigen::Affine3d eigen_transform(eigen_mat);
  Eigen::Quaterniond eigen_quat(eigen_transform.rotation());
  Eigen::Vector3d translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
  g2o::SE3Quat result(eigen_quat, translation);

  return result;
}
}