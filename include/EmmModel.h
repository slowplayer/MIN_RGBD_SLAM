#ifndef EMM_MODEL_H
#define EMM_MODEL_H

#include <Eigen/Core>
#include <math.h>

#include "misc.h"
#include "Node.h"
#include "MatchingResult.h"
#include "ParameterServer.h"

namespace MIN_RGBD_SLAM
{
  double cdf(double x, double mu, double sigma);
  void observationLikelihood(const Eigen::Matrix4f& proposed_transformation,
    pointcloud_type::Ptr new_pc,pointcloud_type::Ptr old_pc,
    unsigned int& inliers,unsigned int& outliers,unsigned int& occluded,unsigned int& all);
  bool observation_criterion_met(unsigned int inliers, unsigned int outliers, unsigned int all, double& quality);
  void pairwiseObservationLikelihood(const Node* newer_node,const Node* older_node,MatchingResult& mr);
};
#endif