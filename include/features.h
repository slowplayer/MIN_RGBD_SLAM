#ifndef FEATURES_H
#define FEATURES_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <string>
#include <stdint.h>
#include <algorithm>

#include "ParameterServer.h"
#include "feature_adjuster.h"

namespace MIN_RGBD_SLAM
{

cv::Feature2D* createDetector();
cv::Ptr<cv::DescriptorExtractor> createDescriptorExtractor();
int bruteForceSearchORB(const uint64_t* v, const uint64_t* search_array, const unsigned int& size, int& result_index);
  
}
#endif