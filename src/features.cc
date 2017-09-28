/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "features.h"

using namespace cv;

StatefulFeatureDetector* adjusterWrapper(cv::Ptr<DetectorAdjuster> detadj, int min, int max)
{
  int iterations = static_cast<int>(ParameterServer::instance()->getParam("adjuster_max_iterations"));
//  ROS_INFO("Using adjusted keypoint detector with %d maximum iterations, keeping the number of keypoints between %d and %d", iterations, min, max);
  return new  VideoDynamicAdaptedFeatureDetector(detadj, min, max, iterations);
}

StatefulFeatureDetector* adjustedGridWrapper(cv::Ptr<DetectorAdjuster> detadj)
{
  ParameterServer* ps = ParameterServer::instance();
  //Try to get between "max" keypoints and 1.5 times of that.
  //We actually want exactly "max_keypoints" keypoints. Therefore
  //it's better to overshoot and then cut away the excessive keypoints
  int min = static_cast<int>(ps->getParam("max_keypoints")); //Shall not get below max
  int max = min * 1.5; //

  int gridRes = static_cast<int>(ps->getParam("detector_grid_resolution"));
  int gridcells = gridRes*gridRes;
  int gridmin = round(min/static_cast<float>(gridcells));
  int gridmax =  round(max/static_cast<float>(gridcells));
  ROS_INFO("Using gridded keypoint detector with %dx%d cells, keeping %d keypoints in total.", gridRes, gridRes, max);
  
  StatefulFeatureDetector* detector = adjusterWrapper(detadj, gridmin, gridmax);

  return new VideoGridAdaptedFeatureDetector(detector, max, gridRes, gridRes);
}

//Use Grid or Dynamic or GridDynamic as prefix ORB
Feature2D* createDetector()
{
  //For anything but SIFTGPU
  DetectorAdjuster* detAdj = NULL;

  detAdj = new DetectorAdjuster("ORB", 20);
  
  assert(detAdj != NULL);

  ParameterServer* ps = ParameterServer::instance();
  int detector_grid_resolution=static_cast<int>(ps->getParam("detector_grid_resolution"));
  int adjuster_max_iterations=static_cast<int>(ps->getParam("adjuster_max_iterations"));
  bool gridWrap = (detector_grid_resolution > 1);
  bool dynaWrap = (adjuster_max_iterations > 0);

  if(dynaWrap && gridWrap){
    return adjustedGridWrapper(detAdj);
  }
  else if(dynaWrap){
    int min = static_cast<int>(ps->getParam("max_keypoints"));
    int max = min * 1.5; //params->get<int>("max_keypoints");
    return adjusterWrapper(detAdj, min, max);
  }
  else return detAdj;
}

cv::Ptr<DescriptorExtractor> createDescriptorExtractor(std::string descriptorType) 
{
  return ORB::create();
}

static inline int hamming_distance_orb32x8_popcountll(const uint64_t* v1, const uint64_t* v2) {
  return (__builtin_popcountll(v1[0] ^ v2[0]) + __builtin_popcountll(v1[1] ^ v2[1])) +
         (__builtin_popcountll(v1[2] ^ v2[2]) + __builtin_popcountll(v1[3] ^ v2[3]));
}

int bruteForceSearchORB(const uint64_t* v, const uint64_t* search_array, const unsigned int& size, int& result_index){
  //constexpr unsigned int howmany64bitwords = 4;//32*8/64;
  const unsigned int howmany64bitwords = 4;//32*8/64;
  assert(search_array && "Nullpointer in bruteForceSearchORB");
  result_index = -1;//impossible
  int min_distance = 1 + 256;//More than maximum distance
  for(unsigned int i = 0; i < size-1; i+=1, search_array+=4){
    int hamming_distance_i = hamming_distance_orb32x8_popcountll(v, search_array);
    if(hamming_distance_i < min_distance){
      min_distance = hamming_distance_i;
      result_index = i;
    }
  } 
  return min_distance;
}
