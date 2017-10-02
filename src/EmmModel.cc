#include "EmmModel.h"

#define SQRT_2 1.41421

namespace MIN_RGBD_SLAM
{
inline int round(float d)
{
  return static_cast<int>(floor(d + 0.5));
}
inline double depth_std_dev(double depth)
{
  // From Khoselham and Elberink?
  static double depth_std_dev = ParameterServer::instance()->getParam("sigma_depth");
  // Previously used 0.006 from information on http://www.ros.org/wiki/openni_kinect/kinect_accuracy;
  // ...using 2sigma = 95%ile
  //static const double depth_std_dev  = 0.006;
  return depth_std_dev * depth * depth;
}
//Functions without dependencies
inline double depth_covariance(double depth)
{
  static double stddev = depth_std_dev(depth);
  static double cov = stddev * stddev;
  return cov;
}
double cdf(double x, double mu, double sigma)
{
	return 0.5 * (1 + erf((x - mu) / (sigma * SQRT_2)));
}

void observationLikelihood(const Eigen::Matrix4f& proposed_transformation,//new to old 
			   pointcloud_type::Ptr new_pc, pointcloud_type::Ptr old_pc, 
			   unsigned int& inliers, unsigned int& outliers, unsigned int& occluded, unsigned int& all)
{
   ParameterServer* ps=ParameterServer::instance();
   int skip_step=static_cast<int>(ps->getParam("emm_skip_step"));
   float observability_threshold=ps->getParam("observability_threshold");
   inliers=outliers=occluded=all=0;
   if(skip_step<0||observability_threshold<=0.0)
   {
     inliers=all=1;
     return;
   }
   if(old_pc->width<=1||old_pc->height<=1)
   {
     inliers=all=1;
     return;
   }
   if(old_pc->width!=new_pc->width)
     return;
   
   pointcloud_type new_pc_transformed;
   pcl::transformPointCloud(*new_pc,new_pc_transformed,proposed_transformation);
   
   float fx,fy,cx,cy;
   fx=ps->getParam("fx");
   fy=ps->getParam("fy");
   cx=ps->getParam("cx");
   cy=ps->getParam("cy");
   
   int cloud_creation_skip_step;//downsampled cloud
   cloud_creation_skip_step=static_cast<int>(ps->getParam("cloud_creation_skip_step"));
   fx/=cloud_creation_skip_step;
   fy/=cloud_creation_skip_step;
   cx/=cloud_creation_skip_step;
   cy/=cloud_creation_skip_step;
   
   //double sumloglikelihood,observation_count;
   unsigned int bad_points,good_points,occluded_points;
   //sumloglikelihood=observation_count=0.0;
   bad_points=good_points=occluded_points=0;
   
   for(int new_ry=0;new_ry<(int)new_pc->height;new_ry+=skip_step)
     for(int new_rx=0;new_rx<(int)new_pc->width;new_rx+=skip_step,all++)
     {
       point_type& p=new_pc_transformed.at(new_rx,new_ry);
       if(p.z!=p.z)continue;//NaN
       if(p.z<0)continue;//behind the camera
       int old_rx_center=round((p.x/p.z)*fx+cx);
       int old_ry_center=round((p.y/p.z)*fy+cy);
       if(old_rx_center>=(int)old_pc->width||old_rx_center<0||
	 old_ry_center>=(int)old_pc->height||old_ry_center<0)
	 continue
       int nbhd=2;//3x3 neighbourhood
       bool good_point,occluded_point,bad_point;
       good_point=occluded_point=bad_point=false;
       int startx=std::max(0,old_rx_center-nbhd);
       int starty=std::max(0,old_ry_center-nbhd);
       int endx=std::min(static_cast<int>(old_pc->width),old_rx_center+nbhd+1);
       int endy=std::min(static_cast<int>(old_pc->height),old_ry_center+nbhd+1);
       
       int nbhd_step=2;
       for(int old_ry=starty;old_ry<endy;old_ry+=nbhd_step)
	 for(int old_rx=startx;old_rx<endx;old_rx+=nbhd_step)
	 {
	    const point_type& old_p=old_pc->at(old_rx,old_ry);
	    if(old_p.z!=old_p.z)continue;//NaN
	    
	    double old_sigma=cloud_creation_skip_step*depth_covariance(old_p.z);
	    double new_sigma=cloud_creation_skip_step*depth_covariance(p.z);
	    double joint_sigma=old_sigma+new_sigma;
	    
	    double p_new_in_front=cdf(old_p.zm,p.z,sqrt(joint_sigma));
	    if(p_new_in_front<0.001)
	      occluded_point=true;
	    else if(p_new_in_front<0.999)
	      good_point=true;
	    else
	      bad_point=true;
	 }
	 if(good_point)good_points++;
	 if(occluded_point)occluded_points++;
	 if(bad_point)bad_points++;
     }
  inliers=good_points;
  outliers=bad_points;
  occluded=occluded_points;
}
bool observation_criterion_met(unsigned int inliers, unsigned int outliers, unsigned int all, double& quality)
{
  double obs_thresh = ParameterServer::instance()->getParam("observability_threshold");
  if(obs_thresh < 0) return true;
  quality = inliers/static_cast<double>(inliers+outliers);
  double certainty = inliers/static_cast<double>(all);
  bool criterion1_met = quality > obs_thresh; //TODO: parametrice certainty (and use meaningful statistic)
  bool criterion2_met = certainty > 0.25; //TODO: parametrice certainty (and use meaningful statistic)
  bool both_criteria_met = criterion1_met && criterion2_met;
  return both_criteria_met;
}
void pairwiseObservationLikelihood(const Node* newer_node, const Node* older_node, MatchingResult& mr)
{
 // double likelihood,confidence;
  unsigned int inlier_points,outlier_points,all_points,occluded_points;
  inlier_points=outlier_points=all_points=occluded_points=0;
  
  observationLikelihood(mr.final_trafo,newer_node->pc_col,older_node->pc_col,
			inlier_points,outlier_points,occluded_points,all_points);
  
  mr.inlier_points=inlier_points;
  mr.outlier_points=outlier_points;
  mr.occluded_points=occluded_points;
  mr.all_points=all_points;
}

}  