#include "Node.h"

namespace MIN_RGBD_SLAM
{
Node::Node(const cv::Mat& imRGB, const cv::Mat& imDepth, const cv::Mat& detector_mask,
	   double timestamp, cv::Ptr< cv::Feature2D > detector, cv::Ptr< cv::DescriptorExtractor > extractor)
:id_(-1),seq_id_(-1),vertex_id_(-1),timestamp_(timestamp),pc_col(new pointcloud_type())
{
  createXYZRGBPointCloud(imDepth,imRGB);
  
  cv::Mat imGray;
  if(imRGB.type()==CV_8UC3)
    cv::cvtColor(imRGB,imGray,CV_RGB2GRAY);
  else
    imGray=imRGB;
  
  detector->detect(imGray,feature_location_2d_,detector_mask);
  
  removeDepthless(feature_location_2d_,imDepth);
  
  extractor->compute(imGray,feature_location_2d_,features_descriptors_);
  
  projectTo3D(feature_location_2d_,feature_location_3d_,feature_color_3d_,imDepth,imRGB);
  
  assert(feature_location_2d_.size()==feature_location_3d_.size());
  assert(feature_location_3d_.size()==(unsigned int)features_descriptors_.rows);
}
pointcloud_type* Node::createXYZRGBPointCloud (const cv::Mat& depth_msg, const cv::Mat& rgb_msg)
{
  pointcloud_type* cloud(new point_type());
  cloud->dense=false;
  
  ParameterServer* ps=ParameterServer::instance();
  float fxinv,fyinv,cx,cy;
  fxinv=1.0/ps->getParam("fx");
  fyinv=1.0/ps->getParam("fy");
  cx=ps->getParam("cx");
  cy=ps->getParam("cy");
  int data_skip_step=static_cast<int>(ps->getParam("cloud_creation_skip_step"));
  
  cloud->height=ceil(depth_msg.rows/static_cast<float>(data_skip_step));
  cloud->width=ceil(depth_msg.cols/static_cast<float>(data_skip_step));
  
  char red_idx=0,green_idx=1,blue_idx=2; //RGB-format
  
  unsigned int depth_pix_step=(depth_msg.cols/cloud->width);
  unsigned int depth_row_step=(depth_msg.rows/cloud->height-1)*depth_msg.cols;
  unsigned int color_pix_step=3*depth_pix_step;
  unsigned int color_row_step=3*depth_row_step;
  
  cloud->resize(cloud->height*cloud->width);
  
  int color_idx=0;
  int depth_idx=0;
  double depth_scaling=static_cast<double>(ps->getParam("depth_scaling_factor"));
  float max_depth=ps->getParam("maximum_depth");
  float min_depth=ps->getParam("minimum_depth");
  if(max_depth<0.0)max_depth=std::numeric_limits<float>::infinity();
  
  pointcloud_type::iterator pt_iter;
  for(int v=0;v<(int)rgb_msg.rows;v+=data_skip_step,color_idx+=color_row_step;depth_idx+=depth_row_step)
  {
    for(int u=0;u<(int)rgb_msg.cols;u+=data_pix_step,color_idx+=color_pix_step;depth_idx+=depth_pix_step)
    {
      if(pt_iter==cloud->end())
	break;
      point_type& pt=*pt_iter;
      float Z=depth_msg.at<float>(depth_idx)*depth_scaling;
      if(!(Z>=min_depth))
      {
	pt.x=(u-cx)*1.0*fxinv;
	pt.y=(v-cy)*1.0*fyinv;
	pt.z=std::numeric_limits< float >::quiet_NaN();
      }
      else
      {
	 pt.x=(u-cx)*Z*fxinv;
	 pt.y=(v-cy)*Z*fyinv;
	 pt.z=Z;
      }
      RGBValue color;
      if(color_idx>0&&color_idx<rgb_msg.total()*color_pix_step)
      {
	color.Red=rgb_msg.at<uint8_t>(color_idx+red_idx);
	color.Green=rgb_msg.at<uint8_t>(color_idx+green_idx);
	color.Blue=rgb_msg.at<uint8_t>(color_idx+blue_idx);
	color.Alpha=0;
	pt.data[3]=color.float_value;
      }
    }
  }
  return cloud;
}
void Node::removeDepthless(std::vector< cv::KeyPoint >& feature_location_2d, const cv::Mat& depth)
{
  cv::Point2f  p2d;
  float Z;
  unsigned int i=0;
  while(i<feature_location_2d.size())
  {
    p2d=feature_location_2d[i].pt;
    if(p2d.x>=depth.cols||p2d.x<0||
      p2d.y>=depth.rows||p2d.y<0||
      std::isnan(p2d.x)||std::isnan(p2d.y))
    {
      feature_location_2d.erase(feature_location_2d.begin()+i);
      continue;
    }
    Z=depth.at<float>(round(p2d.y),round(p2d.x));
    if(std::isnan(Z))
    {
      feature_location_2d.erase(feature_location_2d.begin()+i);
      continue;
    }
    i++;
  }
  
  size_t max_keyp=static_cast<size_t>(ParameterServer::instance()->getParam("max_keypoints"));
  if(feature_location_2d.size()>max_keyp)
  {
    cv::KeyPointsFilter::retainBest(feature_location_2d,max_keyp);
    feature_location_2d.resize(max_keyp);
  }
}

void Node::projectTo3D(std::vector< cv::KeyPoint >& feature_location_2d, std_vector_of_eigen_vector4f& feature_location_3d,std::vector<cv::Vec3b>& feature_color_3d,const cv::Mat& depth,const cv::Mat& color)
{
  ParameterServer* ps=ParameterServer::instance();
  
  double depth_scaling=static_cast<double>(ps->getParam("depth_scaling_factor"));
  size_t max_keyp=static_cast<size_t>(ps->getParam("max_keypoints"));
  double maximum_depth=static_cast<double>(ps->getParam("maximum_depth"));
  float fxinv=1./ps->getParam("camera_fx");
  float fyinv=1./ps->getParam("camera_fy");
  float cx=ps->getParam("camera_cx");
  float cy=ps->getParam("camera_cy");
  
  if(feature_location_3d.size())
    feature_location_3d.clear();
  
  cv::Point2d p2d;
  float Z;
  float x,y,z;
  int u,v;
  cv::Vec3b c;
  for(unsigned int i=0;i<feature_location_2d.size();i++)
  {
    p2d=feature_location_2d_[i];
    u=round(p2d.x);
    v=round(p2d.y);
    Z=depth.at<float>(v,u);
    c=color.at<cv::Vec3b>(v,u);
    
    x=(p2d.x-cx)*Z*fxinv;
    y=(p2d.y-cy)*Z*fyinv;
    z=Z;
    
    feature_location_3d.push_back(Eigen::Vector4f(x,y,z,1.0));
    feature_color_3d.push_back(c);
  }
}
}