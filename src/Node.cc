#include "Node.h"

namespace MIN_RGBD_SLAM
{
Node::Node(const cv::Mat& imRGB, const cv::Mat& imDepth, const cv::Mat& detector_mask,
	   double timestamp, cv::Ptr< cv::Feature2D > detector, cv::Ptr< cv::DescriptorExtractor > extractor)
:id_(-1),seq_id_(-1),vertex_id_(-1),timestamp_(timestamp),pc_col(new pointcloud_type()),
matchable_(true),initial_node_matches_(0),valid_tf_estimate_(true)
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
MatchingResult Node::matchNodePair(const Node* older_node)
{
  MatchingResult mr;
  bool found_transformation=false;
  double ransac_quality=0;
  unsigned int min_matches=(unsigned int)ParameterServer::instance()->getParam("min_matches");
  
  featureMatching(older_node,&mr.all_matches);
  
  if(mr.all_matches.size()>=min_matches)
  {
    found_transformation=getRelativeTransformationTo(older_node,&mr->all_matches,mr.ransac_trafo,mr.rmse,mr.inlier_matches);
    if(found_transformation)
    {
      mr.final_trafo=mr.ransac_trafo;
      mr.edge.informationMatrix=Eigen::Matrix<double,6,6>::Identity()*(mr.inlier_matches.size()/(mr.rmse*mr.rmse));
      mr.edge.id1=older_node->id_;
      mr.edge.id2=this->id_;
      mr.edge.transform=mr.final_trafo.cast<double>();
      
      //EMM model 
      pairwiseObservationLikelihood(this,older_node,mr);
      found_transformation=observation_criterion_met(mr.inlier_points,mr.outlier_points,
	mr.inlier_points+mr.occluded_points+mr.outlier_points,ransac_quality);
    }
  }
  if(found_transformation)
    ++initial_node_matches_;
  else
  {
    mr.edge.id1=-1;
    mr.edge.id2=-1;
  }
  return mr;
}
bool isNearer(const cv::DMatch& m1, const cv::DMatch& m2) { 
  return m1.distance < m2.distance; 
}
static void keepStrongestMatches(int n, std::vector<cv::DMatch>* matches)
{
  if(matches->size() > n)
  {
    std::vector<cv::DMatch>::iterator nth = matches->begin() + n;
    std::nth_element(matches->begin(), nth, matches->end(), isNearer);
    matches->erase(nth, matches->end());
  }
}
unsigned int Node::featureMatching(const Node* other, std::vector< cv::DMatch >* matches) const
{
  assert(matches->size()==0);
  
  uint64_t* query_value=reinterpret_cast<uint64_t*>(this->features_descriptors_.data);
  uint64_t* search_array=reinterpret_cast<uint64_t*>(other->features_descriptors_.data);
  
  for(unsigned int i=0;i<this->feature_location_2d_.size();++i,query_value+=4)
  {
    int result_index=-1;
    int hd=bruteForceSearchORB(query_value,search_array,other->feature_location_2d_.size(),result_index);
    if(hd>=128)continue;
    cv::DMatch match(i,result_index,hd/256.0+(float)rand()/(1000.0*RAND_MAX));
    matches->push_back(match);
  }
  
  float max_matches=ParameterServer::instance()->getParam("max_matches");
  keepStrongestMatches(static_cast<int>(max_matches),matches);
  
  return matches->size();
}
std::vector< cv::DMatch > Node::sample_matches_prefer_by_distance(unsigned int sample_size, std::vector< cv::DMatch >& matches_with_depth)
{
  std::set<unsigned int> sampled_ids;
  int safety_net=0;
  
  while(sampled_ids.size()<sample_size&&matches_with_depth.size()>=sample_size)
  {
      int id1=rand()%matches_with_depth.size();
      int id2=rand()%matches_with_depth.size();
      if(id1>id2)id1=id2;
      sampled_ids.insert(id1);
      if(++safety_net>10000)break;
  }
  std::vector<cv::DMatch> sampled_matches;
  sampled_matches.reserve(sampled_ids.size());
  for(std::set<unsigned int>::iterator it=sampled_ids.begin();it!=sampled_ids.end();++it)
  {
    sampled_matches.push_back(matches_with_depth[*it]);
  }
  return sampled_matches;
}
Eigen::Matrix4f getTransformFromMatches(const Node* newer_node,const Node* earlier_node,
					  const std::vector<cv::DMatch>& matches,
					  bool& valid,const float max_dist_m)
{
  pcl::TransformationFromCorrespondences tfc;
  valid=true;
  float weight;
  for(unsigned int i=0;i<matches.size();i++)
  {
    const cv::DMatch& m=matches[i];
    Eigen::Vector3f from=newer_node->feature_location_3d_[m.queryIdx].head<3>();
    Eigen::Vector3f to=newer_node->feature_location_3d_[m.queryIdx].head<3>();
    if(std::isnan(from(2))||std::isnan(to(2)))
      continue;
    weight=1.0/(from(2)*to(2));
    
    tfc.add(from,to,weight);
  }
  return tfc.getTransformation().matrix();
}
void Node::computeInliersAndError(const std::vector<cv::DMatch> & all_matches,
                                  const Eigen::Matrix4f& transformation4f,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
                                  size_t min_inliers, //break if this can't be reached
                                  std::vector<cv::DMatch>& inliers, //pure output var
                                  double& return_mean_error,//pure output var: rms-mahalanobis-distance
                                  double squaredMaxInlierDistInM) const
{ 
  inliers.clear();
  assert(all_matches.size() > 0);
  inliers.reserve(all_matches.size());
  //errors.clear();
  const size_t all_matches_size = all_matches.size();
  double mean_error = 0.0;
  Eigen::Matrix4d transformation4d = transformation4f.cast<double>();

  for(int i=0; i < all_matches_size; ++i)
  {
    const cv::DMatch& m = all_matches[i];
    const Eigen::Vector4f& origin = origins[m.queryIdx];
    const Eigen::Vector4f& target = earlier[m.trainIdx];
    if(origin(2) == 0.0 || target(2) == 0.0){ //does NOT trigger on NaN
       continue;
    }
    double mahal_dist = errorFunction2(origin, target, transformation4d);
    if(mahal_dist > squaredMaxInlierDistInM){
      continue; //ignore outliers
    }
    if(!(mahal_dist >= 0.0)){
      continue;
    }
    mean_error += mahal_dist;
    inliers.push_back(m); //include inlier
  }

  if (inliers.size()<3){ //at least the samples should be inliers
    return_mean_error = 1e9;
  } else {
    mean_error /= inliers.size();
    return_mean_error = sqrt(mean_error);
  }

}
bool Node::getRelativeTransformationTo(const Node* earlier_node, std::vector< cv::DMatch >* initial_matches, Eigen::Matrix4f& resulting_transformation, float& rmse, std::vector< cv::DMatch >& matches) const
{
  unsigned int min_inlier_threshold=static_cast<unsigned int>(ParameterServer::instance()->getParam("min_matches"));
  const float max_dist_m=ParameterServer::instance()->getParam("max_dist_for_inliers");
  const int ransac_iterations=static_cast<int>(ParameterServer::instance()->getParam("ransac_iterations"));
  const int refine_iterations=static_cast<int>(ParameterServer::instance()->getParam("refine_iterations"));
  
  double inlier_error;
  matches.clear();
  resulting_transformation=Eigen::Matrix4f::Identity();
  rmse=1e6;
  unsigned int valid_iterations=0;
  const unsigned int sample_size=4;
  bool valid_tf=false;
 
  std::vector<cv::DMatch>* matches_with_depth=initial_matches;
  std::sort(matches_with_depth->begin(),matches_with_depth->end());
  
  int real_iterations = 0;
  for(int n = 0; (n < ransac_iterations && matches_with_depth->size() >= sample_size); n++) //Without the minimum number of matches, the transformation can not be computed as usual TODO: implement monocular motion est
  {
    //Initialize Results of refinement
    double refined_error = 1e6;
    std::vector<cv::DMatch> refined_matches; 
    std::vector<cv::DMatch> inlier = sample_matches_prefer_by_distance(sample_size, *matches_with_depth); //initialization with random samples 
    //std::vector<cv::DMatch> inlier = sample_matches(sample_size, *matches_with_depth); //initialization with random samples 
    Eigen::Matrix4f refined_transformation = Eigen::Matrix4f::Identity();

    real_iterations++;
    for(int refinements = 1; refinements < refine_iterations; refinements++) 
    {
        Eigen::Matrix4f transformation = getTransformFromMatches(this, earlier_node, inlier,valid_tf,max_dist_m);
        if (!valid_tf || transformation!=transformation)  //Trafo Contains NaN?
          break; // valid_tf is false iff the sampled points aren't inliers themself 

        //test which features are inliers 
        computeInliersAndError(*initial_matches, transformation, 
                               this->feature_location_3d_, //this->feature_depth_stats_, 
                               earlier_node->feature_location_3d_, //earlier_node->feature_depth_stats_, 
                               std::max(min_inlier_threshold, static_cast<unsigned int>(refined_matches.size())), //break if no chance to reach this amount of inliers
                               inlier, inlier_error, max_dist_m*max_dist_m); 
        
        if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
          break; //hopeless case
        }

        //superior to before?
        if (inlier.size() >= refined_matches.size() && inlier_error <= refined_error) {
          size_t prev_num_inliers = refined_matches.size();
          assert(inlier_error>=0);
          refined_transformation = transformation;
          refined_matches = inlier;
          refined_error = inlier_error;
          if(inlier.size() == prev_num_inliers) break; //only error improved -> no change would happen next iteration
        }
        else break;
    }  //END REFINEMENTS
    //Successful Iteration?
    if(refined_matches.size() > 0){ //Valid?
        valid_iterations++;

        //Acceptable && superior to previous iterations?
        if (refined_error <= rmse &&  
            refined_matches.size() >= matches.size() && 
            refined_matches.size() >= min_inlier_threshold)
        {
          rmse = refined_error;
          resulting_transformation = refined_transformation;
          matches.assign(refined_matches.begin(), refined_matches.end());
          //Performance hacks:
          if (refined_matches.size() > initial_matches->size()*0.5) n+=10;///Iterations with more than half of the initial_matches inlying, count tenfold
          if (refined_matches.size() > initial_matches->size()*0.75) n+=10;///Iterations with more than 3/4 of the initial_matches inlying, count twentyfold
          if (refined_matches.size() > initial_matches->size()*0.8) break; ///Can this get better anyhow?
        }
    }
  } //iterations
  if(valid_iterations == 0) // maybe no depth. Try identity?
  { 
    //IDENTITYTEST
    //1 ransac iteration with identity
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();//hypothesis
    std::vector<cv::DMatch> inlier; //result
    //test which samples are inliers 
    computeInliersAndError(*initial_matches, transformation, 
                           this->feature_location_3d_, //this->feature_depth_stats_, 
                           earlier_node->feature_location_3d_, //earlier_node->feature_depth_stats_, 
                           min_inlier_threshold, //break if no chance to reach this amount of inliers
                           inlier, inlier_error, max_dist_m*max_dist_m); 
    
    //superior to before?
    if (inlier.size() > min_inlier_threshold && inlier_error < max_dist_m) {
      assert(inlier_error>=0);
      resulting_transformation = transformation;
      matches.assign(inlier.begin(), inlier.end());
      rmse = inlier_error;
      valid_iterations++;
    }
  } //END IDENTITY AS GUESS
  
  //TODO:G2O Refinement
  
  return matches.size()>=min_inlier_threshold;
}
}