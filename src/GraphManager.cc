#include "GraphManager.h"

namespace MIN_RGBD_SLAM
{
GraphManager::GraphManager()
:optimizer_(NULL),next_seq_id(0),next_vertex_id(0)
{
  optimizer_=new g2o::SparseOptimizer();
  optimizer_->setVerbose(false);
  
  SlamBlockSolver* solver=NULL;
  SlamLinearCSpacrseSolver* linearSolver=new SlamLinearCSpacrseSolver();
  linearSolver->setBlockOrdering(false);
  solver=new SlamBlockSolver(linearSolver);
  
  g2o::OptimizationAlgorithmLevenberg* algo=new g2o::OptimizationAlgorithmLevenberg(solver);
  optimizer_->setAlgorithm(algo);
}
//TODO::delete is need
GraphManager::~GraphManager()
{

}

bool GraphManager::addNode(Node* new_node)
{
  if(graph_.size()==0)
  {
    firstNode(new_node);
    return true;
  }
  
  Eigen::Matrix4d motion_estimate;
  bool edge_to_last_keyframe_found=false;
  bool found_match=nodeComparisons(new_node,motion_estimate,edge_to_last_keyframe_found);
}
void GraphManager::firstNode(Node* new_node)
{
  new_node->id_=graph_.size();
  new_node->seq_id_=next_seq_id++;
  new_node->vertex_id_=next_vertex_id++;
  
  init_base_pose_=Eigen::Matrix4d::Identity();
  
  g2o::VertexSE3* reference_pose=new g2o::VertexSE3;
  graph_[new_node->id_]=new_node;
  reference_pose->setId(new_node->vertex_id_);
  camera_vertices.insert(reference_pose);
  
  g2o::SE3Quat g2o_ref_se3=eigen2G2O(init_base_pose_);
  reference_pose->setEstimate(g2o_ref_se3);
  reference_pose->setFixed(true);
  
  {
    std::unique_lock<std::mutex> lock(mMutexOptimizer);
    optimizer_->addVertex(reference_pose);
  }
  
  addKeyframe(new_node->id_);
  
  optimizeGraph();
}
void GraphManager::addKeyframe(int id)
{
  keyframe_ids_.push_back(id);
}
bool GraphManager::nodeComparisons(Node* new_node, Eigen::Matrix4f& curr_motion_estimate, bool& edge_to_keyframe)
{

}
std::list< int > GraphManager::getPotentialEdgeTargetsWithDijkstra(const Node* new_node, int seq_targets, int geod_targets, int samp_targets, int prodecessor_id, bool include_predecessor)
{

}
void GraphManager::optimizeGraph()
{

} 
}