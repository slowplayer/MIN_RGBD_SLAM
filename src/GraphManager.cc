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
  //Initial Comparison
  MatchingResult mr;
  Node* prev_node=graph_[graph_.size()-1];
  mr=new_node->matchNodePair(prev_node);
  
  //TODO:evaluate mr and add egde to g2o
  
  //Get Potenial Edge
  std::list<int> vertices_to_comp;
  //TODO:add prev_best or not
  vertices_to_comp = getPotentialEdgeTargetsWithDijkstra(new_node, seq_cand, geod_cand, samp_cand, curr_best_result_.edge.id1); 
  
  //Compare node pairs
  std::list<const Node*> nodes_to_comp;
  for(std::list<int>::iterator it=vertices_to_comp.begin();it!=vertices_to_comp.end();it++)
  {
    nodes_to_comp.push_back(graph_[*it]);
  }
  
  //TODO:run in parallel
  std::list<MatchingResult> results;
  for(std::list<const Node*>::iterator it=nodes_to_comp.begin();it!=nodes_to_comp.end();it++)
  {
    results.push_back(new_node->matchNodePair(*it));
  }
  
  for(std::list<MatchingResult>::iterator it=results.begin();it!=results.end();it++)
  {
    MatchingResult mr=*it;
  }
}
std::list< int > GraphManager::getPotentialEdgeTargetsWithDijkstra(const Node* new_node, int seq_targets, int geod_targets, int samp_targets, int prodecessor_id, bool include_predecessor)
{
  std::list<int> ids_to_link_to;
  if(prodecessor_id<0)
    prodecessor_id=graph_.size()-1;
  
  //the number of vertices already in graph is not enough
  if((int)camera_vertices.size()<=seq_targets+geod_targets+samp_targets||camera_vertices.size()<=1)
  {
    seq_targets=seq_targets+geod_targets+samp_targets;
    geod_targets=0;
    samp_targets=0;
    prodecessor_id=graph_.size()-1;
  }
  
  //Time continuous
  if(seq_targets>0)
  {
    for(int i=0;i<seq_targets+1&&prodecessor_id-i>=0;i++)
    {
      ids_to_link_to.push_back(prodecessor_id-i);
    }
  }
  //spatial continuous
  if(geod_targets>0)
  {
    g2o::HyperDijkstra hypdij(optimizer_);
    g2o::UniformCostFunction cost_function;
    g2o::VertexSE3* prev_vertex=dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_[predecessor_id]->id_));
    hypdij.shortesPaths(prev_vertex,&cost_function,ParameterServer::instance()->getParam("geodesic_path"));
    g2o::HyperGraph::VertexSet& vs=hypdij.visited();
    
    std::map<int,int> vertex_id_node_id;
    for(graph_it it=graph_.begin();it!=graph_.end();++it)
    {
      Node* node=it->second;
      vertex_id_node_id[node->vertex_id_]=node->id_;
    }
  //Geodesic Neighbours except sequential
    std::map<int,int> neighbour_indices;
    int sum_of_weights=0;
    int vid,id;
    for(g2o::HyperGraph::VertexSet::iterator vit=vs.begin();vit!=vs.end();vit++)
    {
      
      vid=(*vit)->id();
      if(vertex_id_node_id.count(vid))
	id=vertex_id_node_id.at(vid);
      else
	continue;
      if(graph_.at(id)->matchable_)continue;
      if(id<prodecessor_id-seq_targets||(id>prodecessor_id&&id<=(int)graph_.size()-1))
      {
	int weight=abs(prodecessor_id-id);
	neighbour_indices[id]=weight;
	sum_of_weights+=weight;
      }
    }
  
    while(ids_to_link_to.size()<seq_targets+geod_targets&&neighbour_indices.size()!=0)
    {
      int random_pick=rand()%sum_of_weights;
      int weight_so_far=0;
      for(std::map<int,int>::iterator map_it=neighbour_indices.begin();map_it!=neighbour_indices.end();map_it++)
      {
	weight_so_far+=map_it->second;
	if(weight_so_far>random_pick)
	{
	  int sampled_id=map_it->first;
	  ids_to_link_to.push_front(sampled_id);
	  sum_of_weights-=map_it->second;
	  neighbour_indices.erase(map_it);
	  break;
	}
      }
    }
  }
  //Sample targets from graph-neighbours
  if(samp_targets>0)
  {
    std::vector<int> non_neighbour_indices;
    non_neighbour_indices.reserve(graph_.size());
    for(std::list<int>::iterator it=keyframe_ids_.begin();it!=keyframe_ids_.end();it++)
    {
      if(find(ids_to_link_to.begin(),ids_to_link_to.end(),*it)!=ids_to_link_to.end())
	continue;
      if(!graph_.at(*id)->matchable_)
	continue;
      non_neighbour_indices.push_back(*it);
    }
    while(ids_to_link_to<seq_targets+geod_targets+samp_targets&&non_neighbour_indices.size()!=0)
    {
      int id=rand()%non_neighbour_indices.size();
      int sampled_id=non_neighbour_indices[id];
      non_neighbour_indices[id]=non_neighbour_indices.back();
      non_neighbour_indices.pop_back();
      ids_to_link_to.push_front(id);
    }
  }
  if(include_predecessor)
  {
    ids_to_link_to.push_back(prodecessor_id);
  }
  
  return ids_to_link_to;
}
void GraphManager::optimizeGraph()
{

} 
}