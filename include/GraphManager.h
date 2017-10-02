#ifndef GRAPH_MANAGER_H
#define GRAPH_MANAGER_H

#include <mutex>
#include <map>
#include <set>

#include <Eigen/Core>
#include <Eigen/Geometry>


#include "Node.h"
#include "MatchingResult.h"

#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

namespace MIN_RGBD_SLAM
{
typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3> > SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSpacrseSolver;

typedef std::trl::unordered_map<int,g2o::HyperGraph::Vertex*> VertexIDMap;
typedef std::pair<int,g2o::HyperGraph::Vertex*> VertexIDPair;
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;
class Node;
class GraphManager
{
public:
  GraphManager();
  ~GraphManager();
  
  bool addNode(Node* new_node);
  
   //Pose vertices (in camera coordinate system)
   g2o::HyperGraph::VertexSet camera_vertices;
   ///"Regular" edges from camera to camera as found form feature correspondeces
   g2o::HyperGraph::EdgeSet cam_cam_edges_;
private:
  void firstNode(Node* new_node);
  
  bool nodeComparisons(Node* new_node,
		       Eigen::Matrix4f& curr_motion_estimate,
		       bool& edge_to_keyframe);
  
  
  void optimizeGraph();
  
  void addKeyframe(int id);
  std::list<int> getPotentialEdgeTargetsWithDijkstra(const Node* new_node,
    int seq_targets,int geod_targets,int samp_targets,int prodecessor_id,bool include_predecessor);
  
  std::mutex mMutexOptimizer;
  g2o::SparseOptimizer* optimizer_;
  
  Eigen::Matrix4d init_base_pose_;
   
  std::map<int,Node*> graph_;
  std::list<int> keyframe_ids_;
  unsigned int min_matches;
  unsigned int next_seq_id,next_vertex_id;
  
  //get potenial edge
  int seq_cand,geod_cand,samp_cand;
};
}
#endif