/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Christoph Rösmann, Franz Albers
 *********************************************************************/

#ifndef GRAPH_SEARCH_INTERFACE_H
#define GRAPH_SEARCH_INTERFACE_H

#ifdef BOOST_NO_CXX11_DEFAULTED_FUNCTIONS
  #include <boost/graph/adjacency_list.hpp>
#else
  // Workaround for a bug in boost graph library (concerning directed graphs), boost version 1.48:
  // boost::add_vertex requires a move constructor/assignment operator in one of the underlying boost objects if C++11 is activated,
  // but they are missing. The compiler fails due to an implicit deletion. We just deactivate C++11 default functions for now.
  #define BOOST_NO_CXX11_DEFAULTED_FUNCTIONS
  #include <boost/graph/adjacency_list.hpp>
  #undef BOOST_NO_CXX11_DEFAULTED_FUNCTIONS
#endif

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/utility.hpp>
#include <boost/random.hpp>

#include <Eigen/Core>

#include <geometry_msgs/Twist.h>

#include <teb_local_planner/equivalence_relations.h>
#include <teb_local_planner/pose_se2.h>
#include <teb_local_planner/teb_config.h>

namespace teb_local_planner
{

class HomotopyClassPlanner; // Forward declaration

//! Vertex in the graph that is used to find homotopy classes (only stores 2D positions)
struct HcGraphVertex
{
public:
  Eigen::Vector2d pos; // position of vertices in the map
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for the homotopy class search-graph type @see HcGraphVertex
typedef boost::adjacency_list < boost::listS, boost::vecS, boost::directedS, HcGraphVertex, boost::no_property > HcGraph;
//! Abbrev. for vertex type descriptors in the homotopy class search-graph
typedef boost::graph_traits<HcGraph>::vertex_descriptor HcGraphVertexType;
//! Abbrev. for edge type descriptors in the homotopy class search-graph
typedef boost::graph_traits<HcGraph>::edge_descriptor HcGraphEdgeType;
//! Abbrev. for the vertices iterator of the homotopy class search-graph
typedef boost::graph_traits<HcGraph>::vertex_iterator HcGraphVertexIterator;
//! Abbrev. for the edges iterator of the homotopy class search-graph
typedef boost::graph_traits<HcGraph>::edge_iterator HcGraphEdgeIterator;
//! Abbrev. for the adjacency iterator that iterates vertices that are adjecent to the specified one
typedef boost::graph_traits<HcGraph>::adjacency_iterator HcGraphAdjecencyIterator;

//!< Inline function used for calculateHSignature() in combination with HCP graph vertex descriptors
inline std::complex<long double> getCplxFromHcGraph(HcGraphVertexType vert_descriptor, const HcGraph& graph)
{
  return std::complex<long double>(graph[vert_descriptor].pos.x(), graph[vert_descriptor].pos.y());
}

//!< Inline function used for initializing the TEB in combination with HCP graph vertex descriptors
inline const Eigen::Vector2d& getVector2dFromHcGraph(HcGraphVertexType vert_descriptor, const HcGraph& graph)
{
  return graph[vert_descriptor].pos;
}

/**
 * @brief Base class for graph based path planning / homotopy class sampling
 */
class GraphSearchInterface
{
public:

  virtual void createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity, bool free_goal_vel = false) = 0;

  /**
   * @brief Clear any existing graph of the homotopy class search
   */
  void clearGraph() {graph_.clear();}

  // HcGraph graph() const {return graph_;}
  // Workaround. graph_ is public for now, beacuse otherwise the compilation fails with the same boost bug mentioned above.
  HcGraph graph_; //!< Store the graph that is utilized to find alternative homotopy classes.

protected:
  /**
   * @brief Protected constructor that should be called by subclasses
   */
  GraphSearchInterface(const TebConfig& cfg, HomotopyClassPlanner* hcp) : cfg_(&cfg), hcp_(hcp){}

  /**
   * @brief Depth First Search implementation to find all paths between the start and the specified goal vertex.
   *
   * Complete paths are stored to the internal path container.
   * @sa http://www.technical-recipes.com/2011/a-recursive-algorithm-to-find-all-paths-between-two-given-nodes/
   * @param g Graph on which the depth first should be performed
   * @param visited A container that stores visited vertices (pass an empty container, it will be filled inside during recursion).
   * @param goal Desired goal vertex
   * @param start_orientation Orientation of the first trajectory pose, required to initialize the trajectory/TEB
   * @param goal_orientation Orientation of the goal trajectory pose, required to initialize the trajectory/TEB
   * @param start_velocity start velocity (optional)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed, otherwise the final velocity will be zero (default: false)
   */
  void DepthFirst(HcGraph& g, std::vector<HcGraphVertexType>& visited, const HcGraphVertexType& goal, double start_orientation, double goal_orientation, const geometry_msgs::Twist* start_velocity, bool free_goal_vel = false);


protected:
    const TebConfig* cfg_; //!< Config class that stores and manages all related parameters
    HomotopyClassPlanner* const hcp_; //!< Raw pointer to the HomotopyClassPlanner. The HomotopyClassPlanner itself is guaranteed to outlive the graph search class it is holding.

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



class lrKeyPointGraph : public GraphSearchInterface
{
public:
  lrKeyPointGraph(const TebConfig& cfg, HomotopyClassPlanner* hcp) : GraphSearchInterface(cfg, hcp){}

  virtual ~lrKeyPointGraph(){}

  /**
   * @brief Create a graph containing points in the global frame that can be used to explore new possible paths between start and goal.
   *
   * This version of the graph creation places a keypoint on the left and right side of each obstacle w.r.t to the goal heading. \n
   * All feasible paths between start and goal point are extracted using a Depth First Search afterwards. \n
   * This version works very well for small point obstacles. For more complex obstacles call the createProbRoadmapGraph()
   * method that samples keypoints in a predefined area and hopefully finds all relevant alternative paths.
   *
   * @see createProbRoadmapGraph
   * @param start Start pose from wich to start on (e.g. the current robot pose).
   * @param goal Goal pose to find paths to (e.g. the robot's goal).
   * @param dist_to_obst Allowed distance to obstacles: if not satisfying, the path will be rejected (note, this is not the distance used for optimization).
   * @param obstacle_heading_threshold Value of the normalized scalar product between obstacle heading and goal heading in order to take them (obstacles) into account [0,1]
   * @param start_velocity start velocity (optional)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed, otherwise the final velocity will be zero (default: false)
   */
  virtual void createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity, bool free_goal_vel = false);
};




class ProbRoadmapGraph : public GraphSearchInterface
{
public:
  ProbRoadmapGraph(const TebConfig& cfg, HomotopyClassPlanner* hcp) : GraphSearchInterface(cfg, hcp){}

  virtual ~ProbRoadmapGraph(){}


  /**
   * @brief Create a graph and sample points in the global frame that can be used to explore new possible paths between start and goal.
   *
   * This version of the graph samples keypoints in a predefined area (config) in the current frame between start and goal. \n
   * Afterwards all feasible paths between start and goal point are extracted using a Depth First Search. \n
   * Use the sampling method for complex, non-point or huge obstacles. \n
   * You may call createGraph() instead.
   *
   * @see createGraph
   * @param start Start pose from wich to start on (e.g. the current robot pose).
   * @param goal Goal pose to find paths to (e.g. the robot's goal).
   * @param dist_to_obst Allowed distance to obstacles: if not satisfying, the path will be rejected (note, this is not the distance used for optimization).
   * @param no_samples number of random samples
   * @param obstacle_heading_threshold Value of the normalized scalar product between obstacle heading and goal heading in order to take them (obstacles) into account [0,1]
   * @param start_velocity start velocity (optional)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed, otherwise the final velocity will be zero (default: false)
   */
  virtual void createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity, bool free_goal_vel = false);

private:
    boost::random::mt19937 rnd_generator_; //!< Random number generator used by createProbRoadmapGraph to sample graph keypoints.
};
} // end namespace

#endif // GRAPH_SEARCH_INTERFACE_H
