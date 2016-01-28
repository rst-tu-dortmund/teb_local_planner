/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
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
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef HOMOTOPY_CLASS_PLANNER_H_
#define HOMOTOPY_CLASS_PLANNER_H_

#include <math.h>
#include <algorithm>
#include <functional>
#include <vector>
#include <iterator>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/random.hpp>
#include <boost/utility.hpp>


#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include <ros/console.h>
#include <ros/ros.h>

#include <teb_local_planner/planner_interface.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/visualization.h>


namespace teb_local_planner
{
  
 
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

/**
 * @class HomotopyClassPlanner
 * @brief Local planner that explores alternative homotopy classes, create a plan for each alternative
 *	  and finally return the robot controls for the current best path (repeated in each sampling interval)
 * 
 * Homotopy classes are explored using the help a search-graph. \n
 * A couple of possible candidates are sampled / generated and filtered afterwards such that only a single candidate
 * per homotopy class remain. Filtering is applied using the H-Signature, a homotopy (resp. homology) invariant: \n
 *      - S. Bhattacharya et al.: Search-based Path Planning with Homotopy Class Constraints, AAAI, 2010
 *      - C. Rösmann et al.: Planning of Multiple Robot Trajectories in Distinctive Topologies, ECMR, 2015.
 * 
 * Followed by the homotopy class search, each candidate is used as an initialization for the underlying trajectory
 * optimization (in this case utilizing the TebOptimalPlanner class with the TimedElasticBand). \n
 * Depending on the config parameters, the optimization is performed in parallel. \n
 * After the optimization is completed, the best optimized candidate is selected w.r.t. to trajectory cost, since the 
 * cost already contains important features like clearance from obstacles and transition time. \n
 * 
 * Everyhting is performed by calling one of the overloaded plan() methods. \n
 * Afterwards the velocity command to control the robot is obtained from the "best" candidate 
 * via getVelocityCommand(). \n
 * 
 * All steps are repeated in the subsequent sampling interval with the exception, that already planned (optimized) trajectories
 * are preferred against new path initilizations in order to improve the hot-starting capability.
 */
class HomotopyClassPlanner : public PlannerInterface
{
public:

  /**
   * @brief Default constructor
   * 
   */
  HomotopyClassPlanner();
  
  /**
   * @brief Construct and initialize the HomotopyClassPlanner
   * @param cfg Const reference to the TebConfig class for internal parameters
   * @param obstacles Container storing all relevant obstacles (see Obstacle)
   * @param visualization Shared pointer to the TebVisualization class (optional)
   */
  HomotopyClassPlanner(const TebConfig& cfg, ObstContainer* obstacles = NULL, TebVisualizationPtr visualization = TebVisualizationPtr());
    
  /**
   * @brief Destruct the HomotopyClassPlanner.
   */
  virtual ~HomotopyClassPlanner();
  
  /**
   * @brief Initialize the HomotopyClassPlanner
   * @param cfg Const reference to the TebConfig class for internal parameters
   * @param obstacles Container storing all relevant obstacles (see Obstacle)
   * @param visualization Shared pointer to the TebVisualization class (optional)
   */
  void initialize(const TebConfig& cfg, ObstContainer* obstacles = NULL, TebVisualizationPtr visualization = TebVisualizationPtr());
  
 
  
  /** @name Plan a trajectory */
  //@{
  
  /**
   * @brief Plan a trajectory based on an initial reference plan.
   * 
   * Provide this method to create and optimize a trajectory that is initialized
   * according to an initial reference plan (given as a container of poses).
   * @warning The current implementation extracts only the start and goal pose and calls the overloaded plan()
   * @param initial_plan vector of geometry_msgs::PoseStamped (must be valid until clearPlanner() is called!)
   * @param start_vel Current start velocity (e.g. the velocity of the robot, only linear.x and angular.z are used)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *		      otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
  virtual bool plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel = NULL, bool free_goal_vel=false);
  
  /**
   * @brief Plan a trajectory between a given start and goal pose (tf::Pose version).
   * 
   * Provide this method to create and optimize a trajectory that is initialized between a given start and goal pose.
   * @param start tf::Pose containing the start pose of the trajectory
   * @param goal tf::Pose containing the goal pose of the trajectory
   * @param start_vel Current start velocity (e.g. the velocity of the robot, only linear.x and angular.z are used)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *		      otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
  virtual bool plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel = NULL, bool free_goal_vel=false);
  
  /**
   * @brief Plan a trajectory between a given start and goal pose.
   * 
   * Provide this method to create and optimize a trajectory that is initialized between a given start and goal pose.
   * @param start PoseSE2 containing the start pose of the trajectory
   * @param goal PoseSE2 containing the goal pose of the trajectory
   * @param start_vel Initial velocity at the start pose (2D vector containing the translational and angular velocity).
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *		      otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
  virtual bool plan(const PoseSE2& start, const PoseSE2& goal, const Eigen::Vector2d& start_vel, bool free_goal_vel=false);
  
  /**
   * @brief Get the velocity command from a previously optimized plan to control the robot at the current sampling interval.
   * @warning Call plan() first and check if the generated plan is feasible.
   * @param[out] v translational velocity [m/s]
   * @param[out] omega rotational velocity [rad/s]
   * @return \c true if command is valid, \c false otherwise
   */
  virtual bool getVelocityCommand(double& v, double& omega) const;
  
  /**
   * @brief Access current best trajectory candidate (that relates to the "best" homotopy class).
   * 
   * If no trajectory is available, the pointer will be empty.
   * If only a single trajectory is available, return it.
   * Otherwise return the best one, but call selectBestTeb() before to perform the actual selection (part of the plan() methods).
   * @return Shared pointer to the best TebOptimalPlanner that contains the selected trajectory (TimedElasticBand).
   */
  TebOptimalPlannerPtr bestTeb() const {return tebs_.empty() ? TebOptimalPlannerPtr() : tebs_.size()==1 ? tebs_.front() : best_teb_;}
    
  /**
   * @brief Check whether the planned trajectory is feasible or not.
   * 
   * This method currently checks only that the trajectory, or a part of the trajectory is collision free.
   * Obstacles are here represented as costmap instead of the internal ObstacleContainer.
   * @param costmap_model Pointer to the costmap model
   * @param footprint_spec The specification of the footprint of the robot in world coordinates
   * @param inscribed_radius The radius of the inscribed circle of the robot
   * @param circumscribed_radius The radius of the circumscribed circle of the robot
   * @param look_ahead_idx Number of poses along the trajectory that should be verified, if -1, the complete trajectory will be checked.
   * @return \c true, if the robot footprint along the first part of the trajectory intersects with 
   *         any obstacle in the costmap, \c false otherwise.
   */
  virtual bool isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                    double inscribed_radius = 0.0, double circumscribed_radius=0.0, int look_ahead_idx=-1);
  
  //@}
  
  /** @name Visualization */
  //@{
  
  /**
   * @brief Register a TebVisualization class to enable visiualization routines (e.g. publish the local plan and pose sequence)
   * @param visualization shared pointer to a TebVisualization instance
   * @see visualizeTeb
   */
  void setVisualization(TebVisualizationPtr visualization);
  
   /**
    * @brief Publish the local plan, pose sequence and additional information via ros topics (e.g. subscribe with rviz).
    * 
    * Make sure to register a TebVisualization instance before using setVisualization() or an overlaoded constructor.
    * @see setVisualization
    */
  virtual void visualize();
  
  //@}
  
  /** @name Important steps that are called during planning */
  //@{
    
 
  /**
   * @brief Explore paths in new homotopy classes and initialize TEBs from them.
   * 
   * This "all-in-one" method creates a graph with position keypoints from which
   * feasible paths (with clearance from obstacles) are extracted. \n
   * All obtained paths are filted to only keep a single path for each homotopy class. \n
   * Each time a new homotopy class is explored (by means of \b no previous trajectory/TEB remain in that homotopy class),
   * a new trajectory/TEB will be initialized. \n
   *
   * Everything is prepared now for the optimization step: see optimizeAllTEBs().
   * @param start Current start pose (e.g. pose of the robot)
   * @param goal Goal pose (e.g. robot's goal)
   * @param dist_to_obst Allowed distance to obstacles: if not satisfying, the path will be rejected (note, this is not the distance used for optimization).
   */
  void exploreHomotopyClassesAndInitTebs(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst);

  
  /**
   * @brief Check all available trajectories (TEBs) for detours and delete found ones.
   * @see TimedElasticBand::detectDetoursBackwards
   * @param threshold Threshold paramter for allowed orientation changes (below 0 -> greater than 90 deg) 
   */
  void deleteTebDetours(double threshold=0.0);
 
  
  /**
   * @brief Add a new Teb to the internal trajectory container and initialize it using a generic 2D reference path
   * 
   * Refer to TimedElasticBand::initTEBtoGoal() for more details about the template parameters.
   * @param path_start start iterator of a generic 2d path
   * @param path_end end iterator of a generic 2d path
   * @param fun_position unary function that returns the Eigen::Vector2d object
   * @param start_orientation Orientation of the first pose of the trajectory (optional, otherwise use goal heading)
   * @param goal_orientation Orientation of the last pose of the trajectory (optional, otherwise use goal heading)
   * @tparam BidirIter Bidirectional iterator type
   * @tparam Fun unyary function that transforms the dereferenced iterator into an Eigen::Vector2d
   */
  template<typename BidirIter, typename Fun>
  void addAndInitNewTeb(BidirIter path_start, BidirIter path_end, Fun fun_position, double start_orientation, double goal_orientation); 
  
  /**
   * @brief Add a new Teb to the internal trajectory container and initialize it with a simple straight line between a given start and goal
   * @param start start pose
   * @param goal goal pose
   */
  void addAndInitNewTeb(const PoseSE2& start, const PoseSE2& goal); 
  
    /**
   * @brief Add a new Teb to the internal trajectory container and initialize it using a PoseStamped container
   * @param initial_plan container of poses (start and goal orientation should be valid!)
   */
  void addAndInitNewTeb(const std::vector<geometry_msgs::PoseStamped>& initial_plan);
  
  /**
   * @brief Update TEBs with new pose, goal and current velocity.
   * @param start New start pose (optional)
   * @param goal New goal pose (optional)
   * @param start_velocity start velocity (optional)
   */
  void updateAllTEBs(boost::optional<const PoseSE2&> start, boost::optional<const PoseSE2&> goal,  boost::optional<const Eigen::Vector2d&> start_velocity);
  
  
  /**
   * @brief Optimize all available trajectories by invoking the optimizer on each one.
   * 
   * Depending on the configuration parameters, the optimization is performed either single or multi threaded.
   * @param iter_innerloop Number of inner iterations (see TebOptimalPlanner::optimizeTEB())
   * @param iter_outerloop Number of outer iterations (see TebOptimalPlanner::optimizeTEB())
   */
  void optimizeAllTEBs(unsigned int iter_innerloop, unsigned int iter_outerloop);
  
  /**
   * @brief In case of multiple, internally stored, alternative trajectories, select the best one according to their cost values.
   * 
   * The trajectory cost includes features such as transition time and clearance from obstacles. \n
   * The best trajectory can be accessed later by bestTeb() within the current sampling interval in order to avoid unessary recalculations.
   * @return Shared pointer to the best TebOptimalPlanner that contains the selected trajectory (TimedElasticBand).
   */
  TebOptimalPlannerPtr selectBestTeb();
  
  //@}
  
   /**
    * @brief Reset the planner.
    * 
    * Clear all previously found H-signatures, paths, tebs and the hcgraph.
    */
  void clearPlanner() {graph_.clear(); h_signatures_.clear(); tebs_.clear(); initial_plan_ = NULL;}
  
  /**
   * @brief Check if the planner suggests a shorter horizon (e.g. to resolve problems)
   * 
   * This method is intendend to be called after determining that a trajectory provided by the planner is infeasible.
   * In some cases a reduction of the horizon length might resolve problems. E.g. if a planned trajectory cut corners.
   * Implemented cases: see TebOptimalPlanner
   * @param initial_plan The intial and transformed plan (part of the local map and pruned up to the robot position)
   * @return \c true, if the planner suggests a shorter horizon, \c false otherwise.
   */
  virtual bool isHorizonReductionAppropriate(const std::vector<geometry_msgs::PoseStamped>& initial_plan) const;
  
  /**
   * @brief Calculate the H-Signature of a path
   * 
   * The H-Signature depends on the obstacle configuration and can be utilized
   * to check whether two trajectores rely to the same homotopy class.
   * Refer to: \n
   * 	- S. Bhattacharya et al.: Search-based Path Planning with Homotopy Class Constraints, AAAI, 2010
   * 
   * The implemented function accepts generic path descriptions that are restricted to the following structure: \n
   * The path is composed of points T and is represented by a std::vector< T > or similar type (std::list, std::deque, ...). \n
   * Provide a unary function with the following signature <c> std::complex< long double > (const T& point_type) </c>
   * that returns a complex value for the position (Re(*)=x, Im(*)=y).
   * 
   * T could also be a pointer type, if the passed function also accepts a const T* point_Type.
   * 
   * @param path_start Iterator to the first element in the path
   * @param path_end Iterator to the last element in the path
   * @param obstacles obstacle container
   * @param fun_cplx_point function accepting the dereference iterator type and that returns the position as complex number.
   * @param prescaler Change this value only if you observe problems with an huge amount of obstacles: interval (0,1]
   * @tparam BidirIter Bidirectional iterator type
   * @tparam Fun function of the form std::complex< long double > (const T& point_type)
   * @return complex H-Signature value
   */  
  template<typename BidirIter, typename Fun>
  static std::complex<long double> calculateHSignature(BidirIter path_start, BidirIter path_end, Fun fun_cplx_point, const ObstContainer* obstacles = NULL, double prescaler = 1);
  
  /**
   * @brief Read-only access to the internal trajectory container.
   * @return read-only reference to the teb container.
   */
  const TebOptPlannerContainer& getTrajectoryContainer() const {return tebs_;};
  
  /**
   * Compute and return the cost of the current optimization graph (supports multiple trajectories)
   * @param[out] cost current cost value for each trajectory
   *                  [for a planner with just a single trajectory: size=1, vector will not be cleared]
   */
  virtual void computeCurrentCost(std::vector<double>& cost);  
    
protected:
  
  /** @name Explore new paths and keep only a single one for each homotopy class */
  //@{
  
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
   */
  void createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold);
  
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
   */  
  void createProbRoadmapGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, int no_samples, double obstacle_heading_threshold);
  
  
  /**
   * @brief Internal helper function that adds a h-signature to the list of known h-signatures only if it is unique.
   * @param H h-signature that should be tested
   * @param threshold Two h-signuteres are assumed to be equal, if both the difference of real parts and complex parts are below \c threshold.
   * @return \c true if the h-signature was added and no duplicate was found, \c false otherwise
   */    
  bool addHSignatureIfNew(const std::complex<long double>& H, double threshold);

 
  /**
   * @brief Renew all found h-signatures for the new planning step based on existing TEBs. Optionally detours can be discarded.
   * 
   * Calling this method in each new planning interval is really important.
   * First all old h-signatures are deleted, since they could be invalid for this planning step (obstacle position may changed).
   * Afterwards the h-signatures are calculated for each existing TEB/trajectory and is inserted to the list of known h-signatures.
   * Doing this is important to prefer already optimized trajectories in contrast to initialize newly explored coarse paths.
   * @param delete_detours if this param is \c true, all existing TEBs are cleared from detour-candidates by utilizing deleteTebDetours(). 
   */
  void renewAndAnalyzeOldTebs(bool delete_detours);
  
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
   */
  void DepthFirst(HcGraph& g, std::vector<HcGraphVertexType>& visited, const HcGraphVertexType& goal, double start_orientation, double goal_orientation);
 
  /**
   * @brief Clear any existing graph of the homotopy class search
   */
  void clearGraph() {graph_.clear();}
  
  /**
   * @brief find the index of the currently best TEB in the container
   * @remarks bestTeb() should be preferred whenever possible
   * @return index of the best TEB obtained with bestTEB(), if no TEB is avaiable, it returns -1.
   */
  int bestTebIdx() const;
  
  //@}
  
    
  // external objects (store weak pointers)
  ObstContainer* obstacles_; //!< Store obstacles that are relevant for planning
  const TebConfig* cfg_; //!< Config class that stores and manages all related parameters
  
  // internal objects (memory management owned)
  TebVisualizationPtr visualization_; //!< Instance of the visualization class (local/global plan, obstacles, ...)
  TebOptimalPlannerPtr best_teb_; //!< Store the current best teb.
  
  const std::vector<geometry_msgs::PoseStamped>* initial_plan_; //!< Store the initial plan if available for a better trajectory initialization
  
  TebOptPlannerContainer tebs_; //!< Container that stores multiple local teb planners (for alternative homotopy classes) and their corresponding costs
  
  HcGraph graph_; //!< Store the graph that is utilized to find alternative homotopy classes.
 
  std::vector< std::complex<long double> > h_signatures_; //!< Store all known h-signatures to allow checking for duplicates after finding and adding new ones.
  
  boost::random::mt19937 rnd_generator_; //!< Random number generator used by createProbRoadmapGraph to sample graph keypoints.   
      
  bool initialized_; //!< Keeps track about the correct initialization of this class
  
  

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    
};

//! Abbrev. for a shared pointer of a HomotopyClassPlanner instance.
typedef boost::shared_ptr<HomotopyClassPlanner> HomotopyClassPlannerPtr;


} // namespace teb_local_planner

// include template implementations / definitions
#include <teb_local_planner/homotopy_class_planner.hpp>

#endif /* HOMOTOPY_CLASS_PLANNER_H_ */
