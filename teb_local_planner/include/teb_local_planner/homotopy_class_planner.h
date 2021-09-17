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
#include <iterator>
#include <memory>
#include <vector>
#include <iterator>
#include <random>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <rclcpp/rclcpp.hpp>

#include "teb_local_planner/planner_interface.h"
#include "teb_local_planner/teb_config.h"
#include "teb_local_planner/obstacles.h"
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/visualization.h"
#include "teb_local_planner/robot_footprint_model.h"
#include "teb_local_planner/equivalence_relations.h"
#include "teb_local_planner/graph_search.h"


namespace teb_local_planner
{

//!< Inline function used for calculateHSignature() in combination with VertexPose pointers
inline std::complex<long double> getCplxFromVertexPosePtr(const VertexPose* pose)
{
  return std::complex<long double>(pose->x(), pose->y());
};


//!< Inline function used for calculateHSignature() in combination with geometry_msgs::msg::PoseStamped
inline std::complex<long double> getCplxFromMsgPoseStamped(const geometry_msgs::msg::PoseStamped& pose)
{
  return std::complex<long double>(pose.pose.position.x, pose.pose.position.y);
};

/**
 * @class HomotopyClassPlanner
 * @brief Local planner that explores alternative homotopy classes, create a plan for each alternative
 *	  and finally return the robot controls for the current best path (repeated in each sampling interval)
 *
 * Equivalence classes (e.g. homotopy) are explored using the help of a search-graph. \n
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

  using EquivalenceClassContainer = std::vector< std::pair<EquivalenceClassPtr, bool> >;

  /**
   * @brief Default constructor
   */
  HomotopyClassPlanner();

  /**
   * @brief Construct and initialize the HomotopyClassPlanner
   * @param node Shared pointer for rclcpp::Node
   * @param cfg Const reference to the TebConfig class for internal parameters
   * @param obstacles Container storing all relevant obstacles (see Obstacle)
   * @param robot_model Shared pointer to the robot shape model used for optimization (optional)
   * @param visualization Shared pointer to the TebVisualization class (optional)
   * @param via_points Container storing via-points (optional)
   */
  HomotopyClassPlanner(nav2_util::LifecycleNode::SharedPtr node, const TebConfig& cfg, ObstContainer* obstacles = NULL,
                       TebVisualizationPtr visualization = TebVisualizationPtr(), const ViaPointContainer* via_points = NULL);

  /**
   * @brief Destruct the HomotopyClassPlanner.
   */
  virtual ~HomotopyClassPlanner();

  /**
   * @brief Initialize the HomotopyClassPlanner
   * @param node Shared pointer for rclcpp::Node
   * @param cfg Const reference to the TebConfig class for internal parameters
   * @param obstacles Container storing all relevant obstacles (see Obstacle)
   * @param visualization Shared pointer to the TebVisualization class (optional)
   * @param via_points Container storing via-points (optional)
   */
  void initialize(nav2_util::LifecycleNode::SharedPtr node, const TebConfig& cfg, ObstContainer* obstacles = NULL,
                  TebVisualizationPtr visualization = TebVisualizationPtr(), const ViaPointContainer* via_points = NULL);

  /** @name Plan a trajectory */
  //@{

  /**
   * @brief Plan a trajectory based on an initial reference plan.
   *
   * Provide this method to create and optimize a trajectory that is initialized
   * according to an initial reference plan (given as a container of poses).
   * @warning The current implementation extracts only the start and goal pose and calls the overloaded plan()
   * @param initial_plan vector of geometry_msgs::msg::PoseStamped (must be valid until clearPlanner() is called!)
   * @param start_vel Current start velocity (e.g. the velocity of the robot, only linear.x, linear.y (holonomic) and angular.z are used)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *		      otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
  virtual bool plan(const std::vector<geometry_msgs::msg::PoseStamped>& initial_plan, const geometry_msgs::msg::Twist* start_vel = NULL, bool free_goal_vel=false);

  /**
   * @brief Plan a trajectory between a given start and goal pose (tf::Pose version).
   *
   * Provide this method to create and optimize a trajectory that is initialized between a given start and goal pose.
   * @param start tf::Pose containing the start pose of the trajectory
   * @param goal tf::Pose containing the goal pose of the trajectory
   * @param start_vel Current start velocity (e.g. the velocity of the robot, only linear.x, linear.y (holonomic) and angular.z are used)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *		      otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
  // tf2 doesn't have tf::Pose
//  virtual bool plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::msg::Twist* start_vel = NULL, bool free_goal_vel=false);

  /**
   * @brief Plan a trajectory between a given start and goal pose.
   *
   * Provide this method to create and optimize a trajectory that is initialized between a given start and goal pose.
   * @param start PoseSE2 containing the start pose of the trajectory
   * @param goal PoseSE2 containing the goal pose of the trajectory
   * @param start_vel Initial velocity at the start pose (twist message containing the translational and angular velocity).
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *		      otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
  virtual bool plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::msg::Twist* start_vel = NULL, bool free_goal_vel=false);

  /**
   * @brief Get the velocity command from a previously optimized plan to control the robot at the current sampling interval.
   * @warning Call plan() first and check if the generated plan is feasible.
   * @param[out] vx translational velocity [m/s]
   * @param[out] vy strafing velocity which can be nonzero for holonomic robots [m/s]
   * @param[out] omega rotational velocity [rad/s]
   * @param[in] look_ahead_poses index of the final pose used to compute the velocity command.
   * @return \c true if command is valid, \c false otherwise
   */
  virtual bool getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const;

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
  virtual bool isTrajectoryFeasible(dwb_critics::ObstacleFootprintCritic* costmap_model, const std::vector<geometry_msgs::msg::Point>& footprint_spec,
                                    double inscribed_radius = 0.0, double circumscribed_radius=0.0, int look_ahead_idx=-1, double feasibility_check_lookahead_distance=-1.0);

  /**
   * @brief In case of empty best teb, scores again the available plans to find the best one.
   *        The best_teb_ variable is updated consequently.
   * @return Shared pointer to the best TebOptimalPlanner that contains the selected trajectory (TimedElasticBand).
   *         An empty pointer is returned if no plan is available.
   */
  TebOptimalPlannerPtr findBestTeb();

  /**
   * @brief Removes the specified teb and the corresponding homotopy class from the list of available ones.
   * @param pointer to the teb Band to be removed
   * @return Iterator to the next valid teb if available, else to the end of the tebs container.
   */
  TebOptPlannerContainer::iterator removeTeb(TebOptimalPlannerPtr& teb);

  //@}

  /** @name Visualization */
  //@{

  /**
   * @brief Register a TebVisualization class to enable visiualization routines (e.g. publish the local plan and pose sequence)
   * @param visualization shared pointer to a TebVisualization instance
   * @see visualizeTeb
   */
  void setVisualization(const TebVisualizationPtr & visualization) override;

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
   * @brief Explore paths in new equivalence classes (e.g. homotopy classes) and initialize TEBs from them.
   *
   * This "all-in-one" method creates a graph with position keypoints from which
   * feasible paths (with clearance from obstacles) are extracted. \n
   * All obtained paths are filted to only keep a single path for each equivalence class. \n
   * Each time a new equivalence class is explored (by means of \b no previous trajectory/TEB remain in that equivalence class),
   * a new trajectory/TEB will be initialized. \n
   *
   * Everything is prepared now for the optimization step: see optimizeAllTEBs().
   * @param start Current start pose (e.g. pose of the robot)
   * @param goal Goal pose (e.g. robot's goal)
   * @param dist_to_obst Allowed distance to obstacles: if not satisfying, the path will be rejected (note, this is not the distance used for optimization).
   * @param @param start_velocity start velocity (optional)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed, otherwise the final velocity will be zero (default: false)
   */
  void exploreEquivalenceClassesAndInitTebs(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, const geometry_msgs::msg::Twist* start_vel, bool free_goal_vel = false);

  /**
   * @brief Add a new Teb to the internal trajectory container, if this teb constitutes a new equivalence class. Initialize it using a generic 2D reference path
   *
   * Refer to TimedElasticBand::initTEBtoGoal() for more details about the template parameters.
   * @param path_start start iterator of a generic 2d path
   * @param path_end end iterator of a generic 2d path
   * @param fun_position unary function that returns the Eigen::Vector2d object
   * @param start_orientation Orientation of the first pose of the trajectory (optional, otherwise use goal heading)
   * @param goal_orientation Orientation of the last pose of the trajectory (optional, otherwise use goal heading)
   * @param start_velocity start velocity (optional)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed, otherwise the final velocity will be zero (default: false)
   * @tparam BidirIter Bidirectional iterator type
   * @tparam Fun unyary function that transforms the dereferenced iterator into an Eigen::Vector2d
   * @return Shared pointer to the newly created teb optimal planner
   */
  template<typename BidirIter, typename Fun>
  TebOptimalPlannerPtr addAndInitNewTeb(BidirIter path_start, BidirIter path_end, Fun fun_position, double start_orientation, double goal_orientation, const geometry_msgs::msg::Twist* start_velocity, bool free_goal_vel = false);

  /**
   * @brief Add a new Teb to the internal trajectory container, if this teb constitutes a new equivalence class. Initialize it with a simple straight line between a given start and goal
   * @param start start pose
   * @param goal goal pose
   * @param start_velocity start velocity (optional)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed, otherwise the final velocity will be zero (default: false)
   * @return Shared pointer to the newly created teb optimal planner
   */
  TebOptimalPlannerPtr addAndInitNewTeb(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::msg::Twist* start_velocity, bool free_goal_vel = false);

  /**
   * @brief Add a new Teb to the internal trajectory container , if this teb constitutes a new equivalence class. Initialize it using a PoseStamped container
   * @param initial_plan container of poses (start and goal orientation should be valid!)
   * @param start_velocity start velocity (optional)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed, otherwise the final velocity will be zero (default: false)
   * @return Shared pointer to the newly created teb optimal planner
   */
  TebOptimalPlannerPtr addAndInitNewTeb(const std::vector<geometry_msgs::msg::PoseStamped>& initial_plan, const geometry_msgs::msg::Twist* start_velocity, bool free_goal_vel = false);

  /**
   * @brief Update TEBs with new pose, goal and current velocity.
   * @param start New start pose (optional)
   * @param goal New goal pose (optional)
   * @param start_velocity start velocity (optional)
   */
  void updateAllTEBs(const PoseSE2* start, const PoseSE2* goal, const geometry_msgs::msg::Twist* start_velocity);


  /**
   * @brief Optimize all available trajectories by invoking the optimizer on each one.
   *
   * Depending on the configuration parameters, the optimization is performed either single or multi threaded.
   * @param iter_innerloop Number of inner iterations (see TebOptimalPlanner::optimizeTEB())
   * @param iter_outerloop Number of outer iterations (see TebOptimalPlanner::optimizeTEB())
   */
  void optimizeAllTEBs(int iter_innerloop, int iter_outerloop);

  /**
   * @brief Returns a shared pointer to the TEB related to the initial plan
   * @return A non-empty shared ptr is returned if a match was found; Otherwise the shared ptr is empty.
   */
  TebOptimalPlannerPtr getInitialPlanTEB();

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
  virtual void clearPlanner() {clearGraph(); equivalence_classes_.clear(); tebs_.clear(); initial_plan_ = NULL;}


  /**
   * @brief Prefer a desired initial turning direction (by penalizing the opposing one)
   *
   * A desired (initial) turning direction might be specified in case the planned trajectory oscillates between two
   * solutions (in the same equivalence class!) with similar cost. Check the parameters in order to adjust the weight of the penalty.
   * Initial means that the penalty is applied only to the first few poses of the trajectory.
   * @param dir This parameter might be RotType::left (prefer left), RotType::right (prefer right) or RotType::none (prefer none)
   */
  virtual void setPreferredTurningDir(RotType dir);

  /**
   * @brief Calculate the equivalence class of a path
   *
   * Currently, only the H-signature (refer to HSignature) is implemented.
   *
   * @param path_start Iterator to the first element in the path
   * @param path_end Iterator to the last element in the path
   * @param obstacles obstacle container
   * @param fun_cplx_point function accepting the dereference iterator type and that returns the position as complex number.
   * @tparam BidirIter Bidirectional iterator type
   * @tparam Fun function of the form std::complex< long double > (const T& point_type)
   * @return pointer to the equivalence class base type
   */
  template<typename BidirIter, typename Fun>
  EquivalenceClassPtr calculateEquivalenceClass(BidirIter path_start, BidirIter path_end, Fun fun_cplx_point, const ObstContainer* obstacles = NULL,
                                                boost::optional<TimeDiffSequence::iterator> timediff_start = boost::none, boost::optional<TimeDiffSequence::iterator> timediff_end = boost::none);

  /**
   * @brief Read-only access to the internal trajectory container.
   * @return read-only reference to the teb container.
   */
  const TebOptPlannerContainer& getTrajectoryContainer() const {return tebs_;}

  bool hasDiverged() const override;

  /**
   * Compute and return the cost of the current optimization graph (supports multiple trajectories)
   * @param[out] cost current cost value for each trajectory
   *                  [for a planner with just a single trajectory: size=1, vector will not be cleared]
   * @param obst_cost_scale Specify extra scaling for obstacle costs
   * @param viapoint_cost_scale Specify extra scaling for via points.
   * @param alternative_time_cost Replace the cost for the time optimal objective by the actual (weighted) transition time
   */
  virtual void computeCurrentCost(std::vector<double>& cost, double obst_cost_scale=1.0, double viapoint_cost_scale=1.0, bool alternative_time_cost=false);

  /**
   * @brief Check if two h-signatures are similar (w.r.t. a certain threshold)
   * @param h1 first h-signature
   * @param h2 second h-signature
   * @return \c true if both h-signatures are similar, false otherwise.
   */
  inline static bool isHSignatureSimilar(const std::complex<long double>& h1, const std::complex<long double>& h2, double threshold)
  {
      double diff_real = std::abs(h2.real() - h1.real());
      double diff_imag = std::abs(h2.imag() - h1.imag());
      if (diff_real<=threshold && diff_imag<=threshold)
        return true; // Found! Homotopy class already exists, therefore nothing added
      return false;
  }
  /**
   * @brief Checks if the orientation of the computed trajectories differs from that of the best plan of more than the
   *  specified threshold and eventually deletes them.
   *  Also deletes detours with a duration much bigger than the duration of the best_teb (duration / best duration > max_ratio_detours_duration_best_duration).
   * @param orient_threshold: Threshold paramter for allowed orientation changes in radians
   * @param len_orientation_vector: length of the vector used to compute the start orientation
   */
  void deletePlansDetouringBackwards(const double orient_threshold, const double len_orientation_vector);
  /**
   * @brief Given a plan, computes its start orientation using a vector of length >= len_orientation_vector
   *        starting from the initial pose.
   * @param plan: Teb to be analyzed
   * @param len_orientation_vector: min length of the vector used to compute the start orientation
   * @param orientation: computed start orientation
   * @return: Could the vector for the orientation check be computed? (False if the plan has no pose with a distance
   *          > len_orientation_vector from the start poseq)
   */
  bool computeStartOrientation(const TebOptimalPlannerPtr plan, const double len_orientation_vector, double& orientation);


  /**
   * @brief Access config (read-only)
   * @return const pointer to the config instance
   */
  const TebConfig* config() const {return cfg_;}

  /**
   * @brief Access current obstacle container (read-only)
   * @return const pointer to the obstacle container instance
   */
  const ObstContainer* obstacles() const {return obstacles_;}

  /**
   * @brief Returns true if the planner is initialized
   */
  bool isInitialized() const {return initialized_;}

  /**
   * @brief Clear any existing graph of the homotopy class search
   */
  void clearGraph() {if(graph_search_) graph_search_->clearGraph();}

  /**
   * @brief find the index of the currently best TEB in the container
   * @remarks bestTeb() should be preferred whenever possible
   * @return index of the best TEB obtained with bestTEB(), if no TEB is avaiable, it returns -1.
   */
  int bestTebIdx() const;


  /**
   * @brief Internal helper function that adds a new equivalence class to the list of known classes only if it is unique.
   * @param eq_class equivalence class that should be tested
   * @param lock if \c true, exclude the H-signature from deletion.
   * @return \c true if the h-signature was added and no duplicate was found, \c false otherwise
   */
  bool addEquivalenceClassIfNew(const EquivalenceClassPtr& eq_class, bool lock=false);

  /**
   * @brief Return the current set of equivalence erelations (read-only)
   * @return reference to the internal set of currently tracked equivalence relations
   */
  const EquivalenceClassContainer& getEquivalenceClassRef() const  {return equivalence_classes_;}

  bool isInBestTebClass(const EquivalenceClassPtr& eq_class) const;

  int numTebsInClass(const EquivalenceClassPtr& eq_class) const;

  int numTebsInBestTebClass() const;

  /**
   * @brief Randomly drop non-optimal TEBs to so we can explore other alternatives
   *
   * The HCP has a tendency to become "fixated" once its tebs_ list becomes
   * fully populated, repeatedly refining and evaluating paths from the same
   * few homotopy classes until the robot moves far enough for a teb to become
   * invalid. As a result, it can fail to discover a more optimal path. This
   * function alleviates this problem by randomly dropping TEBs other than the
   * current "best" one with a probability controlled by
   * selection_dropping_probability parameter.
   */
  void randomlyDropTebs();

protected:

  /** @name Explore new paths and keep only a single one for each homotopy class */
  //@{

  /**
   * @brief Check if a h-signature exists already.
   * @param eq_class equivalence class that should be tested
   * @return \c true if the h-signature is found, \c false otherwise
   */
  bool hasEquivalenceClass(const EquivalenceClassPtr& eq_class) const;


  /**
   * @brief Renew all found h-signatures for the new planning step based on existing TEBs. Optionally detours can be discarded.
   *
   * Calling this method in each new planning interval is really important.
   * First all old h-signatures are deleted, since they could be invalid for this planning step (obstacle position may changed).
   * Afterwards the h-signatures are calculated for each existing TEB/trajectory and is inserted to the list of known h-signatures.
   * Doing this is important to prefer already optimized trajectories in contrast to initialize newly explored coarse paths.
   * @param delete_detours if this param is \c true, all existing TEBs are cleared from detour-candidates calling deletePlansGoingBackwards().
   */
  void renewAndAnalyzeOldTebs(bool delete_detours);

  /**
   * @brief Associate trajectories with via-points
   *
   * If \c all_trajectories is true, all trajectory candidates are connected with the set of via_points,
   * otherwise only the trajectory sharing the homotopy class of the initial/global plan (and all via-points from alternative trajectories are removed)
   * @remarks Requires that the plan method is called with an initial plan provided and that via-points are enabled (config)
   * @param all_trajectories see method description
   */
  void updateReferenceTrajectoryViaPoints(bool all_trajectories);

  //@}

  // external objects (store weak pointers)
  const TebConfig* cfg_; //!< Config class that stores and manages all related parameters
  ObstContainer* obstacles_; //!< Store obstacles that are relevant for planning
  const ViaPointContainer* via_points_; //!< Store the current list of via-points

  // internal objects (memory management owned)
  TebVisualizationPtr visualization_; //!< Instance of the visualization class (local/global plan, obstacles, ...)
  TebOptimalPlannerPtr best_teb_; //!< Store the current best teb.
  EquivalenceClassPtr best_teb_eq_class_; //!< Store the equivalence class of the current best teb
  RobotFootprintModelPtr robot_model_; //!< Robot model shared instance

  const std::vector<geometry_msgs::msg::PoseStamped>* initial_plan_; //!< Store the initial plan if available for a better trajectory initialization
  EquivalenceClassPtr initial_plan_eq_class_; //!< Store the equivalence class of the initial plan
  TebOptimalPlannerPtr initial_plan_teb_; //!< Store pointer to the TEB related to the initial plan (use method getInitialPlanTEB() since it checks if initial_plan_teb_ is still included in tebs_.)

  TebOptPlannerContainer tebs_; //!< Container that stores multiple local teb planners (for alternative equivalence classes) and their corresponding costs

  EquivalenceClassContainer equivalence_classes_; //!< Store all known quivalence classes (e.g. h-signatures) to allow checking for duplicates after finding and adding new ones.
                                                                            //   The second parameter denotes whether to exclude the class from detour deletion or not (true: force keeping).

  std::shared_ptr<GraphSearchInterface> graph_search_;

  rclcpp::Time last_eq_class_switching_time_; //!< Store the time at which the equivalence class changed recently

  std::default_random_engine random_;
  bool initialized_; //!< Keeps track about the correct initialization of this class

  TebOptimalPlannerPtr last_best_teb_;  //!< Points to the plan used in the previous control cycle



public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


};

//! Abbrev. for a shared pointer of a HomotopyClassPlanner instance.
typedef std::shared_ptr<HomotopyClassPlanner> HomotopyClassPlannerPtr;


} // namespace teb_local_planner

// include template implementations / definitions
#include "teb_local_planner/homotopy_class_planner.hpp"

#endif /* HOMOTOPY_CLASS_PLANNER_H_ */
