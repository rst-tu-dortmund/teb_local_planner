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

#ifndef TIMED_ELASTIC_BAND_H_
#define TIMED_ELASTIC_BAND_H_
#include <boost/optional.hpp>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/buffer_core.h>

#include <cassert>
#include <complex>
#include <iterator>

#include "teb_local_planner/obstacles.h"

// G2O Types
#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/g2o_types/vertex_timediff.h"


namespace teb_local_planner
{

//! Container of poses that represent the spatial part of the trajectory
typedef std::vector<VertexPose*> PoseSequence;
//! Container of time differences that define the temporal of the trajectory
typedef std::vector<VertexTimeDiff*> TimeDiffSequence;


/**
 * @class TimedElasticBand
 * @brief Class that defines a trajectory modeled as an elastic band with augmented tempoarl information.
 * 
 * All trajectory related methods (initialization, modifying, ...) are implemented inside this class. \n
 * Let \f$ Q = \lbrace \mathbf{s}_i \rbrace_{i=0...n},\ n \in \mathbb{N} \f$ be a sequence of poses, \n
 * in which \f$ \mathbf{s}_i = [x_i, y_i, \beta_i]^T \in \mathbb{R}^2 \times S^1 \f$ denotes a single pose of the robot. \n
 * The Timed Elastic Band (TEB) augments this sequence of poses by incorporating time intervals between
 * two consecutive poses, resuting in a sequence of \c n-1 time intervals \f$ \Delta T_i \f$: \n
 * \f$ \tau = \lbrace \Delta T_i \rbrace_{i=0...n-1} \f$. \n
 * Each time interval (time diff) denotes the time that the robot requires to transit from the current configuration to the next configuration.
 * The tuple of both sequences defines the underlying trajectory.
 * 
 * Poses and time differences are wrapped into a g2o::Vertex class in order to enable the efficient optimization in TebOptimalPlanner. \n
 * TebOptimalPlanner utilizes this Timed_Elastic_band class for representing an optimizable trajectory.
 * 
 * @todo Move decision if the start or goal state should be marked as fixed or unfixed for the optimization to the TebOptimalPlanner class.
 */
class TimedElasticBand
{
public:

  /**
   * @brief Construct the class
   */
  TimedElasticBand();
  
  /**
   * @brief Destruct the class
   */
  virtual ~TimedElasticBand();

  
  
  /** @name Access pose and timediff sequences */
  //@{
  
  /**
   * @brief Access the complete pose sequence 
   * @return reference to the pose sequence
   */
  PoseSequence& poses() {return pose_vec_;};
  
   /**
   * @brief Access the complete pose sequence (read-only)
   * @return const reference to the pose sequence
   */
  const PoseSequence& poses() const {return pose_vec_;};
  
  /**
   * @brief Access the complete timediff sequence 
   * @return reference to the dimediff sequence
   */
  TimeDiffSequence& timediffs() {return timediff_vec_;};
  
  /**
   * @brief Access the complete timediff sequence 
   * @return reference to the dimediff sequence
   */
  const TimeDiffSequence& timediffs() const {return timediff_vec_;};  
  
  /**
   * @brief Access the time difference at pos \c index of the time sequence
   * @param index element position inside the internal TimeDiffSequence
   * @return reference to the time difference at pos \c index
   */
  double& TimeDiff(int index)
  {
    assert(index<sizeTimeDiffs());
    return timediff_vec_.at(index)->dt();
  }
  
  /**
   * @brief Access the time difference at pos \c index of the time sequence (read-only)
   * @param index element position inside the internal TimeDiffSequence
   * @return const reference to the time difference at pos \c index
   */
  const double& TimeDiff(int index) const
  {
    assert(index<sizeTimeDiffs());
    return timediff_vec_.at(index)->dt();
  }
  
  /**
   * @brief Access the pose at pos \c index of the pose sequence
   * @param index element position inside the internal PoseSequence
   * @return reference to the pose at pos \c index
   */
  PoseSE2& Pose(int index) 
  {
    assert(index<sizePoses());
    return pose_vec_.at(index)->pose();
  }
  
  /**
   * @brief Access the pose at pos \c index of the pose sequence (read-only)
   * @param index element position inside the internal PoseSequence
   * @return const reference to the pose at pos \c index
   */
  const PoseSE2& Pose(int index) const 
  {
    assert(index<sizePoses());
    return pose_vec_.at(index)->pose();
  }

  /**
   * @brief Access the last PoseSE2 in the pose sequence
   */
  PoseSE2& BackPose() {return pose_vec_.back()->pose(); }
  
  /**
   * @brief Access the last PoseSE2 in the pose sequence (read-only) 
   */
  const PoseSE2& BackPose() const {return pose_vec_.back()->pose();}
  
  /**
   * @brief Access the last TimeDiff in the time diff sequence
   */ 
  double& BackTimeDiff() {return timediff_vec_.back()->dt(); }
  
  /**
   * @brief Access the last TimeDiff in the time diff sequence (read-only)
   */  
  const double& BackTimeDiff() const {return timediff_vec_.back()->dt(); }
  
  /**
   * @brief Access the vertex of a pose at pos \c index for optimization purposes
   * @param index element position inside the internal PoseSequence
   * @return Weak raw pointer to the pose vertex at pos \c index
   */ 
  VertexPose* PoseVertex(int index) 
  {
    assert(index<sizePoses());
    return pose_vec_.at(index);
  }
  
  /**
   * @brief Access the vertex of a time difference at pos \c index for optimization purposes
   * @param index element position inside the internal TimeDiffSequence
   * @return Weak raw pointer to the timediff vertex at pos \c index
   */  
  VertexTimeDiff* TimeDiffVertex(int index) 
  {
    assert(index<sizeTimeDiffs());
    return timediff_vec_.at(index);
  }
  
  //@}
  
  
  
  /** @name Append new elements to the pose and timediff sequences */
  //@{
  
  /**
   * @brief Append a new pose vertex to the back of the pose sequence 
   * @param pose PoseSE2 to push back on the internal PoseSequence
   * @param fixed Mark the pose to be fixed or unfixed during trajectory optimization (important for the TebOptimalPlanner)
   */
  void addPose(const PoseSE2& pose, bool fixed=false);  

  /**
   * @brief Append a new pose vertex to the back of the pose sequence 
   * @param position 2D vector representing the position part
   * @param theta yaw angle representing the orientation part
   * @param fixed Mark the pose to be fixed or unfixed during trajectory optimization (important for the TebOptimalPlanner)
   */
  void addPose(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, bool fixed=false);
  
  /**
   * @brief Append a new pose vertex to the back of the pose sequence 
   * @param x x-coordinate of the position part
   * @param y y-coordinate of the position part
   * @param theta yaw angle representing the orientation part
   * @param fixed Mark the pose to be fixed or unfixed during trajectory optimization (important for the TebOptimalPlanner)
   */
  void addPose(double x, double y, double theta, bool fixed=false);
  
  /**
   * @brief Append a new time difference vertex to the back of the time diff sequence 
   * @param dt time difference value to push back on the internal TimeDiffSequence
   * @param fixed Mark the pose to be fixed or unfixed during trajectory optimization (important for the TebOptimalPlanner)
   */
  void addTimeDiff(double dt, bool fixed=false);
  
  /**
   * @brief Append a (pose, timediff) vertex pair to the end of the current trajectory (pose and timediff sequences)
   * @param pose PoseSE2 to push back on the internal PoseSequence
   * @param dt time difference value to push back on the internal TimeDiffSequence
   * @warning 	Since the timediff is defined to connect two consecutive poses, this call is only 
   * 		allowed if there exist already n poses and n-1 timediffs in the sequences (n=1,2,...):
   * 		therefore add a single pose using addPose() first!
   */
  void addPoseAndTimeDiff(const PoseSE2& pose, double dt);
  
  /**
   * @brief Append a (pose, timediff) vertex pair to the end of the current trajectory (pose and timediff sequences)
   * @param position 2D vector representing the position part
   * @param theta yaw angle representing the orientation part
   * @param dt time difference value to push back on the internal TimeDiffSequence
   * @warning see addPoseAndTimeDiff(const PoseSE2& pose, double dt)
   */
  void addPoseAndTimeDiff(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, double dt);
  
  /**
   * @brief Append a (pose, timediff) vertex pair to the end of the current trajectory (pose and timediff sequences)
   * @param x x-coordinate of the position part
   * @param y y-coordinate of the position part
   * @param theta yaw angle representing the orientation part
   * @param dt time difference value to push back on the internal TimeDiffSequence
   * @warning see addPoseAndTimeDiff(const PoseSE2& pose, double dt)
   */
  void addPoseAndTimeDiff(double x, double y, double theta, double dt);
  
  //@}
  

  /** @name Insert new elements and remove elements of the pose and timediff sequences */
  //@{

  /**
   * @brief Insert a new pose vertex at pos. \c index to the pose sequence
   * @param index element position inside the internal PoseSequence
   * @param pose PoseSE2 element to insert into the internal PoseSequence
   */
  void insertPose(int index, const PoseSE2& pose);
  
  /**
   * @brief Insert a new pose vertex at pos. \c index to the pose sequence
   * @param index element position inside the internal PoseSequence
   * @param position 2D vector representing the position part
   * @param theta yaw-angle representing the orientation part
   */
  void insertPose(int index, const Eigen::Ref<const Eigen::Vector2d>& position, double theta);
  
  /**
   * @brief Insert a new pose vertex at pos. \c index to the pose sequence
   * @param index element position inside the internal PoseSequence
   * @param x x-coordinate of the position part
   * @param y y-coordinate of the position part
   * @param theta yaw-angle representing the orientation part
   */
  void insertPose(int index, double x, double y, double theta);
  
  /**
   * @brief Insert a new timediff vertex at pos. \c index to the timediff sequence
   * @param index element position inside the internal TimeDiffSequence
   * @param dt timediff value
   */   
  void insertTimeDiff(int index, double dt);
    
  /**
   * @brief Delete pose at pos. \c index in the pose sequence
   * @param index element position inside the internal PoseSequence
   */
  void deletePose(int index);
  
  /**
   * @brief Delete multiple (\c number) poses starting at pos. \c index in the pose sequence
   * @param index first element position inside the internal PoseSequence
   * @param number number of elements that should be deleted
   */
  void deletePoses(int index, int number);

  /**
   * @brief Delete pose at pos. \c index in the timediff sequence
   * @param index element position inside the internal TimeDiffSequence
   */
  void deleteTimeDiff(int index);
	
  /**
   * @brief Delete multiple (\c number) time differences starting at pos. \c index in the timediff sequence
   * @param index first element position inside the internal TimeDiffSequence
   * @param number number of elements that should be deleted
   */
  void deleteTimeDiffs(int index, int number);
  
  //@}
  
  
  /** @name Init the trajectory */
  //@{
  
  /**
   * @brief Initialize a trajectory between a given start and goal pose.
   * 
   * The implemented algorithm subsamples the straight line between
   * start and goal using a given discretiziation width. \n
   * The discretization width can be defined in the euclidean space
   * using the \c diststep parameter. Each time difference between two consecutive
   * poses is initialized to \c timestep. \n
   * If the \c diststep is chosen to be zero, 
   * the resulting trajectory contains the start and goal pose only.
   * @param start PoseSE2 defining the start of the trajectory
   * @param goal PoseSE2 defining the goal of the trajectory (final pose)
   * @param diststep euclidean distance between two consecutive poses (if 0, no intermediate samples are inserted despite min_samples)
   * @param max_vel_x maximum translational velocity used for determining time differences
   * @param min_samples Minimum number of samples that should be initialized at least
   * @param guess_backwards_motion Allow the initialization of backwards oriented trajectories if the goal heading is pointing behind the robot
   * @return true if everything was fine, false otherwise
   */
  bool initTrajectoryToGoal(const PoseSE2& start, const PoseSE2& goal, double diststep=0, double max_vel_x=0.5, int min_samples = 3, bool guess_backwards_motion = false);
  
  
  /**
   * @brief Initialize a trajectory from a generic 2D reference path.
   * 
   * The temporal information is determined using a given maximum velocity 
   * (2D vector containing the translational and angular velocity). \n
   * A constant velocity profile is implemented. \n
   * A possible maximum acceleration is considered if \c max_acceleration param is provided.
   * 
   * Since the orientation is not included in the reference path, it can be provided seperately
   * (e.g. from the robot pose and robot goal). \n
   * Otherwise the goal heading will be used as start and goal orientation. \n
   * The orientation along the trajectory will be determined using the connection vector
   * between two consecutive positions of the reference path.
   * 
   * The reference path is provided using a start and end iterator to a container class.
   * You must provide a unary function that accepts the dereferenced iterator and returns
   * a copy or (const) reference to an Eigen::Vector2d type containing the 2d position.
   * 
   * @param path_start start iterator of a generic 2d path
   * @param path_end end iterator of a generic 2d path
   * @param fun_position unary function that returns the Eigen::Vector2d object
   * @param max_vel_x maximum translational velocity used for determining time differences
   * @param max_vel_theta maximum angular velocity used for determining time differences
   * @param max_acc_x specify to satisfy a maxmimum transl. acceleration and decceleration (optional)
   * @param max_acc_theta specify to satisfy a maxmimum angular acceleration and decceleration (optional)
   * @param start_orientation Orientation of the first pose of the trajectory (optional, otherwise use goal heading)
   * @param goal_orientation Orientation of the last pose of the trajectory (optional, otherwise use goal heading)
   * @param min_samples Minimum number of samples that should be initialized at least
   * @param guess_backwards_motion Allow the initialization of backwards oriented trajectories if the goal heading is pointing behind the robot
   * @tparam BidirIter Bidirectional iterator type
   * @tparam Fun unyary function that transforms the dereferenced iterator into an Eigen::Vector2d
   * @return true if everything was fine, false otherwise
   * @remarks Use \c boost::none to skip optional arguments.
   */ 
  template<typename BidirIter, typename Fun>
  bool initTrajectoryToGoal(BidirIter path_start, BidirIter path_end, Fun fun_position, double max_vel_x, double max_vel_theta,
		      boost::optional<double> max_acc_x, boost::optional<double> max_acc_theta,
		      boost::optional<double> start_orientation, boost::optional<double> goal_orientation, int min_samples = 3, bool guess_backwards_motion = false);  
  
  /**
   * @brief Initialize a trajectory from a reference pose sequence (positions and orientations).
   *
   * This method initializes the timed elastic band using a pose container
   * (e.g. as local plan from the ros navigation stack). \n
   * The initial time difference between two consecutive poses can be uniformly set
   * via the argument \c dt.
   * @param plan vector of geometry_msgs::msg::PoseStamped
   * @param max_vel_x maximum translational velocity used for determining time differences
   * @param max_vel_theta maximum rotational velocity used for determining time differences
   * @param estimate_orient if \c true, calculate orientation using the straight line distance vector between consecutive poses
   *                        (only copy start and goal orientation; recommended if no orientation data is available).
   * @param min_samples Minimum number of samples that should be initialized at least
   * @param guess_backwards_motion Allow the initialization of backwards oriented trajectories if the goal heading is pointing behind the robot (this parameter is used only if \c estimate_orient is enabled.
   * @return true if everything was fine, false otherwise
   */
  bool initTrajectoryToGoal(const std::vector<geometry_msgs::msg::PoseStamped>& plan, double max_vel_x, double max_vel_theta, bool estimate_orient=false, int min_samples = 3, bool guess_backwards_motion = false);
  
  //@}
  
  /** @name Update and modify the trajectory */
  //@{
  
  
  /**
   * @brief Hot-Start from an existing trajectory with updated start and goal poses.
   *
   * This method updates a previously optimized trajectory with a new start and/or a new goal pose. \n
   * The current simple implementation cuts of pieces of the trajectory that are already passed due to the new start. \n
   * Afterwards the start and goal pose are replaced by the new ones. The resulting discontinuity will not be smoothed.
   * The optimizer has to smooth the trajectory in TebOptimalPlanner. \n
   * 
   * @todo Smooth the trajectory here and test the performance improvement of the optimization.
   * @todo Implement a updateAndPruneTEB based on a new reference path / pose sequence.
   * 
   * @param new_start New start pose (optional)
   * @param new_goal New goal pose (optional)
   * @param min_samples Specify the minimum number of samples that should at least remain in the trajectory
   */  
  void updateAndPruneTEB(boost::optional<const PoseSE2&> new_start, boost::optional<const PoseSE2&> new_goal, int min_samples = 3);
  
  
  /**
   * @brief Resize the trajectory by removing or inserting a (pose,dt) pair depending on a reference temporal resolution.
   *
   * Resizing the trajectory is helpful e.g. for the following scenarios:
   * 
   * 	- Obstacles requires the teb to be extended in order to 
   * 	  satisfy the given discritization width (plan resolution)
   *      and to avoid undesirable behavior due to a large/small discretization step widths \f$ \Delta T_i \f$
   * 	  After clearance of obstacles, the teb should (re-) contract to its (time-)optimal version.
   *    - If the distance to the goal state is getting smaller,
   *      dt is decreasing as well. This leads to a heavily
   *      fine-grained discretization in combination with many
   *      discrete poses. Thus, the computation time will
   *      be/remain high and in addition numerical instabilities
   *      can appear (e.g. due to the division by a small \f$ \Delta T_i \f$).
   *
   * The implemented strategy checks all timediffs \f$ \Delta T_i \f$ and
   * 
   * 	- inserts a new sample if \f$ \Delta T_i > \Delta T_{ref} + \Delta T_{hyst} \f$
   *    - removes a sample if \f$ \Delta T_i < \Delta T_{ref} - \Delta T_{hyst} \f$
   * 
   * Each call only one new sample (pose-dt-pair) is inserted or removed.
   * @param dt_ref reference temporal resolution
   * @param dt_hysteresis hysteresis to avoid oscillations
   * @param min_samples minimum number of samples that should be remain in the trajectory after resizing
   * @param max_samples maximum number of samples that should not be exceeded during resizing
   * @param fast_mode if true, the trajectory is iterated once to insert or erase points; if false the trajectory
   *                  is repeatedly iterated until no poses are added or removed anymore
   */    
  void autoResize(double dt_ref, double dt_hysteresis, int min_samples = 3, int max_samples=1000, bool fast_mode=false);

  /**
   * @brief Set a pose vertex at pos \c index of the pose sequence to be fixed or unfixed during optimization.
   * @param index index to the pose vertex
   * @param status if \c true, the vertex will be fixed, otherwise unfixed
   */
  void setPoseVertexFixed(int index, bool status);
  
  /**
   * @brief Set a timediff vertex at pos \c index of the timediff sequence to be fixed or unfixed during optimization.
   * @param index index to the timediff vertex
   * @param status if \c true, the vertex will be fixed, otherwise unfixed
   */
  void setTimeDiffVertexFixed(int index, bool status);
  
  /**
   * @brief clear all poses and timediffs from the trajectory.
   * The pose and timediff sequences will be empty and isInit() will return \c false
   */
  void clearTimedElasticBand();
  
  //@}
  
  
  /** @name Utility and status methods */
  //@{
  
  /**
   * @brief Find the closest point on the trajectory w.r.t. to a provided reference point.
   * 
   * This function can be useful to find the part of a trajectory that is close to an obstacle.
   * 
   * @todo implement a more efficient version that first performs a coarse search.
   * @todo implement a fast approximation that assumes only one local minima w.r.t to the distance:
   *       Allows simple comparisons starting from the middle of the trajectory.
   * 
   * @param ref_point reference point (2D position vector)
   * @param[out] distance [optional] the resulting minimum distance
   * @param begin_idx start search at this pose index
   * @return Index to the closest pose in the pose sequence
   */
  int findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_point, double* distance = NULL, int begin_idx=0) const;

  /**
   * @brief Find the closest point on the trajectory w.r.t. to a provided reference line.
   * 
   * This function can be useful to find the part of a trajectory that is close to an (line) obstacle.
   * 
   * @todo implement a more efficient version that first performs a coarse search.
   * @todo implement a fast approximation that assumes only one local minima w.r.t to the distance:
   *       Allows simple comparisons starting from the middle of the trajectory.
   * 
   * @param ref_line_start start of the reference line (2D position vector)
	 * @param ref_line_end end of the reference line (2D position vector)
   * @param[out] distance [optional] the resulting minimum distance
   * @return Index to the closest pose in the pose sequence
   */
  int findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_line_start, const Eigen::Ref<const Eigen::Vector2d>& ref_line_end, double* distance = NULL) const;

  /**
   * @brief Find the closest point on the trajectory w.r.t. to a provided reference polygon.
   * 
   * This function can be useful to find the part of a trajectory that is close to an (polygon) obstacle.
   * 
   * @todo implement a more efficient version that first performs a coarse search.
   * @todo implement a fast approximation that assumes only one local minima w.r.t to the distance:
   *       Allows simple comparisons starting from the middle of the trajectory.
   * 
   * @param vertices vertex container containing Eigen::Vector2d points (the last and first point are connected)
   * @param[out] distance [optional] the resulting minimum distance
   * @return Index to the closest pose in the pose sequence
   */
  int findClosestTrajectoryPose(const Point2dContainer& vertices, double* distance = NULL) const;

  /**
   * @brief Find the closest point on the trajectory w.r.t to a provided obstacle type
   * 
   * This function can be useful to find the part of a trajectory that is close to an obstacle.
   * The method is calculates appropriate distance metrics for point, line and polygon obstacles.
   * For all unknown obstacles the centroid is used.
   *
   * @param obstacle Subclass of the Obstacle base class
   * @param[out] distance [optional] the resulting minimum distance
   * @return Index to the closest pose in the pose sequence
   */
  int findClosestTrajectoryPose(const Obstacle& obstacle, double* distance = NULL) const;
  
  
  /**
   * @brief Get the length of the internal pose sequence
   */
  int sizePoses() const {return (int)pose_vec_.size();};
  
  /**
   * @brief Get the length of the internal timediff sequence
   */
  int sizeTimeDiffs() const {return (int)timediff_vec_.size();};
  
  /**
   * @brief Check whether the trajectory is initialized (nonzero pose and timediff sequences)
   */
  bool isInit() const {return !timediff_vec_.empty() && !pose_vec_.empty();}

  /**
   * @brief Calculate the total transition time (sum over all time intervals of the timediff sequence)
   */
  double getSumOfAllTimeDiffs() const;
  
  /**
   * @brief Calculate the estimated transition time up to the pose denoted by index
   * @param index Index of the pose up to which the transition times are summed up
   * @return Estimated transition time up to pose index
   */
  double getSumOfTimeDiffsUpToIdx(int index) const;

  /**
   * @brief Calculate the length (accumulated euclidean distance) of the trajectory
   */
  double getAccumulatedDistance() const;
  
  /**
   * @brief Check if all trajectory points are contained in a specific region
   * 
   * The specific region is a circle around the current robot position (Pose(0)) with given radius \c radius.
   * This method investigates a different radius for points behind the robot if \c max_dist_behind_robot >= 0.
   * @param radius radius of the region with the robot position (Pose(0)) as center
   * @param max_dist_behind_robot A separate radius for trajectory points behind the robot, activated if 0 or positive
   * @param skip_poses If >0: the specified number of poses are skipped for the test, e.g. Pose(0), Pose(0+skip_poses+1), Pose(2*skip_poses+2), ... are tested.
   * @return \c true, if all tested trajectory points are inside the specified region, \c false otherwise.
   */
  bool isTrajectoryInsideRegion(double radius, double max_dist_behind_robot=-1, int skip_poses=0);
  
  
  
  //@}
	
protected:
  PoseSequence pose_vec_; //!< Internal container storing the sequence of optimzable pose vertices
  TimeDiffSequence timediff_vec_;  //!< Internal container storing the sequence of optimzable timediff vertices
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace teb_local_planner


// include template implementations / definitions
#include "teb_local_planner/timed_elastic_band.hpp"


#endif /* TIMED_ELASTIC_BAND_H_ */
