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

#ifndef TEB_CONFIG_H_
#define TEB_CONFIG_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <teb_local_planner/TebLocalPlannerReconfigureConfig.h>


// Definitions
#define USE_ANALYTIC_JACOBI // if available for a specific edge, use analytic jacobi


namespace teb_local_planner
{

/**
 * @class TebConfig
 * @brief Config class for the teb_local_planner and its components.
 */
class TebConfig
{
public:

  std::string odom_topic; //!< Topic name of the odometry message, provided by the robot driver or simulator
  std::string map_frame; //!< Global planning frame

  //! Trajectory related parameters
  struct Trajectory
  {
    double teb_autosize; //!< Enable automatic resizing of the trajectory w.r.t to the temporal resolution (recommended)
    double dt_ref; //!< Desired temporal resolution of the trajectory (should be in the magniture of the underlying control rate)
    double dt_hysteresis; //!< Hysteresis for automatic resizing depending on the current temporal resolution (dt): usually 10% of dt_ref
    int min_samples; //!< Minimum number of samples (should be always greater than 2)
    int max_samples; //!< Maximum number of samples; Warning: if too small the discretization/resolution might not be sufficient for the given robot model or obstacle avoidance does not work anymore.
    bool global_plan_overwrite_orientation; //!< Overwrite orientation of local subgoals provided by the global planner
    bool allow_init_with_backwards_motion; //!< If true, the underlying trajectories might be initialized with backwards motions in case the goal is behind the start within the local costmap (this is only recommended if the robot is equipped with rear sensors)
    double global_plan_viapoint_sep; //!< Min. separation between each two consecutive via-points extracted from the global plan (if negative: disabled)
    bool via_points_ordered; //!< If true, the planner adheres to the order of via-points in the storage container
    double max_global_plan_lookahead_dist; //!< Specify maximum length (cumulative Euclidean distances) of the subset of the global plan taken into account for optimization [if <=0: disabled; the length is also bounded by the local costmap size!]
    double global_plan_prune_distance; //!< Distance between robot and via_points of global plan which is used for pruning
    bool exact_arc_length; //!< If true, the planner uses the exact arc length in velocity, acceleration and turning rate computations [-> increased cpu time], otherwise the euclidean approximation is used.
    double force_reinit_new_goal_dist; //!< Reinitialize the trajectory if a previous goal is updated with a seperation of more than the specified value in meters (skip hot-starting)
    double force_reinit_new_goal_angular; //!< Reinitialize the trajectory if a previous goal is updated with an angular difference of more than the specified value in radians (skip hot-starting)
    int feasibility_check_no_poses; //!< Specify up to which pose (under the feasibility_check_lookahead_distance) on the predicted plan the feasibility should be checked each sampling interval; if -1, all poses up to feasibility_check_lookahead_distance are checked.
    double feasibility_check_lookahead_distance; //!< Specify up to which distance (and with an index below feasibility_check_no_poses) from the robot the feasibility should be checked each sampling interval; if -1, all poses up to feasibility_check_no_poses are checked.
    bool publish_feedback; //!< Publish planner feedback containing the full trajectory and a list of active obstacles (should be enabled only for evaluation or debugging purposes)
    double min_resolution_collision_check_angular; //! Min angular resolution used during the costmap collision check. If not respected, intermediate samples are added. [rad]
    int control_look_ahead_poses; //! Index of the pose used to extract the velocity command
    int prevent_look_ahead_poses_near_goal; //! Prevents control_look_ahead_poses to look within this many poses of the goal in order to prevent overshoot & oscillation when xy_goal_tolerance is very small
  } trajectory; //!< Trajectory related parameters

  //! Robot related parameters
  struct Robot
  {
    double max_vel_x; //!< Maximum translational velocity of the robot
    double max_vel_x_backwards; //!< Maximum translational velocity of the robot for driving backwards
    double max_vel_y; //!< Maximum strafing velocity of the robot (should be zero for non-holonomic robots!)
    double max_vel_trans; //!< Maximum translational velocity of the robot for omni robots, which is different from max_vel_x
    double max_vel_theta; //!< Maximum angular velocity of the robot
    double acc_lim_x; //!< Maximum translational acceleration of the robot
    double acc_lim_y; //!< Maximum strafing acceleration of the robot
    double acc_lim_theta; //!< Maximum angular acceleration of the robot
    double min_turning_radius; //!< Minimum turning radius of a carlike robot (diff-drive robot: zero);
    double wheelbase; //!< The distance between the drive shaft and steering axle (only required for a carlike robot with 'cmd_angle_instead_rotvel' enabled); The value might be negative for back-wheeled robots!
    bool cmd_angle_instead_rotvel; //!< Substitute the rotational velocity in the commanded velocity message by the corresponding steering angle (check 'axles_distance')
    bool is_footprint_dynamic; //<! If true, updated the footprint before checking trajectory feasibility
    bool use_proportional_saturation; //<! If true, reduce all twists components (linear x and y, and angular z) proportionally if any exceed its corresponding bounds, instead of saturating each one individually
    double transform_tolerance = 0.5; //<! Tolerance when querying the TF Tree for a transformation (seconds)
  } robot; //!< Robot related parameters

  //! Goal tolerance related parameters
  struct GoalTolerance
  {
    double yaw_goal_tolerance; //!< Allowed final orientation error
    double xy_goal_tolerance; //!< Allowed final euclidean distance to the goal position
    bool free_goal_vel; //!< Allow the robot's velocity to be nonzero (usally max_vel) for planning purposes
    double trans_stopped_vel; //!< Below what maximum velocity we consider the robot to be stopped in translation
    double theta_stopped_vel; //!< Below what maximum rotation velocity we consider the robot to be stopped in rotation
    bool complete_global_plan; // true prevents the robot from ending the path early when it cross the end goal
  } goal_tolerance; //!< Goal tolerance related parameters

  //! Obstacle related parameters
  struct Obstacles
  {
    double min_obstacle_dist; //!< Minimum desired separation from obstacles
    double inflation_dist; //!< buffer zone around obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)
    double dynamic_obstacle_inflation_dist; //!< Buffer zone around predicted locations of dynamic obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)
    bool include_dynamic_obstacles; //!< Specify whether the movement of dynamic obstacles should be predicted by a constant velocity model (this also effects homotopy class planning); If false, all obstacles are considered to be static.
    bool include_costmap_obstacles; //!< Specify whether the obstacles in the costmap should be taken into account directly
    double costmap_obstacles_behind_robot_dist; //!< Limit the occupied local costmap obstacles taken into account for planning behind the robot (specify distance in meters)
    int obstacle_poses_affected; //!< The obstacle position is attached to the closest pose on the trajectory to reduce computational effort, but take a number of neighbors into account as well
    bool legacy_obstacle_association; //!< If true, the old association strategy is used (for each obstacle, find the nearest TEB pose), otherwise the new one (for each teb pose, find only "relevant" obstacles).
    double obstacle_association_force_inclusion_factor; //!< The non-legacy obstacle association technique tries to connect only relevant obstacles with the discretized trajectory during optimization, all obstacles within a specifed distance are forced to be included (as a multiple of min_obstacle_dist), e.g. choose 2.0 in order to consider obstacles within a radius of 2.0*min_obstacle_dist.
    double obstacle_association_cutoff_factor; //!< See obstacle_association_force_inclusion_factor, but beyond a multiple of [value]*min_obstacle_dist all obstacles are ignored during optimization. obstacle_association_force_inclusion_factor is processed first.
    std::string costmap_converter_plugin; //!< Define a plugin name of the costmap_converter package (costmap cells are converted to points/lines/polygons)
    bool costmap_converter_spin_thread; //!< If \c true, the costmap converter invokes its callback queue in a different thread
    int costmap_converter_rate; //!< The rate that defines how often the costmap_converter plugin processes the current costmap (the value should not be much higher than the costmap update rate)
    double obstacle_proximity_ratio_max_vel; //!< Ratio of the maximum velocities used as an upper bound when reducing the speed due to the proximity to a static obstacles
    double obstacle_proximity_lower_bound; //!< Distance to a static obstacle for which the velocity should be lower
    double obstacle_proximity_upper_bound; //!< Distance to a static obstacle for which the velocity should be higher
  } obstacles; //!< Obstacle related parameters


  //! Optimization related parameters
  struct Optimization
  {
    int no_inner_iterations; //!< Number of solver iterations called in each outerloop iteration
    int no_outer_iterations; //!< Each outerloop iteration automatically resizes the trajectory and invokes the internal optimizer with no_inner_iterations

    bool optimization_activate; //!< Activate the optimization
    bool optimization_verbose; //!< Print verbose information

    double penalty_epsilon; //!< Add a small safety margin to penalty functions for hard-constraint approximations

    double weight_max_vel_x; //!< Optimization weight for satisfying the maximum allowed translational velocity
    double weight_max_vel_y; //!< Optimization weight for satisfying the maximum allowed strafing velocity (in use only for holonomic robots)
    double weight_max_vel_theta; //!< Optimization weight for satisfying the maximum allowed angular velocity
    double weight_acc_lim_x; //!< Optimization weight for satisfying the maximum allowed translational acceleration
    double weight_acc_lim_y; //!< Optimization weight for satisfying the maximum allowed strafing acceleration (in use only for holonomic robots)
    double weight_acc_lim_theta; //!< Optimization weight for satisfying the maximum allowed angular acceleration
    double weight_kinematics_nh; //!< Optimization weight for satisfying the non-holonomic kinematics
    double weight_kinematics_forward_drive; //!< Optimization weight for forcing the robot to choose only forward directions (positive transl. velocities, only diffdrive robot)
    double weight_kinematics_turning_radius; //!< Optimization weight for enforcing a minimum turning radius (carlike robots)
    double weight_optimaltime; //!< Optimization weight for contracting the trajectory w.r.t. transition time
    double weight_shortest_path; //!< Optimization weight for contracting the trajectory w.r.t. path length
    double weight_obstacle; //!< Optimization weight for satisfying a minimum separation from obstacles
    double weight_inflation; //!< Optimization weight for the inflation penalty (should be small)
    double weight_dynamic_obstacle; //!< Optimization weight for satisfying a minimum separation from dynamic obstacles
    double weight_dynamic_obstacle_inflation; //!< Optimization weight for the inflation penalty of dynamic obstacles (should be small)
    double weight_velocity_obstacle_ratio; //!< Optimization weight for satisfying a maximum allowed velocity with respect to the distance to a static obstacle
    double weight_viapoint; //!< Optimization weight for minimizing the distance to via-points
    double weight_prefer_rotdir; //!< Optimization weight for preferring a specific turning direction (-> currently only activated if an oscillation is detected, see 'oscillation_recovery'

    double weight_adapt_factor; //!< Some special weights (currently 'weight_obstacle') are repeatedly scaled by this factor in each outer TEB iteration (weight_new = weight_old*factor); Increasing weights iteratively instead of setting a huge value a-priori leads to better numerical conditions of the underlying optimization problem.
    double obstacle_cost_exponent; //!< Exponent for nonlinear obstacle cost (cost = linear_cost * obstacle_cost_exponent). Set to 1 to disable nonlinear cost (default)
  } optim; //!< Optimization related parameters


  struct HomotopyClasses
  {
    bool enable_homotopy_class_planning; //!< Activate homotopy class planning (Requires much more resources that simple planning, since multiple trajectories are optimized at once).
    bool enable_multithreading; //!< Activate multiple threading for planning multiple trajectories in parallel.
    bool simple_exploration; //!< If true, distinctive trajectories are explored using a simple left-right approach (pass each obstacle on the left or right side) for path generation, otherwise sample possible roadmaps randomly in a specified region between start and goal.
    int max_number_classes; //!< Specify the maximum number of allowed alternative homotopy classes (limits computational effort)
    int max_number_plans_in_current_class; //!< Specify the maximum number of trajectories to try that are in the same homotopy class as the current trajectory (helps avoid local minima)
    double selection_cost_hysteresis; //!< Specify how much trajectory cost must a new candidate have w.r.t. a previously selected trajectory in order to be selected (selection if new_cost < old_cost*factor).
    double selection_prefer_initial_plan; //!< Specify a cost reduction in the interval (0,1) for the trajectory in the equivalence class of the initial plan.
    double selection_obst_cost_scale; //!< Extra scaling of obstacle cost terms just for selecting the 'best' candidate.
    double selection_viapoint_cost_scale; //!< Extra scaling of via-point cost terms just for selecting the 'best' candidate.
    bool selection_alternative_time_cost; //!< If true, time cost is replaced by the total transition time.
    double selection_dropping_probability; //!< At each planning cycle, TEBs other than the current 'best' one will be randomly dropped with this probability. Prevents becoming 'fixated' on sub-optimal alternative homotopies.
    double switching_blocking_period; //!< Specify a time duration in seconds that needs to be expired before a switch to new equivalence class is allowed

    int roadmap_graph_no_samples; //! < Specify the number of samples generated for creating the roadmap graph, if simple_exploration is turend off.
    double roadmap_graph_area_width; //!< Random keypoints/waypoints are sampled in a rectangular region between start and goal. Specify the width of that region in meters.
    double roadmap_graph_area_length_scale; //!< The length of the rectangular region is determined by the distance between start and goal. This parameter further scales the distance such that the geometric center remains equal!
    double h_signature_prescaler; //!< Scale number of obstacle value in order to allow huge number of obstacles. Do not choose it extremly low, otherwise obstacles cannot be distinguished from each other (0.2<H<=1).
    double h_signature_threshold; //!< Two h-signatures are assumed to be equal, if both the difference of real parts and complex parts are below the specified threshold.

    double obstacle_keypoint_offset; //!< If simple_exploration is turned on, this parameter determines the distance on the left and right side of the obstacle at which a new keypoint will be cretead (in addition to min_obstacle_dist).
    double obstacle_heading_threshold; //!< Specify the value of the normalized scalar product between obstacle heading and goal heading in order to take them (obstacles) into account for exploration [0,1]

    bool viapoints_all_candidates; //!< If true, all trajectories of different topologies are attached to the current set of via-points, otherwise only the trajectory sharing the same one as the initial/global plan.

    bool visualize_hc_graph; //!< Visualize the graph that is created for exploring new homotopy classes.
    double visualize_with_time_as_z_axis_scale; //!< If this value is bigger than 0, the trajectory and obstacles are visualized in 3d using the time as the z-axis scaled by this value. Most useful for dynamic obstacles.
    bool delete_detours_backwards; //!< If enabled, the planner will discard the plans detouring backwards with respect to the best plan
    double detours_orientation_tolerance; //!< A plan is considered a detour if its start orientation differs more than this from the best plan
    double length_start_orientation_vector; //!< Length of the vector used to compute the start orientation of a plan
    double max_ratio_detours_duration_best_duration; //!< Detours are discarted if their execution time / the execution time of the best teb is > this
  } hcp;

  //! Recovery/backup related parameters
  struct Recovery
  {
    bool shrink_horizon_backup; //!< Allows the planner to shrink the horizon temporary (50%) in case of automatically detected issues.
    double shrink_horizon_min_duration; //!< Specify minimum duration for the reduced horizon in case an infeasible trajectory is detected.
    bool oscillation_recovery; //!< Try to detect and resolve oscillations between multiple solutions in the same equivalence class (robot frequently switches between left/right/forward/backwards)
    double oscillation_v_eps; //!< Threshold for the average normalized linear velocity: if oscillation_v_eps and oscillation_omega_eps are not exceeded both, a possible oscillation is detected
    double oscillation_omega_eps; //!< Threshold for the average normalized angular velocity: if oscillation_v_eps and oscillation_omega_eps are not exceeded both, a possible oscillation is detected
    double oscillation_recovery_min_duration; //!< Minumum duration [sec] for which the recovery mode is activated after an oscillation is detected.
    double oscillation_filter_duration; //!< Filter length/duration [sec] for the detection of oscillations
    bool divergence_detection_enable; //!< True to enable divergence detection.
    int divergence_detection_max_chi_squared; //!< Maximum acceptable Mahalanobis distance above which it is assumed that the optimization diverged.
  } recovery; //!< Parameters related to recovery and backup strategies


  /**
  * @brief Construct the TebConfig using default values.
  * @warning If the \b rosparam server or/and \b dynamic_reconfigure (rqt_reconfigure) node are used,
  *	     the default variables will be overwritten: \n
  *	     E.g. if \e base_local_planner is utilized as plugin for the navigation stack, the initialize() method will register a
  * 	     dynamic_reconfigure server. A subset (not all but most) of the parameters are considered for dynamic modifications.
  * 	     All parameters considered by the dynamic_reconfigure server (and their \b default values) are
  * 	     set in \e PROJECT_SRC/cfg/TebLocalPlannerReconfigure.cfg. \n
  * 	     In addition the rosparam server can be queried to get parameters e.g. defiend in a launch file.
  * 	     The plugin source (or a possible binary source) can call loadRosParamFromNodeHandle() to update the parameters.
  * 	     In \e summary, default parameters are loaded in the following order (the right one overrides the left ones): \n
  * 		<b>TebConfig Constructor defaults << dynamic_reconfigure defaults << rosparam server defaults</b>
  */
  TebConfig()
  {

    odom_topic = "odom";
    map_frame = "odom";

    // Trajectory

    trajectory.teb_autosize = true;
    trajectory.dt_ref = 0.3;
    trajectory.dt_hysteresis = 0.1;
    trajectory.min_samples = 3;
    trajectory.max_samples = 500;
    trajectory.global_plan_overwrite_orientation = true;
    trajectory.allow_init_with_backwards_motion = false;
    trajectory.global_plan_viapoint_sep = -1;
    trajectory.via_points_ordered = false;
    trajectory.max_global_plan_lookahead_dist = 1;
    trajectory.global_plan_prune_distance = 1;
    trajectory.exact_arc_length = false;
    trajectory.force_reinit_new_goal_dist = 1;
    trajectory.force_reinit_new_goal_angular = 0.5 * M_PI;
    trajectory.feasibility_check_no_poses = 5;
    trajectory.feasibility_check_lookahead_distance = -1;
    trajectory.publish_feedback = false;
    trajectory.min_resolution_collision_check_angular = M_PI;
    trajectory.control_look_ahead_poses = 1;
    trajectory.prevent_look_ahead_poses_near_goal = 0;

    // Robot

    robot.max_vel_x = 0.4;
    robot.max_vel_x_backwards = 0.2;
    robot.max_vel_y = 0.0;
    robot.max_vel_trans = 0.0;
    robot.max_vel_theta = 0.3;
    robot.acc_lim_x = 0.5;
    robot.acc_lim_y = 0.5;
    robot.acc_lim_theta = 0.5;
    robot.min_turning_radius = 0;
    robot.wheelbase = 1.0;
    robot.cmd_angle_instead_rotvel = false;
    robot.is_footprint_dynamic = false;
    robot.use_proportional_saturation = false;

    // GoalTolerance

    goal_tolerance.xy_goal_tolerance = 0.2;
    goal_tolerance.yaw_goal_tolerance = 0.2;
    goal_tolerance.free_goal_vel = false;
    goal_tolerance.trans_stopped_vel = 0.1;
    goal_tolerance.theta_stopped_vel = 0.1;
    goal_tolerance.complete_global_plan = true;

    // Obstacles

    obstacles.min_obstacle_dist = 0.5;
    obstacles.inflation_dist = 0.6;
    obstacles.dynamic_obstacle_inflation_dist = 0.6;
    obstacles.include_dynamic_obstacles = true;
    obstacles.include_costmap_obstacles = true;
    obstacles.costmap_obstacles_behind_robot_dist = 1.5;
    obstacles.obstacle_poses_affected = 25;
    obstacles.legacy_obstacle_association = false;
    obstacles.obstacle_association_force_inclusion_factor = 1.5;
    obstacles.obstacle_association_cutoff_factor = 5;
    obstacles.costmap_converter_plugin = "";
    obstacles.costmap_converter_spin_thread = true;
    obstacles.costmap_converter_rate = 5;
    obstacles.obstacle_proximity_ratio_max_vel = 1;
    obstacles.obstacle_proximity_lower_bound = 0;
    obstacles.obstacle_proximity_upper_bound = 0.5;

    // Optimization

    optim.no_inner_iterations = 5;
    optim.no_outer_iterations = 4;
    optim.optimization_activate = true;
    optim.optimization_verbose = false;
    optim.penalty_epsilon = 0.05;
    optim.weight_max_vel_x = 2; //1
    optim.weight_max_vel_y = 2;
    optim.weight_max_vel_theta = 1;
    optim.weight_acc_lim_x = 1;
    optim.weight_acc_lim_y = 1;
    optim.weight_acc_lim_theta = 1;
    optim.weight_kinematics_nh = 1000;
    optim.weight_kinematics_forward_drive = 1;
    optim.weight_kinematics_turning_radius = 1;
    optim.weight_optimaltime = 1;
    optim.weight_shortest_path = 0;
    optim.weight_obstacle = 50;
    optim.weight_inflation = 0.1;
    optim.weight_dynamic_obstacle = 50;
    optim.weight_dynamic_obstacle_inflation = 0.1;
    optim.weight_velocity_obstacle_ratio = 0;
    optim.weight_viapoint = 1;
    optim.weight_prefer_rotdir = 50;

    optim.weight_adapt_factor = 2.0;
    optim.obstacle_cost_exponent = 1.0;

    // Homotopy Class Planner

    hcp.enable_homotopy_class_planning = true;
    hcp.enable_multithreading = true;
    hcp.simple_exploration = false;
    hcp.max_number_classes = 5;
    hcp.selection_cost_hysteresis = 1.0;
    hcp.selection_prefer_initial_plan = 0.95;
    hcp.selection_obst_cost_scale = 100.0;
    hcp.selection_viapoint_cost_scale = 1.0;
    hcp.selection_alternative_time_cost = false;
    hcp.selection_dropping_probability = 0.0;

    hcp.obstacle_keypoint_offset = 0.1;
    hcp.obstacle_heading_threshold = 0.45;
    hcp.roadmap_graph_no_samples = 15;
    hcp.roadmap_graph_area_width = 6; // [m]
    hcp.roadmap_graph_area_length_scale = 1.0;
    hcp.h_signature_prescaler = 1;
    hcp.h_signature_threshold = 0.1;
    hcp.switching_blocking_period = 0.0;

    hcp.viapoints_all_candidates = true;

    hcp.visualize_hc_graph = false;
    hcp.visualize_with_time_as_z_axis_scale = 0.0;
    hcp.delete_detours_backwards = true;
    hcp.detours_orientation_tolerance = M_PI / 2.0;
    hcp.length_start_orientation_vector = 0.4;
    hcp.max_ratio_detours_duration_best_duration = 3.0;

    // Recovery

    recovery.shrink_horizon_backup = true;
    recovery.shrink_horizon_min_duration = 10;
    recovery.oscillation_recovery = true;
    recovery.oscillation_v_eps = 0.1;
    recovery.oscillation_omega_eps = 0.1;
    recovery.oscillation_recovery_min_duration = 10;
    recovery.oscillation_filter_duration = 10;


  }

  /**
   * @brief Load parmeters from the ros param server.
   * @param nh const reference to the local ros::NodeHandle
   */
  void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

  /**
   * @brief Reconfigure parameters from the dynamic_reconfigure config.
   * Change parameters dynamically (e.g. with <c>rosrun rqt_reconfigure rqt_reconfigure</c>).
   * A reconfigure server needs to be instantiated that calls this method in it's callback.
   * In case of the plugin \e teb_local_planner default values are defined
   * in \e PROJECT_SRC/cfg/TebLocalPlannerReconfigure.cfg.
   * @param cfg Config class autogenerated by dynamic_reconfigure according to the cfg-file mentioned above.
   */
  void reconfigure(TebLocalPlannerReconfigureConfig& cfg);

  /**
   * @brief Check parameters and print warnings in case of discrepancies
   *
   * Call this method whenever parameters are changed using public interfaces to inform the user
   * about some improper uses.
   */
  void checkParameters() const;

  /**
   * @brief Check if some deprecated parameters are found and print warnings
   * @param nh const reference to the local ros::NodeHandle
   */
  void checkDeprecated(const ros::NodeHandle& nh) const;

  /**
   * @brief Return the internal config mutex
   */
  boost::mutex& configMutex() {return config_mutex_;}

private:
  boost::mutex config_mutex_; //!< Mutex for config accesses and changes

};


} // namespace teb_local_planner

#endif
