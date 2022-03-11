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

#include <teb_local_planner/teb_config.h>

namespace teb_local_planner
{
    
void TebConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh)
{
    
  nh.param("odom_topic", odom_topic, odom_topic);
  nh.param("map_frame", map_frame, map_frame);
  
  // Trajectory
  nh.param("teb_autosize", trajectory.teb_autosize, trajectory.teb_autosize);
  nh.param("dt_ref", trajectory.dt_ref, trajectory.dt_ref);
  nh.param("dt_hysteresis", trajectory.dt_hysteresis, trajectory.dt_hysteresis);
  nh.param("min_samples", trajectory.min_samples, trajectory.min_samples);
  nh.param("max_samples", trajectory.max_samples, trajectory.max_samples);
  nh.param("global_plan_overwrite_orientation", trajectory.global_plan_overwrite_orientation, trajectory.global_plan_overwrite_orientation);
  nh.param("allow_init_with_backwards_motion", trajectory.allow_init_with_backwards_motion, trajectory.allow_init_with_backwards_motion);
  nh.getParam("global_plan_via_point_sep", trajectory.global_plan_viapoint_sep); // deprecated, see checkDeprecated()
  if (!nh.param("global_plan_viapoint_sep", trajectory.global_plan_viapoint_sep, trajectory.global_plan_viapoint_sep))
    nh.setParam("global_plan_viapoint_sep", trajectory.global_plan_viapoint_sep); // write deprecated value to param server
  nh.param("via_points_ordered", trajectory.via_points_ordered, trajectory.via_points_ordered);
  nh.param("max_global_plan_lookahead_dist", trajectory.max_global_plan_lookahead_dist, trajectory.max_global_plan_lookahead_dist);
  nh.param("global_plan_prune_distance", trajectory.global_plan_prune_distance, trajectory.global_plan_prune_distance);
  nh.param("exact_arc_length", trajectory.exact_arc_length, trajectory.exact_arc_length);
  nh.param("force_reinit_new_goal_dist", trajectory.force_reinit_new_goal_dist, trajectory.force_reinit_new_goal_dist);
  nh.param("force_reinit_new_goal_angular", trajectory.force_reinit_new_goal_angular, trajectory.force_reinit_new_goal_angular);
  nh.param("feasibility_check_no_poses", trajectory.feasibility_check_no_poses, trajectory.feasibility_check_no_poses);
  nh.param("feasibility_check_lookahead_distance", trajectory.feasibility_check_lookahead_distance, trajectory.feasibility_check_lookahead_distance);
  nh.param("publish_feedback", trajectory.publish_feedback, trajectory.publish_feedback);
  nh.param("min_resolution_collision_check_angular", trajectory.min_resolution_collision_check_angular, trajectory.min_resolution_collision_check_angular);
  nh.param("control_look_ahead_poses", trajectory.control_look_ahead_poses, trajectory.control_look_ahead_poses);
  nh.param("prevent_look_ahead_poses_near_goal", trajectory.prevent_look_ahead_poses_near_goal, trajectory.prevent_look_ahead_poses_near_goal);
  
  // Robot
  nh.param("max_vel_x", robot.max_vel_x, robot.max_vel_x);
  nh.param("max_vel_x_backwards", robot.max_vel_x_backwards, robot.max_vel_x_backwards);
  nh.param("max_vel_y", robot.max_vel_y, robot.max_vel_y);
  nh.param("max_vel_trans", robot.max_vel_trans, robot.max_vel_trans);
  nh.param("max_vel_theta", robot.max_vel_theta, robot.max_vel_theta);
  nh.param("acc_lim_x", robot.acc_lim_x, robot.acc_lim_x);
  nh.param("acc_lim_y", robot.acc_lim_y, robot.acc_lim_y);
  nh.param("acc_lim_theta", robot.acc_lim_theta, robot.acc_lim_theta);
  nh.param("min_turning_radius", robot.min_turning_radius, robot.min_turning_radius);
  nh.param("wheelbase", robot.wheelbase, robot.wheelbase);
  nh.param("cmd_angle_instead_rotvel", robot.cmd_angle_instead_rotvel, robot.cmd_angle_instead_rotvel);
  nh.param("is_footprint_dynamic", robot.is_footprint_dynamic, robot.is_footprint_dynamic);
  nh.param("use_proportional_saturation", robot.use_proportional_saturation, robot.use_proportional_saturation);
  nh.param("transform_tolerance", robot.transform_tolerance, robot.transform_tolerance);

  // GoalTolerance
  nh.param("xy_goal_tolerance", goal_tolerance.xy_goal_tolerance, goal_tolerance.xy_goal_tolerance);
  nh.param("yaw_goal_tolerance", goal_tolerance.yaw_goal_tolerance, goal_tolerance.yaw_goal_tolerance);
  nh.param("free_goal_vel", goal_tolerance.free_goal_vel, goal_tolerance.free_goal_vel);
  nh.param("trans_stopped_vel", goal_tolerance.trans_stopped_vel, goal_tolerance.trans_stopped_vel);
  nh.param("theta_stopped_vel", goal_tolerance.theta_stopped_vel, goal_tolerance.theta_stopped_vel);
  nh.param("complete_global_plan", goal_tolerance.complete_global_plan, goal_tolerance.complete_global_plan);

  // Obstacles
  nh.param("min_obstacle_dist", obstacles.min_obstacle_dist, obstacles.min_obstacle_dist);
  nh.param("inflation_dist", obstacles.inflation_dist, obstacles.inflation_dist);
  nh.param("dynamic_obstacle_inflation_dist", obstacles.dynamic_obstacle_inflation_dist, obstacles.dynamic_obstacle_inflation_dist);
  nh.param("include_dynamic_obstacles", obstacles.include_dynamic_obstacles, obstacles.include_dynamic_obstacles);
  nh.param("include_costmap_obstacles", obstacles.include_costmap_obstacles, obstacles.include_costmap_obstacles);
  nh.param("costmap_obstacles_behind_robot_dist", obstacles.costmap_obstacles_behind_robot_dist, obstacles.costmap_obstacles_behind_robot_dist);
  nh.param("obstacle_poses_affected", obstacles.obstacle_poses_affected, obstacles.obstacle_poses_affected);
  nh.param("legacy_obstacle_association", obstacles.legacy_obstacle_association, obstacles.legacy_obstacle_association);
  nh.param("obstacle_association_force_inclusion_factor", obstacles.obstacle_association_force_inclusion_factor, obstacles.obstacle_association_force_inclusion_factor);
  nh.param("obstacle_association_cutoff_factor", obstacles.obstacle_association_cutoff_factor, obstacles.obstacle_association_cutoff_factor);
  nh.param("costmap_converter_plugin", obstacles.costmap_converter_plugin, obstacles.costmap_converter_plugin);
  nh.param("costmap_converter_spin_thread", obstacles.costmap_converter_spin_thread, obstacles.costmap_converter_spin_thread);
  nh.param("obstacle_proximity_ratio_max_vel",  obstacles.obstacle_proximity_ratio_max_vel, obstacles.obstacle_proximity_ratio_max_vel);
  nh.param("obstacle_proximity_lower_bound", obstacles.obstacle_proximity_lower_bound, obstacles.obstacle_proximity_lower_bound);
  nh.param("obstacle_proximity_upper_bound", obstacles.obstacle_proximity_upper_bound, obstacles.obstacle_proximity_upper_bound);
  
  // Optimization
  nh.param("no_inner_iterations", optim.no_inner_iterations, optim.no_inner_iterations);
  nh.param("no_outer_iterations", optim.no_outer_iterations, optim.no_outer_iterations);
  nh.param("optimization_activate", optim.optimization_activate, optim.optimization_activate);
  nh.param("optimization_verbose", optim.optimization_verbose, optim.optimization_verbose);
  nh.param("penalty_epsilon", optim.penalty_epsilon, optim.penalty_epsilon);
  nh.param("weight_max_vel_x", optim.weight_max_vel_x, optim.weight_max_vel_x);
  nh.param("weight_max_vel_y", optim.weight_max_vel_y, optim.weight_max_vel_y);
  nh.param("weight_max_vel_theta", optim.weight_max_vel_theta, optim.weight_max_vel_theta);
  nh.param("weight_acc_lim_x", optim.weight_acc_lim_x, optim.weight_acc_lim_x);
  nh.param("weight_acc_lim_y", optim.weight_acc_lim_y, optim.weight_acc_lim_y);
  nh.param("weight_acc_lim_theta", optim.weight_acc_lim_theta, optim.weight_acc_lim_theta);
  nh.param("weight_kinematics_nh", optim.weight_kinematics_nh, optim.weight_kinematics_nh);
  nh.param("weight_kinematics_forward_drive", optim.weight_kinematics_forward_drive, optim.weight_kinematics_forward_drive);
  nh.param("weight_kinematics_turning_radius", optim.weight_kinematics_turning_radius, optim.weight_kinematics_turning_radius);
  nh.param("weight_optimaltime", optim.weight_optimaltime, optim.weight_optimaltime);
  nh.param("weight_shortest_path", optim.weight_shortest_path, optim.weight_shortest_path);
  nh.param("weight_obstacle", optim.weight_obstacle, optim.weight_obstacle);
  nh.param("weight_inflation", optim.weight_inflation, optim.weight_inflation);
  nh.param("weight_dynamic_obstacle", optim.weight_dynamic_obstacle, optim.weight_dynamic_obstacle);    
  nh.param("weight_dynamic_obstacle_inflation", optim.weight_dynamic_obstacle_inflation, optim.weight_dynamic_obstacle_inflation);
  nh.param("weight_velocity_obstacle_ratio", optim.weight_velocity_obstacle_ratio, optim.weight_velocity_obstacle_ratio);
  nh.param("weight_viapoint", optim.weight_viapoint, optim.weight_viapoint);
  nh.param("weight_prefer_rotdir", optim.weight_prefer_rotdir, optim.weight_prefer_rotdir);
  nh.param("weight_adapt_factor", optim.weight_adapt_factor, optim.weight_adapt_factor);
  nh.param("obstacle_cost_exponent", optim.obstacle_cost_exponent, optim.obstacle_cost_exponent);
  
  // Homotopy Class Planner
  nh.param("enable_homotopy_class_planning", hcp.enable_homotopy_class_planning, hcp.enable_homotopy_class_planning); 
  nh.param("enable_multithreading", hcp.enable_multithreading, hcp.enable_multithreading); 
  nh.param("simple_exploration", hcp.simple_exploration, hcp.simple_exploration); 
  nh.param("max_number_classes", hcp.max_number_classes, hcp.max_number_classes);
  nh.param("max_number_plans_in_current_class", hcp.max_number_plans_in_current_class, hcp.max_number_plans_in_current_class);
  nh.param("selection_obst_cost_scale", hcp.selection_obst_cost_scale, hcp.selection_obst_cost_scale);
  nh.param("selection_prefer_initial_plan", hcp.selection_prefer_initial_plan, hcp.selection_prefer_initial_plan);
  nh.param("selection_viapoint_cost_scale", hcp.selection_viapoint_cost_scale, hcp.selection_viapoint_cost_scale);
  nh.param("selection_cost_hysteresis", hcp.selection_cost_hysteresis, hcp.selection_cost_hysteresis); 
  nh.param("selection_alternative_time_cost", hcp.selection_alternative_time_cost, hcp.selection_alternative_time_cost); 
  nh.param("selection_dropping_probability", hcp.selection_dropping_probability, hcp.selection_dropping_probability); 
  nh.param("switching_blocking_period", hcp.switching_blocking_period, hcp.switching_blocking_period);
  nh.param("roadmap_graph_samples", hcp.roadmap_graph_no_samples, hcp.roadmap_graph_no_samples); 
  nh.param("roadmap_graph_area_width", hcp.roadmap_graph_area_width, hcp.roadmap_graph_area_width); 
  nh.param("roadmap_graph_area_length_scale", hcp.roadmap_graph_area_length_scale, hcp.roadmap_graph_area_length_scale);
  nh.param("h_signature_prescaler", hcp.h_signature_prescaler, hcp.h_signature_prescaler); 
  nh.param("h_signature_threshold", hcp.h_signature_threshold, hcp.h_signature_threshold); 
  nh.param("obstacle_keypoint_offset", hcp.obstacle_keypoint_offset, hcp.obstacle_keypoint_offset); 
  nh.param("obstacle_heading_threshold", hcp.obstacle_heading_threshold, hcp.obstacle_heading_threshold); 
  nh.param("viapoints_all_candidates", hcp.viapoints_all_candidates, hcp.viapoints_all_candidates);
  nh.param("visualize_hc_graph", hcp.visualize_hc_graph, hcp.visualize_hc_graph); 
  nh.param("visualize_with_time_as_z_axis_scale", hcp.visualize_with_time_as_z_axis_scale, hcp.visualize_with_time_as_z_axis_scale);
  nh.param("delete_detours_backwards", hcp.delete_detours_backwards, hcp.delete_detours_backwards);
  nh.param("detours_orientation_tolerance", hcp.detours_orientation_tolerance, hcp.detours_orientation_tolerance);
  nh.param("length_start_orientation_vector", hcp.length_start_orientation_vector, hcp.length_start_orientation_vector);
  nh.param("max_ratio_detours_duration_best_duration", hcp.max_ratio_detours_duration_best_duration, hcp.max_ratio_detours_duration_best_duration);
  
  // Recovery
  nh.param("shrink_horizon_backup", recovery.shrink_horizon_backup, recovery.shrink_horizon_backup);
  nh.param("shrink_horizon_min_duration", recovery.shrink_horizon_min_duration, recovery.shrink_horizon_min_duration);
  nh.param("oscillation_recovery", recovery.oscillation_recovery, recovery.oscillation_recovery);
  nh.param("oscillation_v_eps", recovery.oscillation_v_eps, recovery.oscillation_v_eps);
  nh.param("oscillation_omega_eps", recovery.oscillation_omega_eps, recovery.oscillation_omega_eps);
  nh.param("oscillation_recovery_min_duration", recovery.oscillation_recovery_min_duration, recovery.oscillation_recovery_min_duration);
  nh.param("oscillation_filter_duration", recovery.oscillation_filter_duration, recovery.oscillation_filter_duration);
  nh.param("divergence_detection", recovery.divergence_detection_enable, recovery.divergence_detection_enable);
  nh.param("divergence_detection_max_chi_squared", recovery.divergence_detection_max_chi_squared, recovery.divergence_detection_max_chi_squared);

  checkParameters();
  checkDeprecated(nh);
}

void TebConfig::reconfigure(TebLocalPlannerReconfigureConfig& cfg)
{ 
  boost::mutex::scoped_lock l(config_mutex_);
  
  // Trajectory
  trajectory.teb_autosize = cfg.teb_autosize;
  trajectory.dt_ref = cfg.dt_ref;
  trajectory.dt_hysteresis = cfg.dt_hysteresis;
  trajectory.global_plan_overwrite_orientation = cfg.global_plan_overwrite_orientation;
  trajectory.allow_init_with_backwards_motion = cfg.allow_init_with_backwards_motion;
  trajectory.global_plan_viapoint_sep = cfg.global_plan_viapoint_sep;
  trajectory.via_points_ordered = cfg.via_points_ordered;
  trajectory.max_global_plan_lookahead_dist = cfg.max_global_plan_lookahead_dist;
  trajectory.exact_arc_length = cfg.exact_arc_length;
  trajectory.force_reinit_new_goal_dist = cfg.force_reinit_new_goal_dist;
  trajectory.force_reinit_new_goal_angular = cfg.force_reinit_new_goal_angular;
  trajectory.feasibility_check_no_poses = cfg.feasibility_check_no_poses;
  trajectory.feasibility_check_lookahead_distance = cfg.feasibility_check_lookahead_distance;
  trajectory.publish_feedback = cfg.publish_feedback;
  trajectory.control_look_ahead_poses = cfg.control_look_ahead_poses;
  trajectory.prevent_look_ahead_poses_near_goal = cfg.prevent_look_ahead_poses_near_goal;
  
  // Robot     
  robot.max_vel_x = cfg.max_vel_x;
  robot.max_vel_x_backwards = cfg.max_vel_x_backwards;
  robot.max_vel_y = cfg.max_vel_y;
  robot.max_vel_theta = cfg.max_vel_theta;
  robot.acc_lim_x = cfg.acc_lim_x;
  robot.acc_lim_y = cfg.acc_lim_y;
  robot.acc_lim_theta = cfg.acc_lim_theta;
  robot.min_turning_radius = cfg.min_turning_radius;
  robot.wheelbase = cfg.wheelbase;
  robot.cmd_angle_instead_rotvel = cfg.cmd_angle_instead_rotvel;
  robot.use_proportional_saturation = cfg.use_proportional_saturation;
  if (cfg.max_vel_trans == 0.0)
  {
    ROS_INFO_STREAM("max_vel_trans is not set, setting it equal to max_vel_x: " << robot.max_vel_x);
    cfg.max_vel_trans = robot.max_vel_x;
  }
  robot.max_vel_trans = cfg.max_vel_trans;
  
  // GoalTolerance
  goal_tolerance.xy_goal_tolerance = cfg.xy_goal_tolerance;
  goal_tolerance.yaw_goal_tolerance = cfg.yaw_goal_tolerance;
  goal_tolerance.free_goal_vel = cfg.free_goal_vel;
  goal_tolerance.trans_stopped_vel = cfg.trans_stopped_vel;
  goal_tolerance.theta_stopped_vel = cfg.theta_stopped_vel;

  // Obstacles
  obstacles.min_obstacle_dist = cfg.min_obstacle_dist;
  obstacles.inflation_dist = cfg.inflation_dist;
  obstacles.dynamic_obstacle_inflation_dist = cfg.dynamic_obstacle_inflation_dist;
  obstacles.include_dynamic_obstacles = cfg.include_dynamic_obstacles;
  obstacles.include_costmap_obstacles = cfg.include_costmap_obstacles;
  obstacles.legacy_obstacle_association = cfg.legacy_obstacle_association;
  obstacles.obstacle_association_force_inclusion_factor = cfg.obstacle_association_force_inclusion_factor;
  obstacles.obstacle_association_cutoff_factor = cfg.obstacle_association_cutoff_factor;
  obstacles.costmap_obstacles_behind_robot_dist = cfg.costmap_obstacles_behind_robot_dist;
  obstacles.obstacle_poses_affected = cfg.obstacle_poses_affected;
  obstacles.obstacle_proximity_ratio_max_vel = cfg.obstacle_proximity_ratio_max_vel;
  obstacles.obstacle_proximity_lower_bound = cfg.obstacle_proximity_lower_bound;
  obstacles.obstacle_proximity_upper_bound = cfg.obstacle_proximity_upper_bound;
  
  // Optimization
  optim.no_inner_iterations = cfg.no_inner_iterations;
  optim.no_outer_iterations = cfg.no_outer_iterations;
  optim.optimization_activate = cfg.optimization_activate;
  optim.optimization_verbose = cfg.optimization_verbose;
  optim.penalty_epsilon = cfg.penalty_epsilon;
  optim.weight_max_vel_x = cfg.weight_max_vel_x;
  optim.weight_max_vel_y = cfg.weight_max_vel_y;
  optim.weight_max_vel_theta = cfg.weight_max_vel_theta;
  optim.weight_acc_lim_x = cfg.weight_acc_lim_x;
  optim.weight_acc_lim_y = cfg.weight_acc_lim_y;
  optim.weight_acc_lim_theta = cfg.weight_acc_lim_theta;
  optim.weight_kinematics_nh = cfg.weight_kinematics_nh;
  optim.weight_kinematics_forward_drive = cfg.weight_kinematics_forward_drive;
  optim.weight_kinematics_turning_radius = cfg.weight_kinematics_turning_radius;
  optim.weight_optimaltime = cfg.weight_optimaltime;
  optim.weight_shortest_path = cfg.weight_shortest_path;
  optim.weight_obstacle = cfg.weight_obstacle;
  optim.weight_inflation = cfg.weight_inflation;
  optim.weight_dynamic_obstacle = cfg.weight_dynamic_obstacle;
  optim.weight_dynamic_obstacle_inflation = cfg.weight_dynamic_obstacle_inflation;
  optim.weight_velocity_obstacle_ratio = cfg.weight_velocity_obstacle_ratio;
  optim.weight_viapoint = cfg.weight_viapoint;
  optim.weight_adapt_factor = cfg.weight_adapt_factor;
  optim.obstacle_cost_exponent = cfg.obstacle_cost_exponent;
  
  // Homotopy Class Planner
  hcp.enable_multithreading = cfg.enable_multithreading;
  hcp.max_number_classes = cfg.max_number_classes; 
  hcp.max_number_plans_in_current_class = cfg.max_number_plans_in_current_class;
  hcp.selection_cost_hysteresis = cfg.selection_cost_hysteresis;
  hcp.selection_prefer_initial_plan = cfg.selection_prefer_initial_plan;
  hcp.selection_obst_cost_scale = cfg.selection_obst_cost_scale;
  hcp.selection_viapoint_cost_scale = cfg.selection_viapoint_cost_scale;
  hcp.selection_alternative_time_cost = cfg.selection_alternative_time_cost;
  hcp.selection_dropping_probability = cfg.selection_dropping_probability;
  hcp.switching_blocking_period = cfg.switching_blocking_period;
  
  hcp.obstacle_heading_threshold = cfg.obstacle_heading_threshold;
  hcp.roadmap_graph_no_samples = cfg.roadmap_graph_no_samples;
  hcp.roadmap_graph_area_width = cfg.roadmap_graph_area_width;
  hcp.roadmap_graph_area_length_scale = cfg.roadmap_graph_area_length_scale;
  hcp.h_signature_prescaler = cfg.h_signature_prescaler;
  hcp.h_signature_threshold = cfg.h_signature_threshold;
  hcp.viapoints_all_candidates = cfg.viapoints_all_candidates;
  hcp.visualize_hc_graph = cfg.visualize_hc_graph;
  hcp.visualize_with_time_as_z_axis_scale = cfg.visualize_with_time_as_z_axis_scale;
  
  // Recovery
  recovery.shrink_horizon_backup = cfg.shrink_horizon_backup;
  recovery.oscillation_recovery = cfg.oscillation_recovery;
  recovery.divergence_detection_enable = cfg.divergence_detection_enable;
  recovery.divergence_detection_max_chi_squared = cfg.divergence_detection_max_chi_squared;

  
  checkParameters();
}
    
    
void TebConfig::checkParameters() const
{
  // positive backward velocity?
  if (robot.max_vel_x_backwards <= 0)
    ROS_WARN("TebLocalPlannerROS() Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  
  // bounds smaller than penalty epsilon
  if (robot.max_vel_x <= optim.penalty_epsilon)
    ROS_WARN("TebLocalPlannerROS() Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.max_vel_x_backwards <= optim.penalty_epsilon)
    ROS_WARN("TebLocalPlannerROS() Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.max_vel_theta <= optim.penalty_epsilon)
    ROS_WARN("TebLocalPlannerROS() Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.acc_lim_x <= optim.penalty_epsilon)
    ROS_WARN("TebLocalPlannerROS() Param Warning: acc_lim_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.acc_lim_theta <= optim.penalty_epsilon)
    ROS_WARN("TebLocalPlannerROS() Param Warning: acc_lim_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
      
  // dt_ref and dt_hyst
  if (trajectory.dt_ref <= trajectory.dt_hysteresis)
    ROS_WARN("TebLocalPlannerROS() Param Warning: dt_ref <= dt_hysteresis. The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at least one of them!");
    
  // min number of samples
  if (trajectory.min_samples <3)
    ROS_WARN("TebLocalPlannerROS() Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't enough degrees of freedom to plan a trajectory for you. Please increase ...");
  
  // costmap obstacle behind robot
  if (obstacles.costmap_obstacles_behind_robot_dist < 0)
    ROS_WARN("TebLocalPlannerROS() Param Warning: parameter 'costmap_obstacles_behind_robot_dist' should be positive or zero.");
    
  // hcp: obstacle heading threshold
  if (hcp.obstacle_keypoint_offset>=1 || hcp.obstacle_keypoint_offset<=0)
    ROS_WARN("TebLocalPlannerROS() Param Warning: parameter obstacle_heading_threshold must be in the interval ]0,1[. 0=0deg opening angle, 1=90deg opening angle.");
  
  // carlike
  if (robot.cmd_angle_instead_rotvel && robot.wheelbase==0)
    ROS_WARN("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but wheelbase is set to zero: undesired behavior.");
  
  if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius==0)
    ROS_WARN("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a diffdrive robot");
  
  // positive weight_adapt_factor
  if (optim.weight_adapt_factor < 1.0)
      ROS_WARN("TebLocalPlannerROS() Param Warning: parameter weight_adapt_factor shoud be >= 1.0");
  
  if (recovery.oscillation_filter_duration < 0)
      ROS_WARN("TebLocalPlannerROS() Param Warning: parameter oscillation_filter_duration must be >= 0");

  // weights
  if (optim.weight_optimaltime <= 0)
      ROS_WARN("TebLocalPlannerROS() Param Warning: parameter weight_optimaltime shoud be > 0 (even if weight_shortest_path is in use)");

  // holonomic check
  if (robot.max_vel_y > 0) {
    if (robot.max_vel_trans < std::min(robot.max_vel_x, robot.max_vel_trans)) {
      ROS_WARN("TebLocalPlannerROS() Param Warning: max_vel_trans < min(max_vel_x, max_vel_y). Note that vel_trans = sqrt(Vx^2 + Vy^2), thus max_vel_trans will limit Vx and Vy in the optimization step.");
    }
    
    if (robot.max_vel_trans > std::max(robot.max_vel_x, robot.max_vel_y)) {
      ROS_WARN("TebLocalPlannerROS() Param Warning: max_vel_trans > max(max_vel_x, max_vel_y). Robot will rotate and move diagonally to achieve max resultant vel (possibly max vel on both axis), limited by the max_vel_trans.");
    }
  }
  
}    

void TebConfig::checkDeprecated(const ros::NodeHandle& nh) const
{
  if (nh.hasParam("line_obstacle_poses_affected") || nh.hasParam("polygon_obstacle_poses_affected"))
    ROS_WARN("TebLocalPlannerROS() Param Warning: 'line_obstacle_poses_affected' and 'polygon_obstacle_poses_affected' are deprecated. They share now the common parameter 'obstacle_poses_affected'.");
  
  if (nh.hasParam("weight_point_obstacle") || nh.hasParam("weight_line_obstacle") || nh.hasParam("weight_poly_obstacle"))
    ROS_WARN("TebLocalPlannerROS() Param Warning: 'weight_point_obstacle', 'weight_line_obstacle' and 'weight_poly_obstacle' are deprecated. They are replaced by the single param 'weight_obstacle'.");
  
  if (nh.hasParam("costmap_obstacles_front_only"))
    ROS_WARN("TebLocalPlannerROS() Param Warning: 'costmap_obstacles_front_only' is deprecated. It is replaced by 'costmap_obstacles_behind_robot_dist' to define the actual area taken into account.");
  
  if (nh.hasParam("costmap_emergency_stop_dist"))
    ROS_WARN("TebLocalPlannerROS() Param Warning: 'costmap_emergency_stop_dist' is deprecated. You can safely remove it from your parameter config.");
  
  if (nh.hasParam("alternative_time_cost"))
    ROS_WARN("TebLocalPlannerROS() Param Warning: 'alternative_time_cost' is deprecated. It has been replaced by 'selection_alternative_time_cost'.");

  if (nh.hasParam("global_plan_via_point_sep"))
    ROS_WARN("TebLocalPlannerROS() Param Warning: 'global_plan_via_point_sep' is deprecated. It has been replaced by 'global_plan_viapoint_sep' due to consistency reasons.");
}
    
} // namespace teb_local_planner
