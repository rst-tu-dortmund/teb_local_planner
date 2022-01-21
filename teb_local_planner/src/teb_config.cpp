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

#include "teb_local_planner/teb_config.h"

using nav2_util::declare_parameter_if_not_declared;

namespace teb_local_planner
{

void TebConfig::declareParameters(const nav2_util::LifecycleNode::SharedPtr nh, const std::string name) {
  node_name = name;

  declare_parameter_if_not_declared(nh, name + "." + "odom_topic", rclcpp::ParameterValue(odom_topic));

  // Trajectory
  declare_parameter_if_not_declared(nh, name + "." + "teb_autosize", rclcpp::ParameterValue(trajectory.teb_autosize));
  declare_parameter_if_not_declared(nh, name + "." + "dt_ref", rclcpp::ParameterValue(trajectory.dt_ref));
  declare_parameter_if_not_declared(nh, name + "." + "dt_hysteresis", rclcpp::ParameterValue(trajectory.dt_hysteresis));
  declare_parameter_if_not_declared(nh, name + "." + "min_samples", rclcpp::ParameterValue(trajectory.min_samples));
  declare_parameter_if_not_declared(nh, name + "." + "max_samples", rclcpp::ParameterValue(trajectory.max_samples));
  declare_parameter_if_not_declared(nh, name + "." + "global_plan_overwrite_orientation", rclcpp::ParameterValue(trajectory.global_plan_overwrite_orientation));
  declare_parameter_if_not_declared(nh, name + "." + "allow_init_with_backwards_motion", rclcpp::ParameterValue(trajectory.allow_init_with_backwards_motion));
  declare_parameter_if_not_declared(nh, name + "." + "global_plan_viapoint_sep", rclcpp::ParameterValue(trajectory.global_plan_viapoint_sep));
  declare_parameter_if_not_declared(nh, name + "." + "via_points_ordered", rclcpp::ParameterValue(trajectory.via_points_ordered));
  declare_parameter_if_not_declared(nh, name + "." + "max_global_plan_lookahead_dist", rclcpp::ParameterValue(trajectory.max_global_plan_lookahead_dist));
  declare_parameter_if_not_declared(nh, name + "." + "global_plan_prune_distance", rclcpp::ParameterValue(trajectory.global_plan_prune_distance));
  declare_parameter_if_not_declared(nh, name + "." + "exact_arc_length", rclcpp::ParameterValue(trajectory.exact_arc_length));
  declare_parameter_if_not_declared(nh, name + "." + "force_reinit_new_goal_dist", rclcpp::ParameterValue(trajectory.force_reinit_new_goal_dist));
  declare_parameter_if_not_declared(nh, name + "." + "force_reinit_new_goal_angular", rclcpp::ParameterValue(trajectory.force_reinit_new_goal_angular));
  declare_parameter_if_not_declared(nh, name + "." + "feasibility_check_no_poses", rclcpp::ParameterValue(trajectory.feasibility_check_no_poses));
  declare_parameter_if_not_declared(nh, name + "." + "publish_feedback", rclcpp::ParameterValue(trajectory.publish_feedback));
  declare_parameter_if_not_declared(nh, name + "." + "min_resolution_collision_check_angular", rclcpp::ParameterValue(trajectory.min_resolution_collision_check_angular));
  declare_parameter_if_not_declared(nh, name + "." + "control_look_ahead_poses", rclcpp::ParameterValue(trajectory.control_look_ahead_poses));
  declare_parameter_if_not_declared(nh, name + "." + "feasibility_check_lookahead_distance", rclcpp::ParameterValue(trajectory.feasibility_check_lookahead_distance));

  // Robot
  declare_parameter_if_not_declared(nh, name + "." + "max_vel_x", rclcpp::ParameterValue(robot.max_vel_x));
  declare_parameter_if_not_declared(nh, name + "." + "max_vel_x_backwards", rclcpp::ParameterValue(robot.max_vel_x_backwards));
  declare_parameter_if_not_declared(nh, name + "." + "max_vel_y", rclcpp::ParameterValue(robot.max_vel_y));
  declare_parameter_if_not_declared(nh, name + "." + "max_vel_theta", rclcpp::ParameterValue(robot.max_vel_theta));
  declare_parameter_if_not_declared(nh, name + "." + "acc_lim_x", rclcpp::ParameterValue(robot.acc_lim_x));
  declare_parameter_if_not_declared(nh, name + "." + "acc_lim_y", rclcpp::ParameterValue(robot.acc_lim_y));
  declare_parameter_if_not_declared(nh, name + "." + "acc_lim_theta", rclcpp::ParameterValue(robot.acc_lim_theta));
  declare_parameter_if_not_declared(nh, name + "." + "min_turning_radius", rclcpp::ParameterValue(robot.min_turning_radius));
  declare_parameter_if_not_declared(nh, name + "." + "wheelbase", rclcpp::ParameterValue(robot.wheelbase));
  declare_parameter_if_not_declared(nh, name + "." + "cmd_angle_instead_rotvel", rclcpp::ParameterValue(robot.cmd_angle_instead_rotvel));
  declare_parameter_if_not_declared(nh, name + "." + "is_footprint_dynamic", rclcpp::ParameterValue(robot.is_footprint_dynamic));

  // GoalTolerance
  declare_parameter_if_not_declared(nh, name + "." + "free_goal_vel", rclcpp::ParameterValue(goal_tolerance.free_goal_vel));

  // Obstacles
  declare_parameter_if_not_declared(nh, name + "." + "min_obstacle_dist", rclcpp::ParameterValue(obstacles.min_obstacle_dist));
  declare_parameter_if_not_declared(nh, name + "." + "inflation_dist", rclcpp::ParameterValue(obstacles.inflation_dist));
  declare_parameter_if_not_declared(nh, name + "." + "dynamic_obstacle_inflation_dist", rclcpp::ParameterValue(obstacles.dynamic_obstacle_inflation_dist));
  declare_parameter_if_not_declared(nh, name + "." + "include_dynamic_obstacles", rclcpp::ParameterValue(obstacles.include_dynamic_obstacles));
  declare_parameter_if_not_declared(nh, name + "." + "include_costmap_obstacles", rclcpp::ParameterValue(obstacles.include_costmap_obstacles));
  declare_parameter_if_not_declared(nh, name + "." + "costmap_obstacles_behind_robot_dist", rclcpp::ParameterValue(obstacles.costmap_obstacles_behind_robot_dist));
  declare_parameter_if_not_declared(nh, name + "." + "obstacle_poses_affected", rclcpp::ParameterValue(obstacles.obstacle_poses_affected));
  declare_parameter_if_not_declared(nh, name + "." + "legacy_obstacle_association", rclcpp::ParameterValue(obstacles.legacy_obstacle_association));
  declare_parameter_if_not_declared(nh, name + "." + "obstacle_association_force_inclusion_factor", rclcpp::ParameterValue(obstacles.obstacle_association_force_inclusion_factor));
  declare_parameter_if_not_declared(nh, name + "." + "obstacle_association_cutoff_factor", rclcpp::ParameterValue(obstacles.obstacle_association_cutoff_factor));
  declare_parameter_if_not_declared(nh, name + "." + "costmap_converter_plugin", rclcpp::ParameterValue(obstacles.costmap_converter_plugin));
  declare_parameter_if_not_declared(nh, name + "." + "costmap_converter_spin_thread", rclcpp::ParameterValue(obstacles.costmap_converter_spin_thread));
  declare_parameter_if_not_declared(nh, name + "." + "obstacle_proximity_ratio_max_vel",  rclcpp::ParameterValue(obstacles.obstacle_proximity_ratio_max_vel));
  declare_parameter_if_not_declared(nh, name + "." + "obstacle_proximity_lower_bound", rclcpp::ParameterValue(obstacles.obstacle_proximity_lower_bound));
  declare_parameter_if_not_declared(nh, name + "." + "obstacle_proximity_upper_bound", rclcpp::ParameterValue(obstacles.obstacle_proximity_upper_bound));

  // Optimization
  declare_parameter_if_not_declared(nh, name + "." + "no_inner_iterations", rclcpp::ParameterValue(optim.no_inner_iterations));
  declare_parameter_if_not_declared(nh, name + "." + "no_outer_iterations", rclcpp::ParameterValue(optim.no_outer_iterations));
  declare_parameter_if_not_declared(nh, name + "." + "optimization_activate", rclcpp::ParameterValue(optim.optimization_activate));
  declare_parameter_if_not_declared(nh, name + "." + "optimization_verbose", rclcpp::ParameterValue(optim.optimization_verbose));
  declare_parameter_if_not_declared(nh, name + "." + "penalty_epsilon", rclcpp::ParameterValue(optim.penalty_epsilon));
  declare_parameter_if_not_declared(nh, name + "." + "weight_max_vel_x", rclcpp::ParameterValue(optim.weight_max_vel_x));
  declare_parameter_if_not_declared(nh, name + "." + "weight_max_vel_y", rclcpp::ParameterValue(optim.weight_max_vel_y));
  declare_parameter_if_not_declared(nh, name + "." + "weight_max_vel_theta", rclcpp::ParameterValue(optim.weight_max_vel_theta));
  declare_parameter_if_not_declared(nh, name + "." + "weight_acc_lim_x", rclcpp::ParameterValue(optim.weight_acc_lim_x));
  declare_parameter_if_not_declared(nh, name + "." + "weight_acc_lim_y", rclcpp::ParameterValue(optim.weight_acc_lim_y));
  declare_parameter_if_not_declared(nh, name + "." + "weight_acc_lim_theta", rclcpp::ParameterValue(optim.weight_acc_lim_theta));
  declare_parameter_if_not_declared(nh, name + "." + "weight_kinematics_nh", rclcpp::ParameterValue(optim.weight_kinematics_nh));
  declare_parameter_if_not_declared(nh, name + "." + "weight_kinematics_forward_drive", rclcpp::ParameterValue(optim.weight_kinematics_forward_drive));
  declare_parameter_if_not_declared(nh, name + "." + "weight_kinematics_turning_radius", rclcpp::ParameterValue(optim.weight_kinematics_turning_radius));
  declare_parameter_if_not_declared(nh, name + "." + "weight_optimaltime", rclcpp::ParameterValue(optim.weight_optimaltime));
  declare_parameter_if_not_declared(nh, name + "." + "weight_shortest_path", rclcpp::ParameterValue(optim.weight_shortest_path));
  declare_parameter_if_not_declared(nh, name + "." + "weight_obstacle", rclcpp::ParameterValue(optim.weight_obstacle));
  declare_parameter_if_not_declared(nh, name + "." + "weight_inflation", rclcpp::ParameterValue(optim.weight_inflation));
  declare_parameter_if_not_declared(nh, name + "." + "weight_dynamic_obstacle", rclcpp::ParameterValue(optim.weight_dynamic_obstacle));
  declare_parameter_if_not_declared(nh, name + "." + "weight_dynamic_obstacle_inflation", rclcpp::ParameterValue(optim.weight_dynamic_obstacle_inflation));
  declare_parameter_if_not_declared(nh, name + "." + "weight_viapoint", rclcpp::ParameterValue(optim.weight_viapoint));
  declare_parameter_if_not_declared(nh, name + "." + "weight_prefer_rotdir", rclcpp::ParameterValue(optim.weight_prefer_rotdir));
  declare_parameter_if_not_declared(nh, name + "." + "weight_adapt_factor", rclcpp::ParameterValue(optim.weight_adapt_factor));
  declare_parameter_if_not_declared(nh, name + "." + "obstacle_cost_exponent", rclcpp::ParameterValue(optim.obstacle_cost_exponent));
  declare_parameter_if_not_declared(nh, name + "." + "weight_velocity_obstacle_ratio", rclcpp::ParameterValue(optim.weight_velocity_obstacle_ratio));

  // Homotopy Class Planner
  declare_parameter_if_not_declared(nh, name + "." + "enable_homotopy_class_planning", rclcpp::ParameterValue(hcp.enable_homotopy_class_planning));
  declare_parameter_if_not_declared(nh, name + "." + "enable_multithreading", rclcpp::ParameterValue(hcp.enable_multithreading));
  declare_parameter_if_not_declared(nh, name + "." + "simple_exploration", rclcpp::ParameterValue(hcp.simple_exploration));
  declare_parameter_if_not_declared(nh, name + "." + "max_number_classes", rclcpp::ParameterValue(hcp.max_number_classes));
  declare_parameter_if_not_declared(nh, name + "." + "selection_obst_cost_scale", rclcpp::ParameterValue(hcp.selection_obst_cost_scale));
  declare_parameter_if_not_declared(nh, name + "." + "selection_prefer_initial_plan", rclcpp::ParameterValue(hcp.selection_prefer_initial_plan));
  declare_parameter_if_not_declared(nh, name + "." + "selection_viapoint_cost_scale", rclcpp::ParameterValue(hcp.selection_viapoint_cost_scale));
  declare_parameter_if_not_declared(nh, name + "." + "selection_cost_hysteresis", rclcpp::ParameterValue(hcp.selection_cost_hysteresis));
  declare_parameter_if_not_declared(nh, name + "." + "selection_alternative_time_cost", rclcpp::ParameterValue(hcp.selection_alternative_time_cost));
  declare_parameter_if_not_declared(nh, name + "." + "switching_blocking_period", rclcpp::ParameterValue(hcp.switching_blocking_period));
  declare_parameter_if_not_declared(nh, name + "." + "roadmap_graph_samples", rclcpp::ParameterValue(hcp.roadmap_graph_no_samples));
  declare_parameter_if_not_declared(nh, name + "." + "roadmap_graph_area_width", rclcpp::ParameterValue(hcp.roadmap_graph_area_width));
  declare_parameter_if_not_declared(nh, name + "." + "roadmap_graph_area_length_scale", rclcpp::ParameterValue(hcp.roadmap_graph_area_length_scale));
  declare_parameter_if_not_declared(nh, name + "." + "h_signature_prescaler", rclcpp::ParameterValue(hcp.h_signature_prescaler));
  declare_parameter_if_not_declared(nh, name + "." + "h_signature_threshold", rclcpp::ParameterValue(hcp.h_signature_threshold));
  declare_parameter_if_not_declared(nh, name + "." + "obstacle_keypoint_offset", rclcpp::ParameterValue(hcp.obstacle_keypoint_offset));
  declare_parameter_if_not_declared(nh, name + "." + "obstacle_heading_threshold", rclcpp::ParameterValue(hcp.obstacle_heading_threshold));
  declare_parameter_if_not_declared(nh, name + "." + "viapoints_all_candidates", rclcpp::ParameterValue(hcp.viapoints_all_candidates));
  declare_parameter_if_not_declared(nh, name + "." + "visualize_hc_graph", rclcpp::ParameterValue(hcp.visualize_hc_graph));
  declare_parameter_if_not_declared(nh, name + "." + "visualize_with_time_as_z_axis_scale", rclcpp::ParameterValue(hcp.visualize_with_time_as_z_axis_scale));
  declare_parameter_if_not_declared(nh, name + "." + "delete_detours_backwards", rclcpp::ParameterValue(hcp.delete_detours_backwards));
  declare_parameter_if_not_declared(nh, name + "." + "detours_orientation_tolerance", rclcpp::ParameterValue(hcp.detours_orientation_tolerance));
  declare_parameter_if_not_declared(nh, name + "." + "length_start_orientation_vector", rclcpp::ParameterValue(hcp.length_start_orientation_vector));
  declare_parameter_if_not_declared(nh, name + "." + "max_ratio_detours_duration_best_duration", rclcpp::ParameterValue(hcp.max_ratio_detours_duration_best_duration));
  declare_parameter_if_not_declared(nh, name + "." + "selection_dropping_probability", rclcpp::ParameterValue(hcp.selection_dropping_probability));

  // Recovery
  declare_parameter_if_not_declared(nh, name + "." + "shrink_horizon_backup", rclcpp::ParameterValue(recovery.shrink_horizon_backup));
  declare_parameter_if_not_declared(nh, name + "." + "shrink_horizon_min_duration", rclcpp::ParameterValue(recovery.shrink_horizon_min_duration));
  declare_parameter_if_not_declared(nh, name + "." + "oscillation_recovery", rclcpp::ParameterValue(recovery.oscillation_recovery));
  declare_parameter_if_not_declared(nh, name + "." + "oscillation_v_eps", rclcpp::ParameterValue(recovery.oscillation_v_eps));
  declare_parameter_if_not_declared(nh, name + "." + "oscillation_omega_eps", rclcpp::ParameterValue(recovery.oscillation_omega_eps));
  declare_parameter_if_not_declared(nh, name + "." + "oscillation_recovery_min_duration", rclcpp::ParameterValue(recovery.oscillation_recovery_min_duration));
  declare_parameter_if_not_declared(nh, name + "." + "oscillation_filter_duration", rclcpp::ParameterValue(recovery.oscillation_filter_duration));
  declare_parameter_if_not_declared(nh, name + "." + "divergence_detection_enable", rclcpp::ParameterValue(recovery.divergence_detection_enable));
  declare_parameter_if_not_declared(nh, name + "." + "divergence_detection_max_chi_squared", rclcpp::ParameterValue(recovery.divergence_detection_max_chi_squared));

  // footprint model
  declare_parameter_if_not_declared(nh, name + "." + "footprint_model.type", rclcpp::ParameterType::PARAMETER_STRING);
}

void TebConfig::loadRosParamFromNodeHandle(const nav2_util::LifecycleNode::SharedPtr nh, const std::string name)
{
  nh->get_parameter_or(name + "." + "odom_topic", odom_topic, odom_topic);
  
  // Trajectory
  nh->get_parameter_or(name + "." + "teb_autosize", trajectory.teb_autosize, trajectory.teb_autosize);
  nh->get_parameter_or(name + "." + "dt_ref", trajectory.dt_ref, trajectory.dt_ref);
  nh->get_parameter_or(name + "." + "dt_hysteresis", trajectory.dt_hysteresis, trajectory.dt_hysteresis);
  nh->get_parameter_or(name + "." + "min_samples", trajectory.min_samples, trajectory.min_samples);
  nh->get_parameter_or(name + "." + "max_samples", trajectory.max_samples, trajectory.max_samples);
  nh->get_parameter_or(name + "." + "global_plan_overwrite_orientation", trajectory.global_plan_overwrite_orientation, trajectory.global_plan_overwrite_orientation);
  nh->get_parameter_or(name + "." + "allow_init_with_backwards_motion", trajectory.allow_init_with_backwards_motion, trajectory.allow_init_with_backwards_motion);
  nh->get_parameter_or(name + "." + "global_plan_viapoint_sep", trajectory.global_plan_viapoint_sep, trajectory.global_plan_viapoint_sep);
  nh->get_parameter_or(name + "." + "via_points_ordered", trajectory.via_points_ordered, trajectory.via_points_ordered);
  nh->get_parameter_or(name + "." + "max_global_plan_lookahead_dist", trajectory.max_global_plan_lookahead_dist, trajectory.max_global_plan_lookahead_dist);
  nh->get_parameter_or(name + "." + "global_plan_prune_distance", trajectory.global_plan_prune_distance, trajectory.global_plan_prune_distance);
  nh->get_parameter_or(name + "." + "exact_arc_length", trajectory.exact_arc_length, trajectory.exact_arc_length);
  nh->get_parameter_or(name + "." + "force_reinit_new_goal_dist", trajectory.force_reinit_new_goal_dist, trajectory.force_reinit_new_goal_dist);
  nh->get_parameter_or(name + "." + "force_reinit_new_goal_angular", trajectory.force_reinit_new_goal_angular, trajectory.force_reinit_new_goal_angular);
  nh->get_parameter_or(name + "." + "feasibility_check_no_poses", trajectory.feasibility_check_no_poses, trajectory.feasibility_check_no_poses);
  nh->get_parameter_or(name + "." + "publish_feedback", trajectory.publish_feedback, trajectory.publish_feedback);
  nh->get_parameter_or(name + "." + "min_resolution_collision_check_angular", trajectory.min_resolution_collision_check_angular, trajectory.min_resolution_collision_check_angular);
  nh->get_parameter_or(name + "." + "control_look_ahead_poses", trajectory.control_look_ahead_poses, trajectory.control_look_ahead_poses);
  nh->get_parameter_or(name + "." + "feasibility_check_lookahead_distance", trajectory.feasibility_check_lookahead_distance, trajectory.feasibility_check_lookahead_distance);

  // Robot
  nh->get_parameter_or(name + "." + "max_vel_x", robot.max_vel_x, robot.max_vel_x);
  nh->get_parameter_or(name + "." + "max_vel_x_backwards", robot.max_vel_x_backwards, robot.max_vel_x_backwards);
  nh->get_parameter_or(name + "." + "max_vel_y", robot.max_vel_y, robot.max_vel_y);
  nh->get_parameter_or(name + "." + "max_vel_theta", robot.max_vel_theta, robot.max_vel_theta);
  nh->get_parameter_or(name + "." + "acc_lim_x", robot.acc_lim_x, robot.acc_lim_x);
  nh->get_parameter_or(name + "." + "acc_lim_y", robot.acc_lim_y, robot.acc_lim_y);
  nh->get_parameter_or(name + "." + "acc_lim_theta", robot.acc_lim_theta, robot.acc_lim_theta);
  nh->get_parameter_or(name + "." + "min_turning_radius", robot.min_turning_radius, robot.min_turning_radius);
  nh->get_parameter_or(name + "." + "wheelbase", robot.wheelbase, robot.wheelbase);
  nh->get_parameter_or(name + "." + "cmd_angle_instead_rotvel", robot.cmd_angle_instead_rotvel, robot.cmd_angle_instead_rotvel);
  nh->get_parameter_or(name + "." + "is_footprint_dynamic", robot.is_footprint_dynamic, robot.is_footprint_dynamic);
  
  // GoalTolerance
  nh->get_parameter_or(name + "." + "free_goal_vel", goal_tolerance.free_goal_vel, goal_tolerance.free_goal_vel);

  // Obstacles
  nh->get_parameter_or(name + "." + "min_obstacle_dist", obstacles.min_obstacle_dist, obstacles.min_obstacle_dist);
  nh->get_parameter_or(name + "." + "inflation_dist", obstacles.inflation_dist, obstacles.inflation_dist);
  nh->get_parameter_or(name + "." + "dynamic_obstacle_inflation_dist", obstacles.dynamic_obstacle_inflation_dist, obstacles.dynamic_obstacle_inflation_dist);
  nh->get_parameter_or(name + "." + "include_dynamic_obstacles", obstacles.include_dynamic_obstacles, obstacles.include_dynamic_obstacles);
  nh->get_parameter_or(name + "." + "include_costmap_obstacles", obstacles.include_costmap_obstacles, obstacles.include_costmap_obstacles);
  nh->get_parameter_or(name + "." + "costmap_obstacles_behind_robot_dist", obstacles.costmap_obstacles_behind_robot_dist, obstacles.costmap_obstacles_behind_robot_dist);
  nh->get_parameter_or(name + "." + "obstacle_poses_affected", obstacles.obstacle_poses_affected, obstacles.obstacle_poses_affected);
  nh->get_parameter_or(name + "." + "legacy_obstacle_association", obstacles.legacy_obstacle_association, obstacles.legacy_obstacle_association);
  nh->get_parameter_or(name + "." + "obstacle_association_force_inclusion_factor", obstacles.obstacle_association_force_inclusion_factor, obstacles.obstacle_association_force_inclusion_factor);
  nh->get_parameter_or(name + "." + "obstacle_association_cutoff_factor", obstacles.obstacle_association_cutoff_factor, obstacles.obstacle_association_cutoff_factor);
  nh->get_parameter_or(name + "." + "costmap_converter_plugin", obstacles.costmap_converter_plugin, obstacles.costmap_converter_plugin);
  nh->get_parameter_or(name + "." + "costmap_converter_spin_thread", obstacles.costmap_converter_spin_thread, obstacles.costmap_converter_spin_thread);
  nh->get_parameter_or(name + "." + "obstacle_proximity_ratio_max_vel", obstacles.obstacle_proximity_ratio_max_vel, obstacles.obstacle_proximity_ratio_max_vel);
  nh->get_parameter_or(name + "." + "obstacle_proximity_lower_bound", obstacles.obstacle_proximity_lower_bound, obstacles.obstacle_proximity_lower_bound);
  nh->get_parameter_or(name + "." + "obstacle_proximity_upper_bound", obstacles.obstacle_proximity_upper_bound, obstacles.obstacle_proximity_upper_bound);
  
  // Optimization
  nh->get_parameter_or(name + "." + "no_inner_iterations", optim.no_inner_iterations, optim.no_inner_iterations);
  nh->get_parameter_or(name + "." + "no_outer_iterations", optim.no_outer_iterations, optim.no_outer_iterations);
  nh->get_parameter_or(name + "." + "optimization_activate", optim.optimization_activate, optim.optimization_activate);
  nh->get_parameter_or(name + "." + "optimization_verbose", optim.optimization_verbose, optim.optimization_verbose);
  nh->get_parameter_or(name + "." + "penalty_epsilon", optim.penalty_epsilon, optim.penalty_epsilon);
  nh->get_parameter_or(name + "." + "weight_max_vel_x", optim.weight_max_vel_x, optim.weight_max_vel_x);
  nh->get_parameter_or(name + "." + "weight_max_vel_y", optim.weight_max_vel_y, optim.weight_max_vel_y);
  nh->get_parameter_or(name + "." + "weight_max_vel_theta", optim.weight_max_vel_theta, optim.weight_max_vel_theta);
  nh->get_parameter_or(name + "." + "weight_acc_lim_x", optim.weight_acc_lim_x, optim.weight_acc_lim_x);
  nh->get_parameter_or(name + "." + "weight_acc_lim_y", optim.weight_acc_lim_y, optim.weight_acc_lim_y);
  nh->get_parameter_or(name + "." + "weight_acc_lim_theta", optim.weight_acc_lim_theta, optim.weight_acc_lim_theta);
  nh->get_parameter_or(name + "." + "weight_kinematics_nh", optim.weight_kinematics_nh, optim.weight_kinematics_nh);
  nh->get_parameter_or(name + "." + "weight_kinematics_forward_drive", optim.weight_kinematics_forward_drive, optim.weight_kinematics_forward_drive);
  nh->get_parameter_or(name + "." + "weight_kinematics_turning_radius", optim.weight_kinematics_turning_radius, optim.weight_kinematics_turning_radius);
  nh->get_parameter_or(name + "." + "weight_optimaltime", optim.weight_optimaltime, optim.weight_optimaltime);
  nh->get_parameter_or(name + "." + "weight_shortest_path", optim.weight_shortest_path, optim.weight_shortest_path);
  nh->get_parameter_or(name + "." + "weight_obstacle", optim.weight_obstacle, optim.weight_obstacle);
  nh->get_parameter_or(name + "." + "weight_inflation", optim.weight_inflation, optim.weight_inflation);
  nh->get_parameter_or(name + "." + "weight_dynamic_obstacle", optim.weight_dynamic_obstacle, optim.weight_dynamic_obstacle);
  nh->get_parameter_or(name + "." + "weight_dynamic_obstacle_inflation", optim.weight_dynamic_obstacle_inflation, optim.weight_dynamic_obstacle_inflation);
  nh->get_parameter_or(name + "." + "weight_viapoint", optim.weight_viapoint, optim.weight_viapoint);
  nh->get_parameter_or(name + "." + "weight_prefer_rotdir", optim.weight_prefer_rotdir, optim.weight_prefer_rotdir);
  nh->get_parameter_or(name + "." + "weight_adapt_factor", optim.weight_adapt_factor, optim.weight_adapt_factor);
  nh->get_parameter_or(name + "." + "obstacle_cost_exponent", optim.obstacle_cost_exponent, optim.obstacle_cost_exponent);
  nh->get_parameter_or(name + "." + "weight_velocity_obstacle_ratio", optim.weight_velocity_obstacle_ratio, optim.weight_velocity_obstacle_ratio);
  
  // Homotopy Class Planner
  nh->get_parameter_or(name + "." + "enable_homotopy_class_planning", hcp.enable_homotopy_class_planning, hcp.enable_homotopy_class_planning);
  nh->get_parameter_or(name + "." + "enable_multithreading", hcp.enable_multithreading, hcp.enable_multithreading);
  nh->get_parameter_or(name + "." + "simple_exploration", hcp.simple_exploration, hcp.simple_exploration);
  nh->get_parameter_or(name + "." + "max_number_classes", hcp.max_number_classes, hcp.max_number_classes);
  nh->get_parameter_or(name + "." + "selection_obst_cost_scale", hcp.selection_obst_cost_scale, hcp.selection_obst_cost_scale);
  nh->get_parameter_or(name + "." + "selection_prefer_initial_plan", hcp.selection_prefer_initial_plan, hcp.selection_prefer_initial_plan);
  nh->get_parameter_or(name + "." + "selection_viapoint_cost_scale", hcp.selection_viapoint_cost_scale, hcp.selection_viapoint_cost_scale);
  nh->get_parameter_or(name + "." + "selection_cost_hysteresis", hcp.selection_cost_hysteresis, hcp.selection_cost_hysteresis);
  nh->get_parameter_or(name + "." + "selection_alternative_time_cost", hcp.selection_alternative_time_cost, hcp.selection_alternative_time_cost);
  nh->get_parameter_or(name + "." + "switching_blocking_period", hcp.switching_blocking_period, hcp.switching_blocking_period);
  nh->get_parameter_or(name + "." + "roadmap_graph_samples", hcp.roadmap_graph_no_samples, hcp.roadmap_graph_no_samples);
  nh->get_parameter_or(name + "." + "roadmap_graph_area_width", hcp.roadmap_graph_area_width, hcp.roadmap_graph_area_width);
  nh->get_parameter_or(name + "." + "roadmap_graph_area_length_scale", hcp.roadmap_graph_area_length_scale, hcp.roadmap_graph_area_length_scale);
  nh->get_parameter_or(name + "." + "h_signature_prescaler", hcp.h_signature_prescaler, hcp.h_signature_prescaler);
  nh->get_parameter_or(name + "." + "h_signature_threshold", hcp.h_signature_threshold, hcp.h_signature_threshold);
  nh->get_parameter_or(name + "." + "obstacle_keypoint_offset", hcp.obstacle_keypoint_offset, hcp.obstacle_keypoint_offset);
  nh->get_parameter_or(name + "." + "obstacle_heading_threshold", hcp.obstacle_heading_threshold, hcp.obstacle_heading_threshold);
  nh->get_parameter_or(name + "." + "viapoints_all_candidates", hcp.viapoints_all_candidates, hcp.viapoints_all_candidates);
  nh->get_parameter_or(name + "." + "visualize_hc_graph", hcp.visualize_hc_graph, hcp.visualize_hc_graph);
  nh->get_parameter_or(name + "." + "visualize_with_time_as_z_axis_scale", hcp.visualize_with_time_as_z_axis_scale, hcp.visualize_with_time_as_z_axis_scale);
  nh->get_parameter_or(name + "." + "delete_detours_backwards", hcp.delete_detours_backwards, hcp.delete_detours_backwards);
  nh->get_parameter_or(name + "." + "detours_orientation_tolerance", hcp.detours_orientation_tolerance, hcp.detours_orientation_tolerance);
  nh->get_parameter_or(name + "." + "length_start_orientation_vector", hcp.length_start_orientation_vector, hcp.length_start_orientation_vector);
  nh->get_parameter_or(name + "." + "max_ratio_detours_duration_best_duration", hcp.max_ratio_detours_duration_best_duration, hcp.max_ratio_detours_duration_best_duration);
  nh->get_parameter_or(name + "." + "selection_dropping_probability", hcp.selection_dropping_probability, hcp.selection_dropping_probability);

  // Recovery
  nh->get_parameter_or(name + "." + "shrink_horizon_backup", recovery.shrink_horizon_backup, recovery.shrink_horizon_backup);
  nh->get_parameter_or(name + "." + "shrink_horizon_min_duration", recovery.shrink_horizon_min_duration, recovery.shrink_horizon_min_duration);
  nh->get_parameter_or(name + "." + "oscillation_recovery", recovery.oscillation_recovery, recovery.oscillation_recovery);
  nh->get_parameter_or(name + "." + "oscillation_v_eps", recovery.oscillation_v_eps, recovery.oscillation_v_eps);
  nh->get_parameter_or(name + "." + "oscillation_omega_eps", recovery.oscillation_omega_eps, recovery.oscillation_omega_eps);
  nh->get_parameter_or(name + "." + "oscillation_recovery_min_duration", recovery.oscillation_recovery_min_duration, recovery.oscillation_recovery_min_duration);
  nh->get_parameter_or(name + "." + "oscillation_filter_duration", recovery.oscillation_filter_duration, recovery.oscillation_filter_duration);
  nh->get_parameter_or(name + "." + "divergence_detection_enable", recovery.divergence_detection_enable, recovery.divergence_detection_enable);
  nh->get_parameter_or(name + "." + "divergence_detection_max_chi_squared", recovery.divergence_detection_max_chi_squared, recovery.divergence_detection_max_chi_squared);

  // footprint model
  if (!nh->get_parameter(name + "." + "footprint_model.type", model_name))
  {
    RCLCPP_INFO(logger_, "No robot footprint model specified for trajectory optimization. Using point-shaped model.");
    robot_model = std::make_shared<PointRobotFootprint>();
  }

  // point
  else if (model_name.compare("point") == 0)
  {
    RCLCPP_INFO(logger_, "Footprint model 'point' loaded for trajectory optimization.");
    robot_model = std::make_shared<PointRobotFootprint>();
  }

  // circular
  else if (model_name.compare("circular") == 0)
  {
    declare_parameter_if_not_declared(nh, name + "." + "footprint_model.radius", rclcpp::ParameterType::PARAMETER_DOUBLE);
    // get radius
    double radius;
    if (!nh->get_parameter(name + "." + "footprint_model.radius", radius))
    {
      RCLCPP_ERROR(logger_,
                   "Footprint model 'circular' cannot be loaded for trajectory optimization, since param '%s.footprint_model.radius' does not exist. Using point-model instead.",
                   nh->get_namespace());

      robot_model = std::make_shared<PointRobotFootprint>();
    }
    RCLCPP_INFO(logger_, "Footprint model 'circular' (radius: %fm) loaded for trajectory optimization.", radius);
    robot_model = std::make_shared<CircularRobotFootprint>(radius);
  }


  // line
  else if (model_name.compare("line") == 0)
  {
    declare_parameter_if_not_declared(nh, name + "." + "footprint_model.line_start", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    declare_parameter_if_not_declared(nh, name + "." + "footprint_model.line_end", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    std::vector<double> line_start, line_end;
    // check parameters
    if (!nh->get_parameter(name + "." + "footprint_model.line_start", line_start) || !nh->get_parameter(name + "." + "footprint_model.line_end", line_end))
    {
      RCLCPP_ERROR(logger_,
                   "Footprint model 'line' cannot be loaded for trajectory optimization, since param '%s.footprint_model.line_start' and/or '.../line_end' do not exist. Using point-model instead.",
                   nh->get_namespace());
      robot_model = std::make_shared<PointRobotFootprint>();
    }
    if (line_start.size() != 2 || line_end.size() != 2)
    {
      RCLCPP_ERROR(logger_, "Footprint model 'line' cannot be loaded for trajectory optimization, since param '%s.footprint_model.line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.",
                   nh->get_namespace());
      robot_model = std::make_shared<PointRobotFootprint>();
    }

    RCLCPP_INFO(logger_,
                "Footprint model 'line' (line_start: [%f,%f]m, line_end: [%f,%f]m) loaded for trajectory optimization.",
                line_start[0], line_start[1], line_end[0], line_end[1]);

    robot_model = std::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()));
  }

  // two circles
  else if (model_name.compare("two_circles") == 0)
  {
    rclcpp::Parameter dummy;
    declare_parameter_if_not_declared(nh, name + "." + "footprint_model.front_offset", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter_if_not_declared(nh, name + "." + "footprint_model.front_radius", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter_if_not_declared(nh, name + "." + "footprint_model.rear_offset", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter_if_not_declared(nh, name + "." + "footprint_model.rear_radius", rclcpp::ParameterType::PARAMETER_DOUBLE);
    // check parameters
    if (!nh->get_parameter(name + "." + "footprint_model.front_offset", dummy) || !nh->get_parameter(name + "." + "footprint_model.front_radius", dummy)
        || !nh->get_parameter(name + "." + "footprint_model.rear_offset", dummy) || !nh->get_parameter(name + "." + "footprint_model.rear_radius", dummy))
    {
      RCLCPP_ERROR(logger_,
                   "Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '%s.footprint_model.front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.",
                   nh->get_namespace());
      robot_model = std::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    nh->get_parameter(name + "." + "footprint_model.front_offset", front_offset);
    nh->get_parameter(name + "." + "footprint_model.front_radius", front_radius);
    nh->get_parameter(name + "." + "footprint_model.rear_offset", rear_offset);
    nh->get_parameter(name + "." + "footprint_model.rear_radius", rear_radius);
    RCLCPP_INFO(logger_,
                "Footprint model 'two_circles' (front_offset: %fm, front_radius: %fm, rear_offset: %fm, rear_radius: %fm) loaded for trajectory optimization.",
                front_offset, front_radius, rear_offset, rear_radius);

    robot_model = std::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }

  // polygon
  else if (model_name.compare("polygon") == 0)
  {
    declare_parameter_if_not_declared(nh, name + "." + "footprint_model.vertices", rclcpp::ParameterType::PARAMETER_STRING);
    // check parameters
    std::string footprint_string;
    if (!nh->get_parameter(name + "." + "footprint_model.vertices", footprint_string) )
    {
      RCLCPP_ERROR(logger_,
                   "Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '%s.footprint_model.vertices' does not exist. Using point-model instead.",
                   nh->get_namespace());

      robot_model = std::make_shared<PointRobotFootprint>();
    }

    std::vector<geometry_msgs::msg::Point> footprint;
    // get vertices
    if (nav2_costmap_2d::makeFootprintFromString(footprint_string, footprint))
    {
      Point2dContainer polygon;
      for(const auto &pt : footprint) {
          polygon.push_back(Eigen::Vector2d(pt.x, pt.y));
      }
      RCLCPP_INFO(logger_, "Footprint model 'polygon' loaded for trajectory optimization.");
      robot_model = std::make_shared<PolygonRobotFootprint>(polygon);
    }
    else
    {
      RCLCPP_ERROR(logger_,
                "Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '%s.footprint_model.vertices' does not define an array of coordinates. Using point-model instead.",
                nh->get_namespace());
      robot_model = std::make_shared<PointRobotFootprint>();
    }

  }
  // otherwise
  else
  {
    RCLCPP_WARN(logger_, "Unknown robot footprint model specified with parameter '%s.footprint_model.type'. Using point model instead.",
                    nh->get_namespace());
    robot_model = std::make_shared<PointRobotFootprint>();
  }


  checkParameters();
  checkDeprecated(nh, name);
}

rcl_interfaces::msg::SetParametersResult
  TebConfig::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  std::lock_guard<std::mutex> l(config_mutex_);

  bool reload_footprint = false;
  
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      // Trajectory
      if (name == node_name + ".teb_autosize") {
        trajectory.teb_autosize = parameter.as_double();
      } else if (name == node_name + ".dt_ref") {
        trajectory.dt_ref = parameter.as_double();
      } else if (name == node_name + ".dt_hysteresis") {
        trajectory.dt_hysteresis = parameter.as_double();
      } else if (name == node_name + ".global_plan_viapoint_sep") {
        trajectory.global_plan_viapoint_sep = parameter.as_double();
      } else if (name == node_name + ".max_global_plan_lookahead_dist") {
        trajectory.max_global_plan_lookahead_dist = parameter.as_double();
      } else if (name == node_name + ".global_plan_prune_distance") {
        trajectory.global_plan_prune_distance = parameter.as_double();
      } else if (name == node_name + ".force_reinit_new_goal_dist") {
        trajectory.force_reinit_new_goal_dist = parameter.as_double();
      } else if (name == node_name + ".force_reinit_new_goal_angular") {
        trajectory.force_reinit_new_goal_angular = parameter.as_double();
      } else if (name == node_name + ".min_resolution_collision_check_angular") {
        trajectory.min_resolution_collision_check_angular = parameter.as_double();
      } else if (name == node_name + ".feasibility_check_lookahead_distance") {
        trajectory.feasibility_check_lookahead_distance = parameter.as_double();
      }
      // Robot
      else if (name == node_name + ".max_vel_x") {
        robot.max_vel_x = parameter.as_double();
        robot.base_max_vel_x = parameter.as_double();
      } else if (name == node_name + ".max_vel_x_backwards") {
        robot.max_vel_x_backwards = parameter.as_double();
        robot.base_max_vel_x_backwards = parameter.as_double();
      } else if (name == node_name + ".max_vel_y") {
        robot.max_vel_y = parameter.as_double();
        robot.base_max_vel_y = parameter.as_double();
      } else if (name == node_name + ".max_vel_theta") {
        robot.max_vel_theta = parameter.as_double();
        robot.base_max_vel_theta = parameter.as_double();
      } else if (name == node_name + ".acc_lim_x") {
        robot.acc_lim_x = parameter.as_double();
      } else if (name == node_name + ".acc_lim_y") {
        robot.acc_lim_y = parameter.as_double();
      } else if (name == node_name + ".acc_lim_theta") {
        robot.acc_lim_theta = parameter.as_double();
      } else if (name == node_name + ".min_turning_radius") {
        robot.min_turning_radius = parameter.as_double();
      } else if (name == node_name + ".wheelbase") {
        robot.wheelbase = parameter.as_double();
      }
      // GoalTolerance
      // Obstacles
      else if (name == node_name + ".min_obstacle_dist") {
        obstacles.min_obstacle_dist = parameter.as_double();
      } else if (name == node_name + ".inflation_dist") {
        obstacles.inflation_dist = parameter.as_double();
      } else if (name == node_name + ".dynamic_obstacle_inflation_dist") {
        obstacles.dynamic_obstacle_inflation_dist = parameter.as_double();
      } else if (name == node_name + ".costmap_obstacles_behind_robot_dist") {
        obstacles.costmap_obstacles_behind_robot_dist = parameter.as_double();
      } else if (name == node_name + ".obstacle_association_force_inclusion_factor") {
        obstacles.obstacle_association_force_inclusion_factor = parameter.as_double();
      } else if (name == node_name + ".obstacle_association_cutoff_factor") {
        obstacles.obstacle_association_cutoff_factor = parameter.as_double();
      } else if (name == node_name + ".obstacle_proximity_ratio_max_vel") {
        obstacles.obstacle_proximity_ratio_max_vel = parameter.as_double();
      } else if (name == node_name + ".obstacle_proximity_lower_bound") {
        obstacles.obstacle_proximity_lower_bound = parameter.as_double();
      } else if (name == node_name + ".obstacle_proximity_upper_bound") {
        obstacles.obstacle_proximity_upper_bound = parameter.as_double();
      }
      // Optimization
      else if (name == node_name + ".penalty_epsilon") {
        optim.penalty_epsilon = parameter.as_double();
      } else if (name == node_name + ".weight_max_vel_x") {
        optim.weight_max_vel_x = parameter.as_double();
      } else if (name == node_name + ".weight_max_vel_y") {
        optim.weight_max_vel_y = parameter.as_double();
      } else if (name == node_name + ".weight_max_vel_theta") {
        optim.weight_max_vel_theta = parameter.as_double();
      } else if (name == node_name + ".weight_acc_lim_x") {
        optim.weight_acc_lim_x = parameter.as_double();
      } else if (name == node_name + ".weight_acc_lim_y") {
        optim.weight_acc_lim_y = parameter.as_double();
      } else if (name == node_name + ".weight_acc_lim_theta") {
        optim.weight_acc_lim_theta = parameter.as_double();
      } else if (name == node_name + ".weight_kinematics_nh") {
        optim.weight_kinematics_nh = parameter.as_double();
      } else if (name == node_name + ".weight_kinematics_forward_drive") {
        optim.weight_kinematics_forward_drive = parameter.as_double();
      } else if (name == node_name + ".weight_kinematics_turning_radius") {
        optim.weight_kinematics_turning_radius = parameter.as_double();
      } else if (name == node_name + ".weight_optimaltime") {
        optim.weight_optimaltime = parameter.as_double();
      } else if (name == node_name + ".weight_shortest_path") {
        optim.weight_shortest_path = parameter.as_double();
      } else if (name == node_name + ".weight_obstacle") {
        optim.weight_obstacle = parameter.as_double();
      } else if (name == node_name + ".weight_inflation") {
        optim.weight_inflation = parameter.as_double();
      } else if (name == node_name + ".weight_dynamic_obstacle") {
        optim.weight_dynamic_obstacle = parameter.as_double();
      } else if (name == node_name + ".weight_dynamic_obstacle_inflation") {
        optim.weight_dynamic_obstacle_inflation = parameter.as_double();
      } else if (name == node_name + ".weight_viapoint") {
        optim.weight_viapoint = parameter.as_double();
      } else if (name == node_name + ".weight_prefer_rotdir") {
        optim.weight_prefer_rotdir = parameter.as_double();
      } else if (name == node_name + ".weight_adapt_factor") {
        optim.weight_adapt_factor = parameter.as_double();
      } else if (name == node_name + ".obstacle_cost_exponent") {
        optim.obstacle_cost_exponent = parameter.as_double();
      }
      // Homotopy Class Planner
      else if (name == node_name + ".selection_cost_hysteresis") {
        hcp.selection_cost_hysteresis = parameter.as_double();
      } else if (name == node_name + ".selection_prefer_initial_plan") {
        hcp.selection_prefer_initial_plan = parameter.as_double();
      } else if (name == node_name + ".selection_obst_cost_scale") {
        hcp.selection_obst_cost_scale = parameter.as_double();
      } else if (name == node_name + ".selection_viapoint_cost_scale") {
        hcp.selection_viapoint_cost_scale = parameter.as_double();
      } else if (name == node_name + ".switching_blocking_period") {
        hcp.switching_blocking_period = parameter.as_double();
      } else if (name == node_name + ".roadmap_graph_area_width") {
        hcp.roadmap_graph_area_width = parameter.as_double();
      } else if (name == node_name + ".roadmap_graph_area_length_scale") {
        hcp.roadmap_graph_area_length_scale = parameter.as_double();
      } else if (name == node_name + ".h_signature_prescaler") {
        hcp.h_signature_prescaler = parameter.as_double();
      } else if (name == node_name + ".h_signature_threshold") {
        hcp.h_signature_threshold = parameter.as_double();
      } else if (name == node_name + ".obstacle_keypoint_offset") {
        hcp.obstacle_keypoint_offset = parameter.as_double();
      } else if (name == node_name + ".obstacle_heading_threshold") {
        hcp.obstacle_heading_threshold = parameter.as_double();
      } else if (name == node_name + ".visualize_with_time_as_z_axis_scale") {
        hcp.visualize_with_time_as_z_axis_scale = parameter.as_double();
      } else if (name == node_name + ".detours_orientation_tolerance") {
        hcp.detours_orientation_tolerance = parameter.as_double();
      } else if (name == node_name + ".length_start_orientation_vector") {
        hcp.length_start_orientation_vector = parameter.as_double();
      } else if (name == node_name + ".max_ratio_detours_duration_best_duration") {
        hcp.max_ratio_detours_duration_best_duration = parameter.as_double();
      } else if (name == node_name + ".selection_dropping_probability") {
        hcp.selection_dropping_probability = parameter.as_double();
      }
      // Recovery
      else if (name == node_name + ".shrink_horizon_min_duration") {
        recovery.shrink_horizon_min_duration = parameter.as_double();
      } else if (name == node_name + ".oscillation_v_eps") {
        recovery.oscillation_v_eps = parameter.as_double();
      } else if (name == node_name + ".oscillation_omega_eps") {
        recovery.oscillation_omega_eps = parameter.as_double();
      } else if (name == node_name + ".oscillation_recovery_min_duration") {
        recovery.oscillation_recovery_min_duration = parameter.as_double();
      } else if (name == node_name + ".oscillation_filter_duration") {
        recovery.oscillation_filter_duration = parameter.as_double();
      } else if (name == node_name + ".divergence_detection_max_chi_squared") {
        recovery.divergence_detection_max_chi_squared = parameter.as_double();
      }
      // Footprint model
      else if (name == node_name + ".footprint_model.radius") {
        reload_footprint = true;
        radius = parameter.as_double();
      } else if (name == node_name + ".footprint_model.front_offset") {
        reload_footprint = true;
        front_offset = parameter.as_double();
      } else if (name == node_name + ".footprint_model.front_radius") {
        reload_footprint = true;
        front_radius = parameter.as_double();
      } else if (name == node_name + ".footprint_model.rear_offset") {
        reload_footprint = true;
        rear_offset = parameter.as_double();
      } else if (name == node_name + ".footprint_model.rear_radius") {
        reload_footprint = true;
        rear_radius = parameter.as_double();
      }
    }

    else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      if (name == node_name + ".footprint_model.line_start") {
        reload_footprint = true;
        line_start = parameter.as_double_array();
      } else if (name == node_name + ".footprint_model.line_end") {
        reload_footprint = true;
        line_end = parameter.as_double_array();
      }
    }

    else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      // Trajectory
      if (name == node_name + ".min_samples") {
        trajectory.min_samples = parameter.as_int();
      } else if (name == node_name + ".max_samples") {
        trajectory.max_samples = parameter.as_int();
      } else if (name == node_name + ".feasibility_check_no_poses") {
        trajectory.feasibility_check_no_poses = parameter.as_int();
      } else if (name == node_name + ".control_look_ahead_poses") {
        trajectory.control_look_ahead_poses = parameter.as_int();
      }
      // Robot
      // GoalTolerance
      // Obstacles
      else if (name == node_name + ".obstacle_poses_affected") {
        obstacles.obstacle_poses_affected = parameter.as_int();
      } else if (name == node_name + ".costmap_converter_rate") {
        obstacles.costmap_converter_rate = parameter.as_int();
      }
      // Optimization
      else if (name == node_name + ".no_inner_iterations") {
        optim.no_inner_iterations = parameter.as_int();
      } else if (name == node_name + ".no_outer_iterations") {
        optim.no_outer_iterations = parameter.as_int();
      }
      // Homotopy Class Planner
      else if (name == node_name + ".max_number_classes") {
        hcp.max_number_classes = parameter.as_int();
      } else if (name == node_name + ".roadmap_graph_no_samples") {
        hcp.roadmap_graph_no_samples = parameter.as_int();
      }
      // Recovery
    }

    else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      // Trajectory
      if (name == node_name + ".global_plan_overwrite_orientation") {
        trajectory.global_plan_overwrite_orientation = parameter.as_bool();
      } else if (name == node_name + ".allow_init_with_backwards_motion") {
        trajectory.allow_init_with_backwards_motion = parameter.as_bool();
      } else if (name == node_name + ".via_points_ordered") {
        trajectory.via_points_ordered = parameter.as_bool();
      } else if (name == node_name + ".exact_arc_length") {
        trajectory.exact_arc_length = parameter.as_bool();
      } else if (name == node_name + ".publish_feedback") {
        trajectory.publish_feedback = parameter.as_bool();
      }
      // Robot
      else if (name == node_name + ".cmd_angle_instead_rotvel") {
        robot.cmd_angle_instead_rotvel = parameter.as_bool();
      } else if (name == node_name + ".is_footprint_dynamic") {
        robot.is_footprint_dynamic = parameter.as_bool();
      }
      // GoalTolerance
      else if (name == node_name + ".free_goal_vel") {
        goal_tolerance.free_goal_vel = parameter.as_bool();
      }
      // Obstacles
      else if (name == node_name + ".include_dynamic_obstacles") {
        obstacles.include_dynamic_obstacles = parameter.as_bool();
      } else if (name == node_name + ".include_costmap_obstacles") {
        obstacles.include_costmap_obstacles = parameter.as_bool();
      } else if (name == node_name + ".legacy_obstacle_association") {
        obstacles.legacy_obstacle_association = parameter.as_bool();
      } else if (name == node_name + ".costmap_converter_spin_thread") {
        obstacles.costmap_converter_spin_thread = parameter.as_bool();
      }
      // Optimization
      else if (name == node_name + ".optimization_activate") {
        optim.optimization_activate = parameter.as_bool();
      } else if (name == node_name + ".optimization_verbose") {
        optim.optimization_verbose = parameter.as_bool();
      }
      // Homotopy Class Planner
      else if (name == node_name + ".enable_homotopy_class_planning") {
        hcp.enable_homotopy_class_planning = parameter.as_bool();
      } else if (name == node_name + ".enable_multithreading") {
        hcp.enable_multithreading = parameter.as_bool();
      } else if (name == node_name + ".simple_exploration") {
        hcp.simple_exploration = parameter.as_bool();
      } else if (name == node_name + ".selection_alternative_time_cost") {
        hcp.selection_alternative_time_cost = parameter.as_bool();
      } else if (name == node_name + ".viapoints_all_candidates") {
        hcp.viapoints_all_candidates = parameter.as_bool();
      } else if (name == node_name + ".visualize_hc_graph") {
        hcp.visualize_hc_graph = parameter.as_bool();
      } else if (name == node_name + ".delete_detours_backwards") {
        hcp.delete_detours_backwards = parameter.as_bool();
      }
      // Recovery
      else if (name == node_name + ".shrink_horizon_backup") {
        recovery.shrink_horizon_backup = parameter.as_bool();
      } else if (name == node_name + ".oscillation_recovery") {
        recovery.oscillation_recovery = parameter.as_bool();
      } else if (name == node_name + ".divergence_detection_enable") {
        recovery.divergence_detection_enable = parameter.as_bool();
      }
    }

    else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      // Trajectory
      // Robot
      // GoalTolerance
      // Obstacles
      if (name == node_name + ".costmap_converter_plugin") {
        obstacles.costmap_converter_plugin = parameter.as_string();
      }
      // Optimization
      // Homotopy Class Planner
      // Recovery
      // Footprint model
      else if (name == node_name + ".footprint_model.type") {
        RCLCPP_WARN(logger_, "Changing footprint model type is not allowed at runtime");
      } else if (name == node_name + ".footprint_model.vertices") {
        reload_footprint = true;
        footprint_string = parameter.as_string();
      }
    }
  }
  checkParameters();

  if (reload_footprint) {
    if (model_name.compare("circular") == 0)
    {
      RCLCPP_INFO(logger_, "Footprint model 'circular' (radius: %fm) reloaded for trajectory optimization.", radius);
      robot_model = std::make_shared<CircularRobotFootprint>(radius);
    }
    else if (model_name.compare("line") == 0) {
      if (line_start.size() != 2 || line_end.size() != 2)
      {
        RCLCPP_ERROR(logger_, "Footprint model 'line' cannot be reloaded for trajectory optimization");
      } else {
        RCLCPP_INFO(logger_,
                    "Footprint model 'line' (line_start: [%f,%f]m, line_end: [%f,%f]m) reloaded for trajectory optimization.",
                    line_start[0], line_start[1], line_end[0], line_end[1]);

        robot_model = std::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()));
      }
    }
    else if (model_name.compare("two_circles") == 0) {
      RCLCPP_INFO(logger_,
                  "Footprint model 'two_circles' (front_offset: %fm, front_radius: %fm, rear_offset: %fm, rear_radius: %fm) loaded for trajectory optimization.",
                  front_offset, front_radius, rear_offset, rear_radius);

      robot_model = std::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
    }
    else if (model_name.compare("polygon") == 0)
    {
      std::vector<geometry_msgs::msg::Point> footprint;
      // get vertices
      if (nav2_costmap_2d::makeFootprintFromString(footprint_string, footprint))
      {
        Point2dContainer polygon;
        for(const auto &pt : footprint) {
            polygon.push_back(Eigen::Vector2d(pt.x, pt.y));
        }
        RCLCPP_INFO(logger_, "Footprint model 'polygon' reloaded for trajectory optimization.");
        robot_model = std::make_shared<PolygonRobotFootprint>(polygon);
      }
      else
      {
        RCLCPP_ERROR(logger_,
                  "Footprint model 'polygon' cannot be reloaded for trajectory optimization, since param 'footprint_model.vertices' does not define an array of coordinates.");
      }
    }
  }
  result.successful = true;
  return result;
}
    
    
void TebConfig::checkParameters() const
{
  //rclcpp::Logger logger_{rclcpp::get_logger("TEBLocalPlanner")};
  // positive backward velocity?
  if (robot.max_vel_x_backwards <= 0)
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  
  // bounds smaller than penalty epsilon
  if (robot.max_vel_x <= optim.penalty_epsilon)
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.max_vel_x_backwards <= optim.penalty_epsilon)
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.max_vel_theta <= optim.penalty_epsilon)
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.acc_lim_x <= optim.penalty_epsilon)
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: acc_lim_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.acc_lim_theta <= optim.penalty_epsilon)
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: acc_lim_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
      
  // dt_ref and dt_hyst
  if (trajectory.dt_ref <= trajectory.dt_hysteresis)
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: dt_ref <= dt_hysteresis. The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at least one of them!");
    
  // min number of samples
  if (trajectory.min_samples <3)
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't enough degrees of freedom to plan a trajectory for you. Please increase ...");
  
  // costmap obstacle behind robot
  if (obstacles.costmap_obstacles_behind_robot_dist < 0)
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: parameter 'costmap_obstacles_behind_robot_dist' should be positive or zero.");
    
  // hcp: obstacle heading threshold
  if (hcp.obstacle_keypoint_offset>=1 || hcp.obstacle_keypoint_offset<=0)
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: parameter obstacle_heading_threshold must be in the interval ]0,1[. 0=0deg opening angle, 1=90deg opening angle.");
  
  // carlike
  if (robot.cmd_angle_instead_rotvel && robot.wheelbase==0)
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but wheelbase is set to zero: undesired behavior.");
  
  if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius==0)
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a diffdrive robot");
  
  // positive weight_adapt_factor
  if (optim.weight_adapt_factor < 1.0)
      RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: parameter weight_adapt_factor shoud be >= 1.0");
  
  if (recovery.oscillation_filter_duration < 0)
      RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: parameter oscillation_filter_duration must be >= 0");
  
  if (optim.weight_optimaltime <= 0)
      RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: parameter weight_optimaltime shoud be > 0 (even if weight_shortest_path is in use)");
}    

void TebConfig::checkDeprecated(const nav2_util::LifecycleNode::SharedPtr nh, const std::string name) const
{
  rclcpp::Parameter dummy;

  if (nh->get_parameter(name + "." + "line_obstacle_poses_affected", dummy) || nh->get_parameter(name + "." + "polygon_obstacle_poses_affected", dummy))
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: 'line_obstacle_poses_affected' and 'polygon_obstacle_poses_affected' are deprecated. They share now the common parameter 'obstacle_poses_affected'.");
  
  if (nh->get_parameter(name + "." + "weight_point_obstacle", dummy) || nh->get_parameter(name + "." + "weight_line_obstacle", dummy) || nh->get_parameter(name + "." + "weight_poly_obstacle", dummy))
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: 'weight_point_obstacle', 'weight_line_obstacle' and 'weight_poly_obstacle' are deprecated. They are replaced by the single param 'weight_obstacle'.");
  
  if (nh->get_parameter(name + "." + "costmap_obstacles_front_only", dummy))
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: 'costmap_obstacles_front_only' is deprecated. It is replaced by 'costmap_obstacles_behind_robot_dist' to define the actual area taken into account.");
  
  if (nh->get_parameter(name + "." + "costmap_emergency_stop_dist", dummy))
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: 'costmap_emergency_stop_dist' is deprecated. You can safely remove it from your parameter config.");
  
  if (nh->get_parameter(name + "." + "alternative_time_cost", dummy))
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: 'alternative_time_cost' is deprecated. It has been replaced by 'selection_alternative_time_cost'.");

  if (nh->get_parameter(name + "." + "global_plan_via_point_sep", dummy))
    RCLCPP_WARN(logger_, "TebLocalPlannerROS() Param Warning: 'global_plan_via_point_sep' is deprecated. It has been replaced by 'global_plan_viapoint_sep' due to consistency reasons.");
}

    
} // namespace teb_local_planner
