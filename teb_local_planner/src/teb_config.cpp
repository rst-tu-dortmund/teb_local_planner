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
    
void TebConfig::loadRosParamFromNodeHandle(const rclcpp::Node::SharedPtr& nh)
{
  nh->get_parameter_or("odom_topic", odom_topic, odom_topic);
  nh->get_parameter_or("map_frame", map_frame, map_frame);
  
  // Trajectory
  nh->get_parameter_or("teb_autosize", trajectory.teb_autosize, trajectory.teb_autosize);
  nh->get_parameter_or("dt_ref", trajectory.dt_ref, trajectory.dt_ref);
  nh->get_parameter_or("dt_hysteresis", trajectory.dt_hysteresis, trajectory.dt_hysteresis);
  nh->get_parameter_or("min_samples", trajectory.min_samples, trajectory.min_samples);
  nh->get_parameter_or("max_samples", trajectory.max_samples, trajectory.max_samples);
  nh->get_parameter_or("global_plan_overwrite_orientation", trajectory.global_plan_overwrite_orientation, trajectory.global_plan_overwrite_orientation);
  nh->get_parameter_or("allow_init_with_backwards_motion", trajectory.allow_init_with_backwards_motion, trajectory.allow_init_with_backwards_motion);
  nh->get_parameter_or("global_plan_viapoint_sep", trajectory.global_plan_viapoint_sep, trajectory.global_plan_viapoint_sep);
  nh->get_parameter_or("via_points_ordered", trajectory.via_points_ordered, trajectory.via_points_ordered);
  nh->get_parameter_or("max_global_plan_lookahead_dist", trajectory.max_global_plan_lookahead_dist, trajectory.max_global_plan_lookahead_dist);
  nh->get_parameter_or("exact_arc_length", trajectory.exact_arc_length, trajectory.exact_arc_length);
  nh->get_parameter_or("force_reinit_new_goal_dist", trajectory.force_reinit_new_goal_dist, trajectory.force_reinit_new_goal_dist);
  nh->get_parameter_or("feasibility_check_no_poses", trajectory.feasibility_check_no_poses, trajectory.feasibility_check_no_poses);
  nh->get_parameter_or("publish_feedback", trajectory.publish_feedback, trajectory.publish_feedback);
  
  // Robot
  nh->get_parameter_or("max_vel_x", robot.max_vel_x, robot.max_vel_x);
  nh->get_parameter_or("max_vel_x_backwards", robot.max_vel_x_backwards, robot.max_vel_x_backwards);
  nh->get_parameter_or("max_vel_y", robot.max_vel_y, robot.max_vel_y);
  nh->get_parameter_or("max_vel_theta", robot.max_vel_theta, robot.max_vel_theta);
  nh->get_parameter_or("acc_lim_x", robot.acc_lim_x, robot.acc_lim_x);
  nh->get_parameter_or("acc_lim_y", robot.acc_lim_y, robot.acc_lim_y);
  nh->get_parameter_or("acc_lim_theta", robot.acc_lim_theta, robot.acc_lim_theta);
  nh->get_parameter_or("min_turning_radius", robot.min_turning_radius, robot.min_turning_radius);
  nh->get_parameter_or("wheelbase", robot.wheelbase, robot.wheelbase);
  nh->get_parameter_or("cmd_angle_instead_rotvel", robot.cmd_angle_instead_rotvel, robot.cmd_angle_instead_rotvel);
  nh->get_parameter_or("is_footprint_dynamic", robot.is_footprint_dynamic, robot.is_footprint_dynamic);
  
  // GoalTolerance
  nh->get_parameter_or("xy_goal_tolerance", goal_tolerance.xy_goal_tolerance, goal_tolerance.xy_goal_tolerance);
  nh->get_parameter_or("yaw_goal_tolerance", goal_tolerance.yaw_goal_tolerance, goal_tolerance.yaw_goal_tolerance);
  nh->get_parameter_or("free_goal_vel", goal_tolerance.free_goal_vel, goal_tolerance.free_goal_vel);
  nh->get_parameter_or("complete_global_plan", goal_tolerance.complete_global_plan, goal_tolerance.complete_global_plan);

  // Obstacles
  nh->get_parameter_or("min_obstacle_dist", obstacles.min_obstacle_dist, obstacles.min_obstacle_dist);
  nh->get_parameter_or("inflation_dist", obstacles.inflation_dist, obstacles.inflation_dist);
  nh->get_parameter_or("dynamic_obstacle_inflation_dist", obstacles.dynamic_obstacle_inflation_dist, obstacles.dynamic_obstacle_inflation_dist);
  nh->get_parameter_or("include_dynamic_obstacles", obstacles.include_dynamic_obstacles, obstacles.include_dynamic_obstacles);
  nh->get_parameter_or("include_costmap_obstacles", obstacles.include_costmap_obstacles, obstacles.include_costmap_obstacles);
  nh->get_parameter_or("costmap_obstacles_behind_robot_dist", obstacles.costmap_obstacles_behind_robot_dist, obstacles.costmap_obstacles_behind_robot_dist);
  nh->get_parameter_or("obstacle_poses_affected", obstacles.obstacle_poses_affected, obstacles.obstacle_poses_affected);
  nh->get_parameter_or("legacy_obstacle_association", obstacles.legacy_obstacle_association, obstacles.legacy_obstacle_association);
  nh->get_parameter_or("obstacle_association_force_inclusion_factor", obstacles.obstacle_association_force_inclusion_factor, obstacles.obstacle_association_force_inclusion_factor);
  nh->get_parameter_or("obstacle_association_cutoff_factor", obstacles.obstacle_association_cutoff_factor, obstacles.obstacle_association_cutoff_factor);
  nh->get_parameter_or("costmap_converter_plugin", obstacles.costmap_converter_plugin, obstacles.costmap_converter_plugin);
  nh->get_parameter_or("costmap_converter_spin_thread", obstacles.costmap_converter_spin_thread, obstacles.costmap_converter_spin_thread);
  
  // Optimization
  nh->get_parameter_or("no_inner_iterations", optim.no_inner_iterations, optim.no_inner_iterations);
  nh->get_parameter_or("no_outer_iterations", optim.no_outer_iterations, optim.no_outer_iterations);
  nh->get_parameter_or("optimization_activate", optim.optimization_activate, optim.optimization_activate);
  nh->get_parameter_or("optimization_verbose", optim.optimization_verbose, optim.optimization_verbose);
  nh->get_parameter_or("penalty_epsilon", optim.penalty_epsilon, optim.penalty_epsilon);
  nh->get_parameter_or("weight_max_vel_x", optim.weight_max_vel_x, optim.weight_max_vel_x);
  nh->get_parameter_or("weight_max_vel_y", optim.weight_max_vel_y, optim.weight_max_vel_y);
  nh->get_parameter_or("weight_max_vel_theta", optim.weight_max_vel_theta, optim.weight_max_vel_theta);
  nh->get_parameter_or("weight_acc_lim_x", optim.weight_acc_lim_x, optim.weight_acc_lim_x);
  nh->get_parameter_or("weight_acc_lim_y", optim.weight_acc_lim_y, optim.weight_acc_lim_y);
  nh->get_parameter_or("weight_acc_lim_theta", optim.weight_acc_lim_theta, optim.weight_acc_lim_theta);
  nh->get_parameter_or("weight_kinematics_nh", optim.weight_kinematics_nh, optim.weight_kinematics_nh);
  nh->get_parameter_or("weight_kinematics_forward_drive", optim.weight_kinematics_forward_drive, optim.weight_kinematics_forward_drive);
  nh->get_parameter_or("weight_kinematics_turning_radius", optim.weight_kinematics_turning_radius, optim.weight_kinematics_turning_radius);
  nh->get_parameter_or("weight_optimaltime", optim.weight_optimaltime, optim.weight_optimaltime);
  nh->get_parameter_or("weight_obstacle", optim.weight_obstacle, optim.weight_obstacle);
  nh->get_parameter_or("weight_inflation", optim.weight_inflation, optim.weight_inflation);
  nh->get_parameter_or("weight_dynamic_obstacle", optim.weight_dynamic_obstacle, optim.weight_dynamic_obstacle);
  nh->get_parameter_or("weight_dynamic_obstacle_inflation", optim.weight_dynamic_obstacle_inflation, optim.weight_dynamic_obstacle_inflation);
  nh->get_parameter_or("weight_viapoint", optim.weight_viapoint, optim.weight_viapoint);
  nh->get_parameter_or("weight_prefer_rotdir", optim.weight_prefer_rotdir, optim.weight_prefer_rotdir);
  nh->get_parameter_or("weight_adapt_factor", optim.weight_adapt_factor, optim.weight_adapt_factor);
  
  // Homotopy Class Planner
  nh->get_parameter_or("enable_homotopy_class_planning", hcp.enable_homotopy_class_planning, hcp.enable_homotopy_class_planning);
  nh->get_parameter_or("enable_multithreading", hcp.enable_multithreading, hcp.enable_multithreading);
  nh->get_parameter_or("simple_exploration", hcp.simple_exploration, hcp.simple_exploration);
  nh->get_parameter_or("max_number_classes", hcp.max_number_classes, hcp.max_number_classes);
  nh->get_parameter_or("selection_obst_cost_scale", hcp.selection_obst_cost_scale, hcp.selection_obst_cost_scale);
  nh->get_parameter_or("selection_prefer_initial_plan", hcp.selection_prefer_initial_plan, hcp.selection_prefer_initial_plan);
  nh->get_parameter_or("selection_viapoint_cost_scale", hcp.selection_viapoint_cost_scale, hcp.selection_viapoint_cost_scale);
  nh->get_parameter_or("selection_cost_hysteresis", hcp.selection_cost_hysteresis, hcp.selection_cost_hysteresis);
  nh->get_parameter_or("selection_alternative_time_cost", hcp.selection_alternative_time_cost, hcp.selection_alternative_time_cost);
  nh->get_parameter_or("switching_blocking_period", hcp.switching_blocking_period, hcp.switching_blocking_period);
  nh->get_parameter_or("roadmap_graph_samples", hcp.roadmap_graph_no_samples, hcp.roadmap_graph_no_samples);
  nh->get_parameter_or("roadmap_graph_area_width", hcp.roadmap_graph_area_width, hcp.roadmap_graph_area_width);
  nh->get_parameter_or("roadmap_graph_area_length_scale", hcp.roadmap_graph_area_length_scale, hcp.roadmap_graph_area_length_scale);
  nh->get_parameter_or("h_signature_prescaler", hcp.h_signature_prescaler, hcp.h_signature_prescaler);
  nh->get_parameter_or("h_signature_threshold", hcp.h_signature_threshold, hcp.h_signature_threshold);
  nh->get_parameter_or("obstacle_keypoint_offset", hcp.obstacle_keypoint_offset, hcp.obstacle_keypoint_offset);
  nh->get_parameter_or("obstacle_heading_threshold", hcp.obstacle_heading_threshold, hcp.obstacle_heading_threshold);
  nh->get_parameter_or("viapoints_all_candidates", hcp.viapoints_all_candidates, hcp.viapoints_all_candidates);
  nh->get_parameter_or("visualize_hc_graph", hcp.visualize_hc_graph, hcp.visualize_hc_graph);
  nh->get_parameter_or("visualize_with_time_as_z_axis_scale", hcp.visualize_with_time_as_z_axis_scale, hcp.visualize_with_time_as_z_axis_scale);
  
  // Recovery
  
  nh->get_parameter_or("shrink_horizon_backup", recovery.shrink_horizon_backup, recovery.shrink_horizon_backup);
  nh->get_parameter_or("shrink_horizon_min_duration", recovery.shrink_horizon_min_duration, recovery.shrink_horizon_min_duration);
  nh->get_parameter_or("oscillation_recovery", recovery.oscillation_recovery, recovery.oscillation_recovery);
  nh->get_parameter_or("oscillation_v_eps", recovery.oscillation_v_eps, recovery.oscillation_v_eps);
  nh->get_parameter_or("oscillation_omega_eps", recovery.oscillation_omega_eps, recovery.oscillation_omega_eps);
  nh->get_parameter_or("oscillation_recovery_min_duration", recovery.oscillation_recovery_min_duration, recovery.oscillation_recovery_min_duration);
  nh->get_parameter_or("oscillation_filter_duration", recovery.oscillation_filter_duration, recovery.oscillation_filter_duration);

  checkParameters(nh);
  checkDeprecated(nh);
}

// TODO : Dynamic reconfigure is not supported in ROS2 until now
//void TebConfig::reconfigure(TebLocalPlannerReconfigureConfig& cfg)
//{
//  std::lock_guard<std::mutex> l(config_mutex_);
  
//  // Trajectory
//  trajectory.teb_autosize = cfg.teb_autosize;
//  trajectory.dt_ref = cfg.dt_ref;
//  trajectory.dt_hysteresis = cfg.dt_hysteresis;
//  trajectory.global_plan_overwrite_orientation = cfg.global_plan_overwrite_orientation;
//  trajectory.allow_init_with_backwards_motion = cfg.allow_init_with_backwards_motion;
//  trajectory.global_plan_viapoint_sep = cfg.global_plan_viapoint_sep;
//  trajectory.via_points_ordered = cfg.via_points_ordered;
//  trajectory.max_global_plan_lookahead_dist = cfg.max_global_plan_lookahead_dist;
//  trajectory.exact_arc_length = cfg.exact_arc_length;
//  trajectory.force_reinit_new_goal_dist = cfg.force_reinit_new_goal_dist;
//  trajectory.feasibility_check_no_poses = cfg.feasibility_check_no_poses;
//  trajectory.publish_feedback = cfg.publish_feedback;
  
//  // Robot
//  robot.max_vel_x = cfg.max_vel_x;
//  robot.max_vel_x_backwards = cfg.max_vel_x_backwards;
//  robot.max_vel_y = cfg.max_vel_y;
//  robot.max_vel_theta = cfg.max_vel_theta;
//  robot.acc_lim_x = cfg.acc_lim_x;
//  robot.acc_lim_y = cfg.acc_lim_y;
//  robot.acc_lim_theta = cfg.acc_lim_theta;
//  robot.min_turning_radius = cfg.min_turning_radius;
//  robot.wheelbase = cfg.wheelbase;
//  robot.cmd_angle_instead_rotvel = cfg.cmd_angle_instead_rotvel;
  
//  // GoalTolerance
//  goal_tolerance.xy_goal_tolerance = cfg.xy_goal_tolerance;
//  goal_tolerance.yaw_goal_tolerance = cfg.yaw_goal_tolerance;
//  goal_tolerance.free_goal_vel = cfg.free_goal_vel;
  
//  // Obstacles
//  obstacles.min_obstacle_dist = cfg.min_obstacle_dist;
//  obstacles.inflation_dist = cfg.inflation_dist;
//  obstacles.dynamic_obstacle_inflation_dist = cfg.dynamic_obstacle_inflation_dist;
//  obstacles.include_dynamic_obstacles = cfg.include_dynamic_obstacles;
//  obstacles.include_costmap_obstacles = cfg.include_costmap_obstacles;
//  obstacles.legacy_obstacle_association = cfg.legacy_obstacle_association;
//  obstacles.obstacle_association_force_inclusion_factor = cfg.obstacle_association_force_inclusion_factor;
//  obstacles.obstacle_association_cutoff_factor = cfg.obstacle_association_cutoff_factor;
//  obstacles.costmap_obstacles_behind_robot_dist = cfg.costmap_obstacles_behind_robot_dist;
//  obstacles.obstacle_poses_affected = cfg.obstacle_poses_affected;

  
//  // Optimization
//  optim.no_inner_iterations = cfg.no_inner_iterations;
//  optim.no_outer_iterations = cfg.no_outer_iterations;
//  optim.optimization_activate = cfg.optimization_activate;
//  optim.optimization_verbose = cfg.optimization_verbose;
//  optim.penalty_epsilon = cfg.penalty_epsilon;
//  optim.weight_max_vel_x = cfg.weight_max_vel_x;
//  optim.weight_max_vel_y = cfg.weight_max_vel_y;
//  optim.weight_max_vel_theta = cfg.weight_max_vel_theta;
//  optim.weight_acc_lim_x = cfg.weight_acc_lim_x;
//  optim.weight_acc_lim_y = cfg.weight_acc_lim_y;
//  optim.weight_acc_lim_theta = cfg.weight_acc_lim_theta;
//  optim.weight_kinematics_nh = cfg.weight_kinematics_nh;
//  optim.weight_kinematics_forward_drive = cfg.weight_kinematics_forward_drive;
//  optim.weight_kinematics_turning_radius = cfg.weight_kinematics_turning_radius;
//  optim.weight_optimaltime = cfg.weight_optimaltime;
//  optim.weight_obstacle = cfg.weight_obstacle;
//  optim.weight_inflation = cfg.weight_inflation;
//  optim.weight_dynamic_obstacle = cfg.weight_dynamic_obstacle;
//  optim.weight_dynamic_obstacle_inflation = cfg.weight_dynamic_obstacle_inflation;
//  optim.weight_viapoint = cfg.weight_viapoint;
//  optim.weight_adapt_factor = cfg.weight_adapt_factor;
  
//  // Homotopy Class Planner
//  hcp.enable_multithreading = cfg.enable_multithreading;
//  hcp.max_number_classes = cfg.max_number_classes;
//  hcp.selection_cost_hysteresis = cfg.selection_cost_hysteresis;
//  hcp.selection_prefer_initial_plan = cfg.selection_prefer_initial_plan;
//  hcp.selection_obst_cost_scale = cfg.selection_obst_cost_scale;
//  hcp.selection_viapoint_cost_scale = cfg.selection_viapoint_cost_scale;
//  hcp.selection_alternative_time_cost = cfg.selection_alternative_time_cost;
//  hcp.switching_blocking_period = cfg.switching_blocking_period;
  
//  hcp.obstacle_heading_threshold = cfg.obstacle_heading_threshold;
//  hcp.roadmap_graph_no_samples = cfg.roadmap_graph_no_samples;
//  hcp.roadmap_graph_area_width = cfg.roadmap_graph_area_width;
//  hcp.roadmap_graph_area_length_scale = cfg.roadmap_graph_area_length_scale;
//  hcp.h_signature_prescaler = cfg.h_signature_prescaler;
//  hcp.h_signature_threshold = cfg.h_signature_threshold;
//  hcp.viapoints_all_candidates = cfg.viapoints_all_candidates;
//  hcp.visualize_hc_graph = cfg.visualize_hc_graph;
//  hcp.visualize_with_time_as_z_axis_scale = cfg.visualize_with_time_as_z_axis_scale;
  
//  // Recovery
  
//  recovery.shrink_horizon_backup = cfg.shrink_horizon_backup;
//  recovery.oscillation_recovery = cfg.oscillation_recovery;
  
//  checkParameters();
//}
    
    
void TebConfig::checkParameters(const rclcpp::Node::SharedPtr& nh) const
{
  // positive backward velocity?
  if (robot.max_vel_x_backwards <= 0)
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  
  // bounds smaller than penalty epsilon
  if (robot.max_vel_x <= optim.penalty_epsilon)
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.max_vel_x_backwards <= optim.penalty_epsilon)
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.max_vel_theta <= optim.penalty_epsilon)
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.acc_lim_x <= optim.penalty_epsilon)
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: acc_lim_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.acc_lim_theta <= optim.penalty_epsilon)
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: acc_lim_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
      
  // dt_ref and dt_hyst
  if (trajectory.dt_ref <= trajectory.dt_hysteresis)
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: dt_ref <= dt_hysteresis. The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at least one of them!");
    
  // min number of samples
  if (trajectory.min_samples <3)
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't enough degrees of freedom to plan a trajectory for you. Please increase ...");
  
  // costmap obstacle behind robot
  if (obstacles.costmap_obstacles_behind_robot_dist < 0)
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: parameter 'costmap_obstacles_behind_robot_dist' should be positive or zero.");
    
  // hcp: obstacle heading threshold
  if (hcp.obstacle_keypoint_offset>=1 || hcp.obstacle_keypoint_offset<=0)
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: parameter obstacle_heading_threshold must be in the interval ]0,1[. 0=0deg opening angle, 1=90deg opening angle.");
  
  // carlike
  if (robot.cmd_angle_instead_rotvel && robot.wheelbase==0)
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but wheelbase is set to zero: undesired behavior.");
  
  if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius==0)
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a diffdrive robot");
  
  // positive weight_adapt_factor
  if (optim.weight_adapt_factor < 1.0)
      RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: parameter weight_adapt_factor shoud be >= 1.0");
  
  if (recovery.oscillation_filter_duration < 0)
      RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: parameter oscillation_filter_duration must be >= 0");
  
}    

void TebConfig::checkDeprecated(const rclcpp::Node::SharedPtr& nh) const
{
  rclcpp::Parameter dummy;

  if (nh->get_parameter("line_obstacle_poses_affected", dummy) || nh->get_parameter("polygon_obstacle_poses_affected", dummy))
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: 'line_obstacle_poses_affected' and 'polygon_obstacle_poses_affected' are deprecated. They share now the common parameter 'obstacle_poses_affected'.");
  
  if (nh->get_parameter("weight_point_obstacle", dummy) || nh->get_parameter("weight_line_obstacle", dummy) || nh->get_parameter("weight_poly_obstacle", dummy))
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: 'weight_point_obstacle', 'weight_line_obstacle' and 'weight_poly_obstacle' are deprecated. They are replaced by the single param 'weight_obstacle'.");
  
  if (nh->get_parameter("costmap_obstacles_front_only", dummy))
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: 'costmap_obstacles_front_only' is deprecated. It is replaced by 'costmap_obstacles_behind_robot_dist' to define the actual area taken into account.");
  
  if (nh->get_parameter("costmap_emergency_stop_dist", dummy))
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: 'costmap_emergency_stop_dist' is deprecated. You can safely remove it from your parameter config.");
  
  if (nh->get_parameter("alternative_time_cost", dummy))
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: 'alternative_time_cost' is deprecated. It has been replaced by 'selection_alternative_time_cost'.");

  if (nh->get_parameter("global_plan_via_point_sep", dummy))
    RCLCPP_WARN(nh->get_logger(), "TebLocalPlannerROS() Param Warning: 'global_plan_via_point_sep' is deprecated. It has been replaced by 'global_plan_viapoint_sep' due to consistency reasons.");
}

    
} // namespace teb_local_planner
