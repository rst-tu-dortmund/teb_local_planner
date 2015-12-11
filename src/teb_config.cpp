/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015,
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
  nh.param("global_plan_overwrite_orientation", trajectory.global_plan_overwrite_orientation, trajectory.global_plan_overwrite_orientation);
  nh.param("force_reinit_new_goal_dist", trajectory.force_reinit_new_goal_dist, trajectory.force_reinit_new_goal_dist);
  
  // Robot
  nh.param("max_vel_x", robot.max_vel_x, robot.max_vel_x);
  nh.param("max_vel_x_backwards", robot.max_vel_x_backwards, robot.max_vel_x_backwards);
  nh.param("max_vel_theta", robot.max_vel_theta, robot.max_vel_theta);
  nh.param("acc_lim_x", robot.acc_lim_x, robot.acc_lim_x);
  nh.param("acc_lim_theta", robot.acc_lim_theta, robot.acc_lim_theta);
  
  // GoalTolerance
  nh.param("xy_goal_tolerance", goal_tolerance.xy_goal_tolerance, goal_tolerance.xy_goal_tolerance);
  nh.param("yaw_goal_tolerance", goal_tolerance.yaw_goal_tolerance, goal_tolerance.yaw_goal_tolerance);
  nh.param("free_goal_vel", goal_tolerance.free_goal_vel, goal_tolerance.free_goal_vel);
  
  // Obstacles
  nh.param("min_obstacle_dist", obstacles.min_obstacle_dist, obstacles.min_obstacle_dist);
  nh.param("costmap_emergency_stop_dist", obstacles.costmap_emergency_stop_dist, obstacles.costmap_emergency_stop_dist);
  nh.param("include_costmap_obstacles", obstacles.include_costmap_obstacles, obstacles.include_costmap_obstacles);
  nh.param("costmap_obstacles_front_only", obstacles.costmap_obstacles_front_only, obstacles.costmap_obstacles_front_only);
  nh.param("obstacle_poses_affected", obstacles.obstacle_poses_affected, obstacles.obstacle_poses_affected);
  nh.param("polygon_obstacle_poses_affected", obstacles.polygon_obstacle_poses_affected, obstacles.polygon_obstacle_poses_affected);

  
  // Optimization
  nh.param("no_inner_iterations", optim.no_inner_iterations, optim.no_inner_iterations);
  nh.param("no_outer_iterations", optim.no_outer_iterations, optim.no_outer_iterations);
  nh.param("optimization_activate", optim.optimization_activate, optim.optimization_activate);
  nh.param("optimization_verbose", optim.optimization_verbose, optim.optimization_verbose);
  nh.param("penalty_scale", optim.penalty_scale, optim.penalty_scale);
  nh.param("penalty_epsilon", optim.penalty_epsilon, optim.penalty_epsilon);
  nh.param("weight_max_vel_x", optim.weight_max_vel_x, optim.weight_max_vel_x);
  nh.param("weight_max_vel_theta", optim.weight_max_vel_theta, optim.weight_max_vel_theta);
  nh.param("weight_acc_lim_x", optim.weight_acc_lim_x, optim.weight_acc_lim_x);
  nh.param("weight_acc_lim_theta", optim.weight_acc_lim_theta, optim.weight_acc_lim_theta);
  nh.param("weight_kinematics_nh", optim.weight_kinematics_nh, optim.weight_kinematics_nh);
  nh.param("weight_kinematics_forward_drive", optim.weight_kinematics_forward_drive, optim.weight_kinematics_forward_drive);
  nh.param("weight_optimaltime", optim.weight_optimaltime, optim.weight_optimaltime);
  nh.param("weight_point_obstacle", optim.weight_point_obstacle, optim.weight_point_obstacle);
  nh.param("weight_poly_obstacle", optim.weight_poly_obstacle, optim.weight_poly_obstacle);
  nh.param("weight_dynamic_obstacle", optim.weight_dynamic_obstacle, optim.weight_dynamic_obstacle);    
  nh.param("alternative_time_cost", optim.alternative_time_cost, optim.alternative_time_cost); 
  
  // Homotopy Class Planner
  nh.param("enable_homotopy_class_planning", hcp.enable_homotopy_class_planning, hcp.enable_homotopy_class_planning); 
  nh.param("enable_multithreading", hcp.enable_multithreading, hcp.enable_multithreading); 
  nh.param("simple_exploration", hcp.simple_exploration, hcp.simple_exploration); 
  nh.param("max_number_classes", hcp.max_number_classes, hcp.max_number_classes); 
  nh.param("roadmap_graph_samples", hcp.roadmap_graph_no_samples, hcp.roadmap_graph_no_samples); 
  nh.param("roadmap_graph_area_width", hcp.roadmap_graph_area_width, hcp.roadmap_graph_area_width); 
  nh.param("h_signature_prescaler", hcp.h_signature_prescaler, hcp.h_signature_prescaler); 
  nh.param("h_signature_threshold", hcp.h_signature_threshold, hcp.h_signature_threshold); 
  nh.param("obstacle_keypoint_offset", hcp.obstacle_keypoint_offset, hcp.obstacle_keypoint_offset); 
  nh.param("obstacle_heading_threshold", hcp.obstacle_heading_threshold, hcp.obstacle_heading_threshold); 
  nh.param("visualize_hc_graph", hcp.visualize_hc_graph, hcp.visualize_hc_graph); 
}

void TebConfig::reconfigure(TebLocalPlannerReconfigureConfig& cfg)
{ 
  boost::mutex::scoped_lock l(config_mutex_);
  
  // Trajectory
  trajectory.teb_autosize = cfg.teb_autosize;
  trajectory.dt_ref = cfg.dt_ref;
  trajectory.dt_hysteresis = cfg.dt_hysteresis;
  trajectory.global_plan_overwrite_orientation = cfg.global_plan_overwrite_orientation;
  trajectory.force_reinit_new_goal_dist = cfg.force_reinit_new_goal_dist;
  
  // Robot     
  robot.max_vel_x = cfg.max_vel_x;
  robot.max_vel_x_backwards = cfg.max_vel_x_backwards;
  robot.max_vel_theta = cfg.max_vel_theta;
  robot.acc_lim_x = cfg.acc_lim_x;
  robot.acc_lim_theta = cfg.acc_lim_theta;
  
  // GoalTolerance
  goal_tolerance.xy_goal_tolerance = cfg.xy_goal_tolerance;
  goal_tolerance.yaw_goal_tolerance = cfg.yaw_goal_tolerance;
  goal_tolerance.free_goal_vel = cfg.free_goal_vel;
  
  // Obstacles
  obstacles.min_obstacle_dist = cfg.min_obstacle_dist;
  obstacles.costmap_emergency_stop_dist = cfg.costmap_emergency_stop_dist;
  obstacles.include_costmap_obstacles = cfg.include_costmap_obstacles;
  obstacles.costmap_obstacles_front_only = cfg.costmap_obstacles_front_only;
  obstacles.obstacle_poses_affected = cfg.obstacle_poses_affected;
  obstacles.polygon_obstacle_poses_affected = cfg.polygon_obstacle_poses_affected;

  
  // Optimization
  optim.no_inner_iterations = cfg.no_inner_iterations;
  optim.no_outer_iterations = cfg.no_outer_iterations;
  optim.optimization_activate = cfg.optimization_activate;
  optim.optimization_verbose = cfg.optimization_verbose;
  optim.penalty_scale = cfg.penalty_scale;
  optim.penalty_epsilon = cfg.penalty_epsilon;
  optim.weight_max_vel_x = cfg.weight_max_vel_x;
  optim.weight_max_vel_theta = cfg.weight_max_vel_theta;
  optim.weight_acc_lim_x = cfg.weight_acc_lim_x;
  optim.weight_acc_lim_theta = cfg.weight_acc_lim_theta;
  optim.weight_kinematics_nh = cfg.weight_kinematics_nh;
  optim.weight_kinematics_forward_drive = cfg.weight_kinematics_forward_drive;
  optim.weight_optimaltime = cfg.weight_optimaltime;
  optim.weight_point_obstacle = cfg.weight_point_obstacle;
  optim.weight_poly_obstacle = cfg.weight_poly_obstacle;
  optim.weight_dynamic_obstacle = cfg.weight_dynamic_obstacle;
  optim.alternative_time_cost = cfg.alternative_time_cost;
  
  // Homotopy Class Planner
  hcp.enable_multithreading = cfg.enable_multithreading;
  hcp.simple_exploration = cfg.simple_exploration;
  hcp.max_number_classes = cfg.max_number_classes; 
  
  hcp.obstacle_keypoint_offset = cfg.obstacle_keypoint_offset;
  hcp.obstacle_heading_threshold = cfg.obstacle_heading_threshold;
  hcp.roadmap_graph_no_samples = cfg.roadmap_graph_no_samples;
  hcp.roadmap_graph_area_width = cfg.roadmap_graph_area_width;
  hcp.h_signature_prescaler = cfg.h_signature_prescaler;
  hcp.h_signature_threshold = cfg.h_signature_threshold;
  hcp.visualize_hc_graph = cfg.visualize_hc_graph;
}
    
} // namespace teb_local_planner