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

#include "teb_local_planner/teb_local_planner_ros.h"

//#include <tf_conversions/tf_eigen.h>
#include <boost/algorithm/string.hpp>

#include <string>

// pluginlib macros
#include <pluginlib/class_list_macros.hpp>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav_2d_utils/tf_help.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using nav2_util::declare_parameter_if_not_declared;

namespace teb_local_planner
{
  

TebLocalPlannerROS::TebLocalPlannerROS() 
    : costmap_ros_(nullptr), tf_(nullptr), cfg_(new TebConfig()), costmap_model_(nullptr), intra_proc_node_(nullptr),
                                           costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                           custom_via_points_active_(false), no_infeasible_plans_(0),
                                           last_preferred_rotdir_(RotType::none), initialized_(false)
{
}


TebLocalPlannerROS::~TebLocalPlannerROS()
{
}

void TebLocalPlannerROS::initialize(nav2_util::LifecycleNode::SharedPtr node)
{
  // check if the plugin is already initialized
  if(!initialized_)
  {	
    // declare parameters (ros2-dashing)
    intra_proc_node_.reset( 
            new rclcpp::Node("costmap_converter", node->get_namespace(), 
              rclcpp::NodeOptions()));
    cfg_->declareParameters(node, name_);

    // get parameters of TebConfig via the nodehandle and override the default config
    cfg_->loadRosParamFromNodeHandle(node, name_);
    
    // reserve some memory for obstacles
    obstacles_.reserve(500);
        
    // create the planner instance
    if (cfg_->hcp.enable_homotopy_class_planning)
    {
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(node, *cfg_.get(), &obstacles_, visualization_, &via_points_));
      RCLCPP_INFO(logger_, "Parallel planning in distinctive topologies enabled.");
    }
    else
    {
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(node, *cfg_.get(), &obstacles_, visualization_, &via_points_));
      RCLCPP_INFO(logger_, "Parallel planning in distinctive topologies disabled.");
    }
    
    // init other variables
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
    
    costmap_model_ = std::make_shared<dwb_critics::ObstacleFootprintCritic>();
    std::string costmap_model_name("costmap_model");
    costmap_model_->initialize(node, costmap_model_name, name_, costmap_ros_);

    cfg_->map_frame = costmap_ros_->getGlobalFrameID(); // TODO

    //Initialize a costmap to polygon converter
    if (!cfg_->obstacles.costmap_converter_plugin.empty())
    {
      try
      {
        costmap_converter_ = costmap_converter_loader_.createSharedInstance(cfg_->obstacles.costmap_converter_plugin);
        std::string converter_name = costmap_converter_loader_.getName(cfg_->obstacles.costmap_converter_plugin);
        RCLCPP_INFO(logger_, "library path : %s", costmap_converter_loader_.getClassLibraryPath(cfg_->obstacles.costmap_converter_plugin).c_str());
        // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
        boost::replace_all(converter_name, "::", "/");

        costmap_converter_->setOdomTopic(cfg_->odom_topic);
        costmap_converter_->initialize(intra_proc_node_);
        costmap_converter_->setCostmap2D(costmap_);
        const auto rate = std::make_shared<rclcpp::Rate>((double)cfg_->obstacles.costmap_converter_rate);
        costmap_converter_->startWorker(rate, costmap_, cfg_->obstacles.costmap_converter_spin_thread);
        RCLCPP_INFO(logger_, "Costmap conversion plugin %s loaded.", cfg_->obstacles.costmap_converter_plugin.c_str());
      }
      catch(pluginlib::PluginlibException& ex)
      {
        RCLCPP_INFO(logger_,
                    "The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
        costmap_converter_.reset();
      }
    }
    else {
      RCLCPP_INFO(logger_, "No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
    }
  
    
    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);

    // Add callback for dynamic parameters
    dyn_params_handler = node->add_on_set_parameters_callback(
      std::bind(&TebConfig::dynamicParametersCallback, std::ref(cfg_), std::placeholders::_1));

    // validate optimization footprint and costmap footprint
    validateFootprints(cfg_->robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_->obstacles.min_obstacle_dist);
        
    // setup callback for custom obstacles
    custom_obst_sub_ = node->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
                "obstacles", 
                rclcpp::SystemDefaultsQoS(),
                std::bind(&TebLocalPlannerROS::customObstacleCB, this, std::placeholders::_1));

    // setup callback for custom via-points
    via_points_sub_ = node->create_subscription<nav_msgs::msg::Path>(
                "via_points", 
                rclcpp::SystemDefaultsQoS(),
                std::bind(&TebLocalPlannerROS::customViaPointsCB, this, std::placeholders::_1));
    
    // initialize failure detector
    //rclcpp::Node::SharedPtr nh_move_base("~");
    double controller_frequency = 5;
    node->get_parameter("controller_frequency", controller_frequency);
    failure_detector_.setBufferLength(std::round(cfg_->recovery.oscillation_filter_duration*controller_frequency));
    
    // set initialized flag
    initialized_ = true;

    // This should be called since to prevent different time sources exception
    time_last_infeasible_plan_ = clock_->now();
    time_last_oscillation_ = clock_->now();
    RCLCPP_DEBUG(logger_, "teb_local_planner plugin initialized.");
  }
  else
  {
    RCLCPP_INFO(logger_, "teb_local_planner has already been initialized, doing nothing.");
  }
}

void TebLocalPlannerROS::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  nh_ = parent;

  auto node = nh_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  name_ = name;

  initialize(node);
  visualization_ = std::make_shared<TebVisualization>(node, *cfg_);
  visualization_->on_configure();
  planner_->setVisualization(visualization_);
  
  return;
}

void TebLocalPlannerROS::setPlan(const nav_msgs::msg::Path & orig_global_plan)
{
  // check if plugin is initialized
  if(!initialized_)
  {
    RCLCPP_ERROR(logger_, "teb_local_planner has not been initialized, please call initialize() before using this planner");
    return;
  }

  // store the global plan
  global_plan_.clear();
  global_plan_.reserve(orig_global_plan.poses.size());
  for(const auto &in_pose :orig_global_plan.poses) {
    geometry_msgs::msg::PoseStamped out_pose;
    out_pose.pose = in_pose.pose;
    out_pose.header = orig_global_plan.header;
    global_plan_.push_back(out_pose);
  }

  // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
  // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.  
            
  return;
}


geometry_msgs::msg::TwistStamped TebLocalPlannerROS::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
  const geometry_msgs::msg::Twist &velocity, nav2_core::GoalChecker *goal_checker)
{
  // check if plugin initialized
  if(!initialized_)
  {
    throw nav2_core::PlannerException(
      std::string("teb_local_planner has not been initialized, please call initialize() before using this planner")
    );
  }
  geometry_msgs::msg::TwistStamped cmd_vel;
  
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd_vel.twist.linear.x = 0;
  cmd_vel.twist.linear.y = 0;
  cmd_vel.twist.angular.z = 0;

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    cfg_->goal_tolerance.xy_goal_tolerance = pose_tolerance.position.x;
  }
  
  // Get robot pose
  robot_pose_ = PoseSE2(pose.pose);
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header = pose.header;
  robot_pose_.toPoseMsg(robot_pose.pose);
  
  // Get robot velocity
  robot_vel_ = velocity;
  
  // prune global plan to cut off parts of the past (spatially before the robot)
  pruneGlobalPlan(robot_pose, global_plan_, cfg_->trajectory.global_plan_prune_distance);

  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
  int goal_idx;
  geometry_msgs::msg::TransformStamped tf_plan_to_global;
  if (!transformGlobalPlan(global_plan_, robot_pose, *costmap_, cfg_->map_frame, cfg_->trajectory.max_global_plan_lookahead_dist,
                           transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    throw nav2_core::PlannerException(
      std::string("Could not transform the global plan to the frame of the controller")
    );
  }

  // update via-points container
  if (!custom_via_points_active_)
    updateViaPointsContainer(transformed_plan, cfg_->trajectory.global_plan_viapoint_sep);

  // check if we should enter any backup mode and apply settings
  configureBackupModes(transformed_plan, goal_idx);
    
  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    throw nav2_core::PlannerException(
      std::string("Transformed plan is empty. Cannot determine a local plan.")
    );
  }
              
  // Get current goal point (last point of the transformed plan)
  const geometry_msgs::msg::PoseStamped &goal_point = transformed_plan.back();
  robot_goal_.x() = goal_point.pose.position.x;
  robot_goal_.y() = goal_point.pose.position.y;
  if (cfg_->trajectory.global_plan_overwrite_orientation)
  {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, goal_point, goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
    //transformed_plan.back().pose.orientation = tf::createQuaternionMsgFromYaw(robot_goal_.theta());
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    transformed_plan.back().pose.orientation = tf2::toMsg(q);
  }  
  else
  {
    robot_goal_.theta() = tf2::getYaw(goal_point.pose.orientation);
  }

  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // plan only contains the goal
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::msg::PoseStamped()); // insert start (not yet initialized)
  }
  //tf::poseTFToMsg(robot_pose, transformed_plan.front().pose); // update start;
  transformed_plan.front().pose = robot_pose.pose;
    
  // clear currently existing obstacles
  obstacles_.clear();
  
  // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
  if (costmap_converter_)
    updateObstacleContainerWithCostmapConverter();
  else
    updateObstacleContainerWithCostmap();
  
  // also consider custom obstacles (must be called after other updates, since the container is not cleared)
  updateObstacleContainerWithCustomObstacles();
  
    
  // Do not allow config changes during the following optimization step
  std::lock_guard<std::mutex> cfg_lock(cfg_->configMutex());
    
  // Now perform the actual planning
//   bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_->goal_tolerance.free_goal_vel); // straight line init
  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_->goal_tolerance.free_goal_vel);
  if (!success)
  {
    planner_->clearPlanner(); // force reinitialization for next time
    
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    
    throw nav2_core::PlannerException(
      std::string("teb_local_planner was not able to obtain a local plan for the current setting.")
    );
  }

  // Check for divergence
  if (planner_->hasDiverged())
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // Reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 1, "TebLocalPlannerROS: the trajectory has diverged. Resetting planner...");

    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    throw nav2_core::PlannerException(
      std::string("TebLocalPlannerROS: velocity command invalid (hasDiverged). Resetting planner...")
    );
  }
         
  // Check feasibility (but within the first few states only)
  if(cfg_->robot.is_footprint_dynamic)
  {
    // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    std::vector<geometry_msgs::msg::Point> updated_footprint_spec_ = costmap_ros_->getRobotFootprint();
    if (updated_footprint_spec_ != footprint_spec_) {
      updated_footprint_spec_ = footprint_spec_;
      nav2_costmap_2d::calculateMinAndMaxDistances(updated_footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
    }
  }

  bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_->trajectory.feasibility_check_no_poses, cfg_->trajectory.feasibility_check_lookahead_distance);
  if (!feasible)
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
   
    // now we reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();

    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    
    throw nav2_core::PlannerException(
      std::string("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...")
    );
  }

  // Get the velocity command for this sampling interval
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->trajectory.control_look_ahead_poses))
  {
    planner_->clearPlanner();
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    
    throw nav2_core::PlannerException(
      std::string("TebLocalPlannerROS: velocity command invalid. Resetting planner...")
    );
  }
  
  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->robot.max_vel_x, cfg_->robot.max_vel_y,
                   cfg_->robot.max_vel_theta, cfg_->robot.max_vel_x_backwards);

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
  // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
  if (cfg_->robot.cmd_angle_instead_rotvel)
  {
    cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, cfg_->robot.wheelbase, 0.95*cfg_->robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.twist.angular.z))
    {
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
      last_cmd_ = cmd_vel.twist;
      planner_->clearPlanner();

      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = clock_->now();
      
      throw nav2_core::PlannerException(
        std::string("TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...")
      );
    }
  }
  
  // a feasible solution should be found, reset counter
  no_infeasible_plans_ = 0;
  
  // store last command (for recovery analysis etc.)
  last_cmd_ = cmd_vel.twist;
  
  // Now visualize everything    
  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  
  return cmd_vel;
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
{  
  // Add costmap obstacles if desired
  if (cfg_->obstacles.include_costmap_obstacles)
  {
    std::lock_guard<std::recursive_mutex> lock(*costmap_->getMutex());

    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();
    
    for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
    {
      for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
      {
        if (costmap_->getCost(i,j) == nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
            
          // check if obstacle is interesting (e.g. not far behind the robot)
          Eigen::Vector2d obs_dir = obs-robot_pose_.position();
          if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_->obstacles.costmap_obstacles_behind_robot_dist  )
            continue;
            
          obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
        }
      }
    }
  }
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
{
  if (!costmap_converter_)
    return;
    
  //Get obstacles from costmap converter
  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  if (!obstacles)
    return;

  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter_msgs::msg::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::msg::Polygon* polygon = &obstacle->polygon;

    if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
    {
      obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
    }
    else if (polygon->points.size()==1) // Point
    {
      obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
    }
    else if (polygon->points.size()==2) // Line
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        polygon->points[1].x, polygon->points[1].y )));
    }
    else if (polygon->points.size()>2) // Real polygon
    {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (std::size_t j=0; j<polygon->points.size(); ++j)
        {
            polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
    }

    // Set velocity, if obstacle is moving
    if(!obstacles_.empty())
      obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
  }
}


void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
{
  // Add custom obstacles obtained via message
  std::lock_guard<std::mutex> l(custom_obst_mutex_);

  if (!custom_obstacle_msg_.obstacles.empty())
  {
    // We only use the global header to specify the obstacle coordinate system instead of individual ones
    Eigen::Affine3d obstacle_to_map_eig;
    try 
    {
      geometry_msgs::msg::TransformStamped obstacle_to_map = tf_->lookupTransform(
                  cfg_->map_frame, tf2::timeFromSec(0),
                  custom_obstacle_msg_.header.frame_id, tf2::timeFromSec(0),
                  custom_obstacle_msg_.header.frame_id, tf2::durationFromSec(0.5));
      obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
      //tf2::fromMsg(obstacle_to_map.transform, obstacle_to_map_eig);
    }
    catch (tf2::TransformException ex)
    {
      RCLCPP_ERROR(logger_, "%s",ex.what());
      obstacle_to_map_eig.setIdentity();
    }
    
    for (size_t i=0; i<custom_obstacle_msg_.obstacles.size(); ++i)
    {
      if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0 ) // circle
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new CircularObstacle( (obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 ) // point
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new PointObstacle( (obstacle_to_map_eig * pos).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2 ) // line
      {
        Eigen::Vector3d line_start( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        Eigen::Vector3d line_end( custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z );
        obstacles_.push_back(ObstaclePtr(new LineObstacle( (obstacle_to_map_eig * line_start).head(2),
                                                           (obstacle_to_map_eig * line_end).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
      {
        RCLCPP_INFO(logger_, "Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
        continue;
      }
      else // polygon
      {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (size_t j=0; j<custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
        {
          Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z );
          polyobst->pushBackVertex( (obstacle_to_map_eig * pos).head(2) );
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
      }

      // Set velocity, if obstacle is moving
      if(!obstacles_.empty())
        obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
    }
  }
}

void TebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, double min_separation)
{
  via_points_.clear();
  
  if (min_separation<=0)
    return;
  
  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
      continue;
        
    // add via-point
    via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
    prev_idx = i;
  }
  
}
      
//Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
//{
//  Eigen::Vector2d vel;
//  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
//  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
//  return vel;
//}
      
      
bool TebLocalPlannerROS::pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose, std::vector<geometry_msgs::msg::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return true;
  
  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    //geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, tf2::timeFromSec(0));
    geometry_msgs::msg::PoseStamped robot = tf_->transform(
              global_pose,
              global_plan.front().header.frame_id);

    //robot.setData( global_to_plan_transform * global_pose );
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::msg::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::msg::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
         erase_end = it;
         break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;
    
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_DEBUG(logger_, "Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}
      

bool TebLocalPlannerROS::transformGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
                  const geometry_msgs::msg::PoseStamped& global_pose, const nav2_costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                  std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::msg::TransformStamped* tf_plan_to_global) const
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::msg::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try 
  {
    if (global_plan.empty())
    {
      RCLCPP_ERROR(logger_, "Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::msg::TransformStamped plan_to_global_transform = tf_->lookupTransform(
                global_frame, tf2_ros::fromMsg(plan_pose.header.stamp),
                plan_pose.header.frame_id, tf2::timeFromSec(0),
                plan_pose.header.frame_id, tf2::durationFromSec(0.5));

//    tf_->waitForTransform(global_frame, ros::Time::now(),
//    plan_pose.header.frame_id, plan_pose.header.stamp,
//    plan_pose.header.frame_id, ros::Duration(0.5));
//    tf_->lookupTransform(global_frame, ros::Time(),
//    plan_pose.header.frame_id, plan_pose.header.stamp,
//    plan_pose.header.frame_id, plan_to_global_transform);

    //let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose = tf_->transform(global_pose, plan_pose.header.frame_id);

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                           // located on the border of the local costmap
    

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;
    
    //we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (new_sq_dist > sq_dist_threshold)
        break;  // force stop if we have reached the costmap border

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist) // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
      }
    }

    geometry_msgs::msg::PoseStamped newer_pose;
    
    double plan_length = 0; // check cumulative Euclidean distance along the plan
    
    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      //const geometry_msgs::msg::PoseStamped& pose = global_plan[i];
      //tf::poseStampedMsgToTF(pose, tf_pose);
      //tf_pose.setData(plan_to_global_transform * tf_pose);
      tf2::doTransform(global_plan[i], newer_pose, plan_to_global_transform);

//      tf_pose.stamp_ = plan_to_global_transform.stamp_;
//      tf_pose.frame_id_ = global_frame;
//      tf::poseStampedTFToMsg(tf_pose, newer_pose);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      
      // caclulate distance to previous pose
      if (i>0 && max_plan_length>0)
        plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

      ++i;
    }
        
    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
//      tf::poseStampedMsgToTF(global_plan.back(), tf_pose);
//      tf_pose.setData(plan_to_global_transform * tf_pose);
//      tf_pose.stamp_ = plan_to_global_transform.stamp_;
//      tf_pose.frame_id_ = global_frame;
//      tf::poseStampedTFToMsg(tf_pose, newer_pose);
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);
      
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }
    
    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf2::LookupException& ex)
  {
    RCLCPP_ERROR(logger_, "No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf2::ConnectivityException& ex)
  {
    RCLCPP_ERROR(logger_, "Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf2::ExtrapolationException& ex)
  {
    RCLCPP_ERROR(logger_, "Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      RCLCPP_ERROR(logger_, "Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }

  return true;
}

    
      
      
double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan, const geometry_msgs::msg::PoseStamped& local_goal,
                    int current_goal_idx, const geometry_msgs::msg::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
  int n = (int)global_plan.size();
  
  // check if we are near the global goal already
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // we've exactly reached the goal
    {
      return tf2::getYaw(local_goal.pose.orientation);
    }
    else
    {
//      tf::Quaternion global_orientation;
//      tf::quaternionMsgToTF(global_plan.back().pose.orientation, global_orientation);
//      return  tf2::getYaw(tf_plan_to_global.getRotation() *  global_orientation );

        tf2::Quaternion global_orientation, tf_plan_to_global_orientation;
        tf2::fromMsg(global_plan.back().pose.orientation, global_orientation);
        tf2::fromMsg(tf_plan_to_global.transform.rotation, tf_plan_to_global_orientation);

        return tf2::getYaw(tf_plan_to_global_orientation * global_orientation);
    }     
  }
  
  // reduce number of poses taken into account if the desired number of poses is not available
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
  
  std::vector<double> candidates;
  geometry_msgs::msg::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::msg::PoseStamped tf_pose_kp1;
  
  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // Transform pose of the global plan to the planning frame
    const geometry_msgs::msg::PoseStamped& pose = global_plan.at(i+1);
    tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);
      
    // calculate yaw angle  
    candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
              tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );
    
    if (i<range_end-1) 
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);
}
      
      
void TebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards) const
{
  double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
  // Limit translational velocity for forward driving
  if (vx > max_vel_x)
    ratio_x = max_vel_x / vx;
  
  // limit strafing velocity
  if (vy > max_vel_y || vy < -max_vel_y)
    ratio_y = std::abs(max_vel_y / vy);
  
  // Limit angular velocity
  if (omega > max_vel_theta || omega < -max_vel_theta)
    ratio_omega = std::abs(max_vel_theta / omega);
  
  // Limit backwards velocity
  if (max_vel_x_backwards<=0)
  {
    RCLCPP_WARN_ONCE(
                logger_,
                "TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  }
  else if (vx < -max_vel_x_backwards)
    ratio_x = - max_vel_x_backwards / vx;

  if (cfg_->robot.use_proportional_saturation)
  {
    double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
    vx *= ratio;
    vy *= ratio;
    omega *= ratio;
  }
  else
  {
    vx *= ratio_x;
    vy *= ratio_y;
    omega *= ratio_omega;
  }
}
     
     
double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
{
  if (omega==0 || v==0)
    return 0;
    
  double radius = v/omega;
  
  if (fabs(radius) < min_turning_radius)
    radius = double(g2o::sign(radius)) * min_turning_radius; 

  return std::atan(wheelbase / radius);
}
     

void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    RCLCPP_WARN_EXPRESSION(
                logger_, opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}
   
   
   
void TebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,  int& goal_idx)
{
    rclcpp::Time current_time = clock_->now();
    
    // reduced horizon backup mode
    if (cfg_->recovery.shrink_horizon_backup && 
        goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
       (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).seconds() < cfg_->recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
    {
        RCLCPP_INFO_EXPRESSION(
                    logger_,
                    no_infeasible_plans_==1,
                    "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_->recovery.shrink_horizon_min_duration);


        // Shorten horizon if requested
        // reduce to 50 percent:
        int horizon_reduction = goal_idx/2;
        
        if (no_infeasible_plans_ > 9)
        {
            RCLCPP_INFO_EXPRESSION(
                        logger_,
                        no_infeasible_plans_==10,
                        "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
            horizon_reduction /= 2;
        }
        
        // we have a small overhead here, since we already transformed 50% more of the trajectory.
        // But that's ok for now, since we do not need to make transformGlobalPlan more complex 
        // and a reduced horizon should occur just rarely.
        int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
        goal_idx -= horizon_reduction;
        if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
            transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
        else
            goal_idx += horizon_reduction; // this should not happen, but safety first ;-) 
    }
    
    
    // detect and resolve oscillations
    if (cfg_->recovery.oscillation_recovery)
    {
        double max_vel_theta;
        double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_->robot.max_vel_x : cfg_->robot.max_vel_x_backwards;
        if (cfg_->robot.min_turning_radius!=0 && max_vel_current>0)
            max_vel_theta = std::max( max_vel_current/std::abs(cfg_->robot.min_turning_radius),  cfg_->robot.max_vel_theta );
        else
            max_vel_theta = cfg_->robot.max_vel_theta;
        
        failure_detector_.update(last_cmd_, cfg_->robot.max_vel_x, cfg_->robot.max_vel_x_backwards, max_vel_theta,
                               cfg_->recovery.oscillation_v_eps, cfg_->recovery.oscillation_omega_eps);
        
        bool oscillating = failure_detector_.isOscillating();
        bool recently_oscillated = (clock_->now()-time_last_oscillation_).seconds() < cfg_->recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently
        
        if (oscillating)
        {
            if (!recently_oscillated)
            {
                // save current turning direction
                if (robot_vel_.angular.z > 0)
                    last_preferred_rotdir_ = RotType::left;
                else
                    last_preferred_rotdir_ = RotType::right;
                RCLCPP_INFO(logger_, "TebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
            }
            time_last_oscillation_ = clock_->now();
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
        }
        else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
        {
            last_preferred_rotdir_ = RotType::none;
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
            RCLCPP_INFO(logger_, "TebLocalPlannerROS: oscillation recovery disabled/expired.");
        }
    }

}
     

void TebLocalPlannerROS::setSpeedLimit(
    const double & speed_limit, const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    cfg_->robot.max_vel_x = cfg_->robot.base_max_vel_x;
    cfg_->robot.base_max_vel_x_backwards = cfg_->robot.base_max_vel_x_backwards;
    cfg_->robot.base_max_vel_y = cfg_->robot.base_max_vel_y;
    cfg_->robot.base_max_vel_theta = cfg_->robot.base_max_vel_theta;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      cfg_->robot.max_vel_x = cfg_->robot.base_max_vel_x * speed_limit / 100.0;
      cfg_->robot.base_max_vel_x_backwards = cfg_->robot.base_max_vel_x_backwards * speed_limit / 100.0;
      cfg_->robot.base_max_vel_y = cfg_->robot.base_max_vel_y * speed_limit / 100.0;
      cfg_->robot.base_max_vel_theta = cfg_->robot.base_max_vel_theta * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      double max_speed_xy = std::max(
            std::max(cfg_->robot.base_max_vel_x,cfg_->robot.base_max_vel_x_backwards),cfg_->robot.base_max_vel_y);
      if (speed_limit < max_speed_xy) {
        // Handling components and angular velocity changes:
        // Max velocities are being changed in the same proportion
        // as absolute linear speed changed in order to preserve
        // robot moving trajectories to be the same after speed change.
        // G. Doisy: not sure if that's applicable to base_max_vel_x_backwards.
        const double ratio = speed_limit / max_speed_xy;
        cfg_->robot.max_vel_x = cfg_->robot.base_max_vel_x * ratio;
        cfg_->robot.base_max_vel_x_backwards = cfg_->robot.base_max_vel_x_backwards * ratio;
        cfg_->robot.base_max_vel_y = cfg_->robot.base_max_vel_y * ratio;
        cfg_->robot.base_max_vel_theta = cfg_->robot.base_max_vel_theta * ratio;
      }
    }
  }
}

void TebLocalPlannerROS::customObstacleCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obst_msg)
{
  std::lock_guard<std::mutex> l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;  
}

void TebLocalPlannerROS::customViaPointsCB(const nav_msgs::msg::Path::ConstSharedPtr via_points_msg)
{
  RCLCPP_INFO_ONCE(logger_, "Via-points received. This message is printed once.");
  if (cfg_->trajectory.global_plan_viapoint_sep > 0)
  {
    RCLCPP_INFO(logger_, "Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
             "Ignoring custom via-points.");
    custom_via_points_active_ = false;
    return;
  }

  std::lock_guard<std::mutex> l(via_point_mutex_);
  via_points_.clear();
  for (const geometry_msgs::msg::PoseStamped& pose : via_points_msg->poses)
  {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  custom_via_points_active_ = !via_points_.empty();
}

void TebLocalPlannerROS::activate() {
  visualization_->on_activate();

  return;
}
void TebLocalPlannerROS::deactivate() {
  visualization_->on_deactivate();

  return;
}
void TebLocalPlannerROS::cleanup() {
  visualization_->on_cleanup();
  costmap_converter_->stopWorker();
  
  return;
}

} // end namespace teb_local_planner

// register this planner as a nav2_core::Controller plugin
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav2_core::Controller)
