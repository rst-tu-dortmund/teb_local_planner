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

#include <teb_local_planner/teb_local_planner_ros.h>

#include <tf_conversions/tf_eigen.h>

#include <boost/algorithm/string.hpp>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"


// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(teb_local_planner, TebLocalPlannerROS, teb_local_planner::TebLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace teb_local_planner
{
  

TebLocalPlannerROS::TebLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), costmap_model_(NULL), dynamic_recfg_(NULL), goal_reached_(false), horizon_reduced_(false),
                                           initialized_(false), costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons")
{
}


TebLocalPlannerROS::~TebLocalPlannerROS()
{
  if (dynamic_recfg_!=NULL)
    delete dynamic_recfg_;
  if (tf_!=NULL)
    delete tf_;
  if (costmap_model_!=NULL)
    delete costmap_model_;
}

void TebLocalPlannerROS::reconfigureCB(TebLocalPlannerReconfigureConfig& config, uint32_t level)
{
  cfg_.reconfigure(config);
}

void TebLocalPlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  // check if the plugin is already initialized
  if(!initialized_)
  {	
    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle nh("~/" + name);
	        
    // get parameters of TebConfig via the nodehandle and override the default config
    cfg_.loadRosParamFromNodeHandle(nh);       
    
    // reserve some memory for obstacles
    obstacles_.reserve(500);
        
    // create visualization instance	
    visualization_ = TebVisualizationPtr(new TebVisualization(nh, cfg_)); 
        
    // create the planner instance
    if (cfg_.hcp.enable_homotopy_class_planning)
    {
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(cfg_, &obstacles_, visualization_));
      ROS_INFO("Parallel planning in distinctive topologies enabled.");
    }
    else
    {
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, visualization_));
      ROS_INFO("Parallel planning in distinctive topologies disabled.");
    }
    
    // init other variables
    tf_ = new tf::TransformListener;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
    
    costmap_model_ = new base_local_planner::CostmapModel(*costmap_);

    global_frame_ = costmap_ros_->getGlobalFrameID();
    cfg_.map_frame = global_frame_; // TODO
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    //Initialize a costmap to polygon converter
    if (!cfg_.obstacles.costmap_converter_plugin.empty())
    {
      try
      {
        costmap_converter_ = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
        std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin);
        // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
        boost::replace_all(converter_name, "::", "/");
        costmap_converter_->initialize(ros::NodeHandle(nh, converter_name));
        costmap_converter_->setCostmap2D(costmap_);
        
        costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_, cfg_.obstacles.costmap_converter_spin_thread);
        ROS_INFO_STREAM("Costmap conversion plugin " << cfg_.obstacles.costmap_converter_plugin << " loaded.");        
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
        costmap_converter_.reset();
      }
    }
    else 
      ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
  
    
    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);    

    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(cfg_.odom_topic);

    // setup dynamic reconfigure
    dynamic_recfg_ = new dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>(nh);
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&TebLocalPlannerROS::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);
        
    // setup callback for custom obstacles
    custom_obst_sub_ = nh.subscribe("obstacles", 1, &TebLocalPlannerROS::customObstacleCB, this);
    
    // set initialized flag
    initialized_ = true;

    ROS_DEBUG("teb_local_planner plugin initialized.");
  }
  else
  {
    ROS_WARN("teb_local_planner has already been initialized, doing nothing.");
  }
}



bool TebLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // check if plugin is initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
  // the local planner checks whether it is required to reinitializes the trajectory or not within each velocity computation step.  
            
  // reset goal_reached_ flag
  goal_reached_ = false;
  
  return true;
}


bool TebLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  // check if plugin initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  goal_reached_ = false;  
  
  // Get robot pose
  tf::Stamped<tf::Pose> robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  robot_pose_ = PoseSE2(robot_pose.getOrigin().x(),robot_pose.getOrigin().y(), tf::getYaw(robot_pose.getRotation()));
    
  // Get robot velocity
  tf::Stamped<tf::Pose> robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_ = tfPoseToEigenVector2dTransRot(robot_vel_tf);
  geometry_msgs::Twist robot_vel_twist;
  robot_vel_twist.linear.x = robot_vel_[0];
  robot_vel_twist.angular.z = robot_vel_[1];
    
  // check if the received robot velocity is identically zero (not just closed to zero)
  // in that case the probability is high, that the odom helper has not received any
  // odom message on the given topic. Unfortunately, the odom helper does not have any exception for that case.
  // We only check translational velocities here in order to query a warning message
  if (robot_vel_tf.getOrigin().getX()==0 && robot_vel_tf.getOrigin().getY()==0)
    ROS_WARN_ONCE("The robot velocity is zero w.r.t to the max. available precision. \
    Often the odom topic is not specified correctly (e.g. with namespaces), please check that. \
    Some robot drivers programmatically set the velocity to zero if it is below a certain treshold, in that case ignore this message. \
    This message will be printed once.");
    
  
  // prune global plan to cut off parts of the past (spatially before the robot)
  pruneGlobalPlan(*tf_, robot_pose, global_plan_);
  
  // Transform global plan to the frame of interest (w.r.t to the local costmap)
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  int goal_idx;
  tf::StampedTransform tf_plan_to_global;
  if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }
  
  // Check if the horizon should be reduced this run
  if (horizon_reduced_)
  {
    // reduce to 50 percent:
    int horizon_reduction = goal_idx/2;
    // we have a small overhead here, since we already transformed 50% more of the trajectory.
    // But that's ok for now, since we do not need to make transformGlobalPlan more complex 
    // and a reduced horizon should occur just rarely.
    int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
    goal_idx -= horizon_reduction;
    if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
      transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
    else goal_idx += horizon_reduction; // this should not happy, but safety first ;-)
  }
  
  // check if global goal is reached
  tf::Stamped<tf::Pose> global_goal;
  tf::poseStampedMsgToTF(global_plan_.back(), global_goal);
  global_goal.setData( tf_plan_to_global * global_goal );
  double dx = global_goal.getOrigin().getX() - robot_pose_.x();
  double dy = global_goal.getOrigin().getY() - robot_pose_.y();
  double delta_orient = g2o::normalize_theta( tf::getYaw(global_goal.getRotation()) - robot_pose_.theta() );
  if(fabs(std::sqrt(dx*dx+dy*dy)) < cfg_.goal_tolerance.xy_goal_tolerance
    && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance)
  {
    goal_reached_ = true;
    return true;
  }
  
  // Return false if the transformed global plan is empty
  if (transformed_plan.empty()) return false;
              
  // Get current goal point (last point of the transformed plan)
  tf::Stamped<tf::Pose> goal_point;
  tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
  robot_goal_.x() = goal_point.getOrigin().getX();
  robot_goal_.y() = goal_point.getOrigin().getY();      
  if (cfg_.trajectory.global_plan_overwrite_orientation)
  {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, goal_point, goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
    transformed_plan.back().pose.orientation = tf::createQuaternionMsgFromYaw(robot_goal_.theta());
  }  
  else
  {
    robot_goal_.theta() = tf::getYaw(goal_point.getRotation());
  }

  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  tf::poseTFToMsg(robot_pose, transformed_plan.front().pose);
    
  
  // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
  if (costmap_converter_)
    updateObstacleContainerWithCostmapConverter();
  else
    updateObstacleContainerWithCostmap();
  
  // also consider custom obstacles (must be called after other updates, since the container is not cleared)
  updateObstacleContainerWithCustomObstacles();
    
  // Do not allow config changes during the following optimization step
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());
    
  // Now perform the actual planning
//   bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_.goal_tolerance.free_goal_vel); // straight line init
  bool success = planner_->plan(transformed_plan, &robot_vel_twist, cfg_.goal_tolerance.free_goal_vel);
  if (!success)
  {
    planner_->clearPlanner(); // force reinitialization for next time
    ROS_WARN("teb_local_planner was not able to obtain a local plan for the current setting.");
    return false;
  }
  
  // Undo temporary horizon reduction
  if (horizon_reduced_ && (ros::Time::now()-horizon_reduced_stamp_).toSec() >= 10 && !planner_->isHorizonReductionAppropriate(transformed_plan)) // 10s are hardcoded for now...
  {
    horizon_reduced_ = false;
    ROS_INFO("Switching back to full horizon length.");
  }
     
  // Check feasibility (but within the first few states only)
  bool feasible = planner_->isTrajectoryFeasible(costmap_model_, footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses);
  if (!feasible)
  {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
   
    if (!horizon_reduced_ && cfg_.trajectory.shrink_horizon_backup && planner_->isHorizonReductionAppropriate(transformed_plan))
    {
      horizon_reduced_ = true;
      ROS_WARN("TebLocalPlannerROS: trajectory is not feasible, but the planner suggests a shorter horizon temporary. Complying with its wish for at least 10s...");
      horizon_reduced_stamp_ = ros::Time::now();
      return true; // commanded velocity is zero for this step
    }
    // now we reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();
    ROS_WARN("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...");
       
    return false;
  }

  // Get the velocity command for this sampling interval
  if (!planner_->getVelocityCommand(cmd_vel.linear.x, cmd_vel.angular.z))
  {
    planner_->clearPlanner();
    ROS_WARN("TebLocalPlannerROS: velocity command invalid. Resetting planner...");
    return false;
  }
  
  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.linear.x, cmd_vel.angular.z, cfg_.robot.max_vel_x, cfg_.robot.max_vel_theta, cfg_.robot.max_vel_x_backwards);

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
  // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
  if (cfg_.robot.cmd_angle_instead_rotvel)
  {
    cmd_vel.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.linear.x, cmd_vel.angular.z, cfg_.robot.wheelbase, 0.95*cfg_.robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.angular.z))
    {
      cmd_vel.linear.x = cmd_vel.angular.z = 0;
      planner_->clearPlanner();
      ROS_WARN("TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...");
      return false;
    }
  }
  
  // Now visualize everything		    
  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishGlobalPlan(global_plan_);
  
  return true;
}


bool TebLocalPlannerROS::isGoalReached()
{
  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    planner_->clearPlanner();
    return true;
  }
  return false;
}



void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
{  
  // Add costmap obstacles if desired
  if (cfg_.obstacles.include_costmap_obstacles)
  {
    // first clear current obstacle vector
    obstacles_.clear();
  
    // now scan costmap for obstacles and add them to the obst_vector
    /*
    unsigned int window_range = static_cast<unsigned int>(tebConfig.costmap_emergency_stop_dist/costmap->getResolution());
    unsigned int robot_xm;
    unsigned int robot_ym;
    costmap->worldToMap(robot_pose[0],robot_pose[1],robot_xm,robot_ym);
    
    for(unsigned int j=std::max((int)robot_xm - (int)window_range,0) ; j<=std::min(robot_xm+window_range,costmap->getSizeInCellsX()-1) ; j++)
    {
	    for(unsigned int k=std::max((int)robot_ym-(int)window_range,0) ; k<=std::min(robot_ym+window_range,costmap->getSizeInCellsY()-1) ;	k++)
	    {
		    if(costmap->getCost(j,k) == costmap_2d::LETHAL_OBSTACLE) return true;
	    }
    }
    */
    
    Eigen::Vector2d robot2goal = robot_goal_.position() - robot_pose_.position();
    
    for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
    {
      for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
      {
        if (costmap_->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE)
        {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
            
          // check if obstacle is interesting (maybe more efficient if the indices are checked before, instead of testing all points inside the loop)
          if ( cfg_.obstacles.costmap_obstacles_front_only && (obs-robot_pose_.position()).dot(robot2goal) < -0.2 )
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
  
  obstacles_.clear();
  
  //Get obstacles from costmap converter
  costmap_converter::PolygonContainerConstPtr polygons =  costmap_converter_->getPolygons();
  if (!polygons)
    return;
  
  for (int i=0; i<polygons->size(); ++i)
  {
    if (polygons->at(i).points.size()==1) // Point
    {
      obstacles_.push_back(ObstaclePtr(new PointObstacle(polygons->at(i).points[0].x, polygons->at(i).points[0].y)));
    }
    else if (polygons->at(i).points.size()==2) // Line
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(polygons->at(i).points[0].x, polygons->at(i).points[0].y,
                                                        polygons->at(i).points[1].x, polygons->at(i).points[1].y )));
    }
    else if (polygons->at(i).points.size()>2) // Real polygon
    {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (int j=0; j<polygons->at(i).points.size(); ++j)
        {
            polyobst->pushBackVertex(polygons->at(i).points[j].x, polygons->at(i).points[j].y);
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
    }
  }
}


void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
{
  // Add custom obstacles obtained via message
  boost::mutex::scoped_lock l(custom_obst_mutex_);

  if (!custom_obstacle_msg_.obstacles.empty())
  {
    // We only use the global header to specify the obstacle coordinate system instead of individual ones
    Eigen::Affine3d obstacle_to_map_eig;
    try 
    {
      tf::StampedTransform obstacle_to_map;
      tf_->waitForTransform(global_frame_, ros::Time(0),
            custom_obstacle_msg_.header.frame_id, ros::Time(0),
            custom_obstacle_msg_.header.frame_id, ros::Duration(0.5));
      tf_->lookupTransform(global_frame_, ros::Time(0),
          custom_obstacle_msg_.header.frame_id, ros::Time(0), 
          custom_obstacle_msg_.header.frame_id, obstacle_to_map);
      tf::transformTFToEigen(obstacle_to_map, obstacle_to_map_eig);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      obstacle_to_map_eig.setIdentity();
    }
    
    for (std::vector<geometry_msgs::PolygonStamped>::const_iterator obst_it = custom_obstacle_msg_.obstacles.begin(); obst_it != custom_obstacle_msg_.obstacles.end(); ++obst_it)
    {
      if (obst_it->polygon.points.size() == 1 ) // point
      {
        Eigen::Vector3d pos( obst_it->polygon.points.front().x, obst_it->polygon.points.front().y, obst_it->polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new PointObstacle( (obstacle_to_map_eig * pos).head(2) )));
      }
      else if (obst_it->polygon.points.size() == 2 ) // line
      {
        Eigen::Vector3d line_start( obst_it->polygon.points.front().x, obst_it->polygon.points.front().y, obst_it->polygon.points.front().z );
        Eigen::Vector3d line_end( obst_it->polygon.points.back().x, obst_it->polygon.points.back().y, obst_it->polygon.points.back().z );
        obstacles_.push_back(ObstaclePtr(new LineObstacle( (obstacle_to_map_eig * line_start).head(2), (obstacle_to_map_eig * line_end).head(2) )));
      }
      else // polygon
      {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (int i=0; i<(int)obst_it->polygon.points.size(); ++i)
        {
          Eigen::Vector3d pos( obst_it->polygon.points[i].x, obst_it->polygon.points[i].y, obst_it->polygon.points[i].z );
          polyobst->pushBackVertex( (obstacle_to_map_eig * pos).head(2) );
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
      }
    }
  }
}

      
Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
{
  Eigen::Vector2d vel;
  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
  return vel;
}
      
      
bool TebLocalPlannerROS::pruneGlobalPlan(const tf::TransformListener& tf, const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return true;
  
  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    tf::StampedTransform global_to_plan_transform;
    tf.lookupTransform(global_plan.front().header.frame_id, global_pose.frame_id_, ros::Time(0), global_to_plan_transform);
    tf::Stamped<tf::Pose> robot;
    robot.setData( global_to_plan_transform * global_pose );
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.getOrigin().x() - it->pose.position.x;
      double dy = robot.getOrigin().y() - it->pose.position.y;
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
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}
      

bool TebLocalPlannerROS::transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
		             const tf::Stamped<tf::Pose>& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame,
		             std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, tf::StampedTransform* tf_plan_to_global) const
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try 
  {
    if (!global_plan.size() > 0)
    {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    tf::StampedTransform plan_to_global_transform;
    tf.waitForTransform(global_frame, ros::Time::now(),
    plan_pose.header.frame_id, plan_pose.header.stamp,
    plan_pose.header.frame_id, ros::Duration(0.5));
    tf.lookupTransform(global_frame, ros::Time(),
    plan_pose.header.frame_id, plan_pose.header.stamp, 
    plan_pose.header.frame_id, plan_to_global_transform);

    //let's get the pose of the robot in the frame of the plan
    tf::Stamped<tf::Pose> robot_pose;
    tf.transformPose(plan_pose.header.frame_id, global_pose, robot_pose);

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                           // located on the border of the local costmap
    

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 0;

    //we need to loop to a point on the plan that is within a certain distance of the robot
    while(i < (int)global_plan.size())
    {
      double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
      double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (sq_dist <= sq_dist_threshold) 
      {
        break;
      }
      ++i;
    }

    tf::Stamped<tf::Pose> tf_pose;
    geometry_msgs::PoseStamped newer_pose;

    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold)
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      tf::poseStampedMsgToTF(pose, tf_pose);
      tf_pose.setData(plan_to_global_transform * tf_pose);
      tf_pose.stamp_ = plan_to_global_transform.stamp_;
      tf_pose.frame_id_ = global_frame;
      tf::poseStampedTFToMsg(tf_pose, newer_pose);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
      double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;

      ++i;
    }
    
    // Modification for teb_local_planner:
    // Return the index of the current goal point (inside the distance threshold)
    if (current_goal_idx) *current_goal_idx = i-1; // minus 1, since i was increased once before leaving the loop
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex) 
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex) 
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }

  return true;
}

    
      
      
double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const tf::Stamped<tf::Pose>& local_goal,
			        int current_goal_idx, const tf::StampedTransform& tf_plan_to_global, int moving_average_length) const
{
  int n = (int)global_plan.size();
  
  // check if we are near the global goal already
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // we've exactly reached the goal
    {
      return tf::getYaw(local_goal.getRotation());
    }
    else
    {
      tf::Quaternion global_orientation;
      tf::quaternionMsgToTF(global_plan.back().pose.orientation, global_orientation);
      return  tf::getYaw(tf_plan_to_global.getRotation() *  global_orientation );
    }     
  }
  
  // reduce number of poses taken into account if the desired number of poses is not available
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
  
  std::vector<double> candidates;
  tf::Stamped<tf::Pose> tf_pose_k = local_goal;
  tf::Stamped<tf::Pose> tf_pose_kp1;
  
  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // Transform pose of the global plan to the planning frame
    const geometry_msgs::PoseStamped& pose = global_plan.at(i+1);
    tf::poseStampedMsgToTF(pose, tf_pose_kp1);
    tf_pose_kp1.setData(tf_plan_to_global * tf_pose_kp1);
      
    // calculate yaw angle  
    candidates.push_back( std::atan2(tf_pose_kp1.getOrigin().getY() - tf_pose_k.getOrigin().getY(),
			  tf_pose_kp1.getOrigin().getX() - tf_pose_k.getOrigin().getX() ) );
    
    if (i<range_end-1) 
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);
}
      
      
void TebLocalPlannerROS::saturateVelocity(double& v, double& omega, double max_vel_x, double max_vel_theta, double max_vel_x_backwards) const
{
  // Limit translational velocity for forward driving
  if (v > max_vel_x)
    v = max_vel_x;
  
  // Limit angular velocity
  if (omega > max_vel_theta)
    omega = max_vel_theta;
  else if (omega < -max_vel_theta)
    omega = -max_vel_theta;
  
  // Limit backwards velocity
  if (max_vel_x_backwards<=0)
  {
    ROS_WARN_ONCE("TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  }
  else if (v < -max_vel_x_backwards)
    v = -max_vel_x_backwards;
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
     
     
void TebLocalPlannerROS::customObstacleCB(const teb_local_planner::ObstacleMsg::ConstPtr& obst_msg)
{
  boost::mutex::scoped_lock l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;  
}
     
         

} // end namespace teb_local_planner


