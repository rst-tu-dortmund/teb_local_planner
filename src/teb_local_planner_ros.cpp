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

#include <teb_local_planner/teb_local_planner_ros.h>

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
  

TebLocalPlannerROS::TebLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), costmap_model_(NULL), dynamic_recfg_(NULL), initialized_(false) 
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
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(cfg_, &obstacles_, visualization_));
    else
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, visualization_));
    
    // init other variables
    tf_ = new tf::TransformListener;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
    
    costmap_model_ = new base_local_planner::CostmapModel(*costmap_);

    global_frame_ = costmap_ros_->getGlobalFrameID();
    cfg_.map_frame = global_frame_; // TODO
    robot_base_frame_ = costmap_ros_->getBaseFrameID();
    
    // Get footprint of the robot and minimum and maximum distance from the center of the robo to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);    

    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(cfg_.odom_topic);

    // setup dynamic reconfigure
    dynamic_recfg_ = new dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>(nh);
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&TebLocalPlannerROS::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);
        
    // set initialized flag
    initialized_ = true;

    // this is only here to make this process visible in the rxlogger right from the start
    ROS_DEBUG("teb_local_planner plugin initialized.");
  }
  else
  {
    ROS_WARN("teb_local_planner has already been initialized, doing nothing.");
  }
}



bool TebLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // check if plugin initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;
  planner_->clearPlanner();
            
  // TODO:
  // maybe perform a few optimization steps here before actually controlling the robot

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
        
  // Get robot pose
  tf::Stamped<tf::Pose> global_pose;
  costmap_ros_->getRobotPose(global_pose);
  robot_pose_ = PoseSE2(global_pose.getOrigin().x(),global_pose.getOrigin().y(), tf::getYaw(global_pose.getRotation()));
    
  // Get robot velocity
  tf::Stamped<tf::Pose> robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_ = tfPoseToEigenVector2dTransRot(robot_vel_tf);
    
  // check if the received robot velocity is identically zero (not just closed to zero)
  // in that case the probability is high, that the odom helper has not received any
  // odom message on the given topic. Unfortunately, the odom helper does not have any exception for that case.
  // We only check translational velocities here in order query a warning message
  if (robot_vel_tf.getOrigin().getX()==0 && robot_vel_tf.getOrigin().getY()==0)
    ROS_WARN("The robot velocity is zero w.r.t to the max. available precision. Often the odom topic is not specified correctly (e.g. with namespaces), please check that.");
    

  // Transform global plan to the frame of interest (w.r.t to the local costmap)
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  unsigned int goal_idx;
  tf::StampedTransform tf_plan_to_global;
  if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }

  // Prune plan based on the position of the robot
  base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);
    
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
  }  
  else
  {
    robot_goal_.theta() = tf::getYaw(goal_point.getRotation());
  }

  // Add costmap obstacles if desired
  if (cfg_.obstacles.include_costmap_obstacles)
  {
    updateCostmapObstacles();
  }
    
  // Do not allow config changes during the following optimization step
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());
    
  // Now perform the actual planning
  bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_.goal_tolerance.free_goal_vel);
  if (!success)
  {
    planner_->clearPlanner(); // force reinitialization for next time
    ROS_WARN("teb_local_planner was not able to obtain a local plan for the current setting.");
    return false;
  }
    
   
  // Check feasibility (but within the first few states, 5 for now)
  bool feasible = planner_->isTrajectoryFeasible(costmap_model_, footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, 1);
  if (!feasible)
  {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    // TODO backup behavior
    // now we reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();
    ROS_WARN("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...");
    return false;
  }

  // Get the velocity command for this sampling interval
  Eigen::Vector2d vel_teb = planner_->getVelocityCommand();

  // Saturate the velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(&vel_teb, cfg_.robot.max_vel_x, cfg_.robot.max_vel_theta, cfg_.robot.max_vel_x_backwards);

  cmd_vel.linear.x = vel_teb.coeffRef(0);
  cmd_vel.angular.z = vel_teb.coeffRef(1);
    
  // Now visualize everything		    
  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishGlobalPlan(global_plan_);
  
  return true;
}


bool TebLocalPlannerROS::isGoalReached()
{
  if (! initialized_ )
  {
    ROS_ERROR("teb_multi_planner: isGoalReached() - please call initialize() before using this planner");
    return false;
  }
            
  if (global_plan_.empty())
  {
    ROS_INFO("teb_multi_planner: isGoalReached() - global plan is empty.");
    planner_->clearPlanner();
    return true;			
  }
            
  tf::Stamped<tf::Pose> global_pose;
  if(!costmap_ros_->getRobotPose(global_pose)) 
    return true;

  geometry_msgs::PoseStamped goal_pose;
  try
  {
    tf_->transformPose(cfg_.map_frame,global_plan_.back(),goal_pose);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  Eigen::Vector2d deltaS;
  deltaS.x() = goal_pose.pose.position.x - global_pose.getOrigin().getX();
  deltaS.y() = goal_pose.pose.position.y - global_pose.getOrigin().getY();
  if(fabs(deltaS.norm())<cfg_.goal_tolerance.xy_goal_tolerance)
  {
    ROS_INFO("GOAL Reached!");
    planner_->clearPlanner();
//  obst_vector.clear();
    return true;
  }

  return false;
}



void TebLocalPlannerROS::updateCostmapObstacles()
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

      
Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
{
  Eigen::Vector2d vel;
  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
  return vel;
}
      
      

bool TebLocalPlannerROS::transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
		             const tf::Stamped<tf::Pose>& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame,
		             std::vector<geometry_msgs::PoseStamped>& transformed_plan, unsigned int* current_goal_idx, tf::StampedTransform* tf_plan_to_global) const
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try 
  {
    if (!global_plan.size() > 0)
    {
      ROS_ERROR("Received plan with zero length");
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

    unsigned int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 0;

    //we need to loop to a point on the plan that is within a certain distance of the robot
    while(i < (unsigned int)global_plan.size())
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
    while(i < (unsigned int)global_plan.size() && sq_dist <= sq_dist_threshold)
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
			        unsigned int current_goal_idx, const tf::StampedTransform& tf_plan_to_global, unsigned int moving_average_length) const
{
  unsigned int n = (unsigned int)global_plan.size();
  
  // check if we are near the global goal already
  if (current_goal_idx >= n-moving_average_length)
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
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we are check the vicinity of the goal before
  
  std::vector<double> candidates;
  tf::Stamped<tf::Pose> tf_pose_k = local_goal;
  tf::Stamped<tf::Pose> tf_pose_kp1;
  
  unsigned int range_end = current_goal_idx + moving_average_length;
  for (unsigned int i = current_goal_idx; i < range_end; ++i)
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
      
      
void TebLocalPlannerROS::saturateVelocity(Eigen::Vector2d* velocity, double max_vel_x, double max_vel_theta, double max_vel_x_backwards)
{
  // Limit translational velocity for forward driving
  if (velocity->x() > max_vel_x)
    velocity->x() = max_vel_x;
  
  // Limit angular velocity
  if (velocity->y() > max_vel_theta)
    velocity->y() = max_vel_theta;
  else if (velocity->y() < -max_vel_theta)
    velocity->y() = -max_vel_theta;
  
  // Limit backwards velocity
  if (max_vel_x_backwards<=0)
    ROS_WARN_ONCE("TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  
  if (velocity->x() < -max_vel_x_backwards)
    velocity->x() = -max_vel_x_backwards;
}
      

} // end namespace teb_local_planner


