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

// ros stuff
#include "teb_local_planner/visualization.h"
#include "teb_local_planner/optimal_planner.h"

namespace teb_local_planner
{

void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path,
                 rclcpp::Publisher<nav_msgs::msg::Path> *pub) {
    if(path.empty())
        return;

    nav_msgs::msg::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    pub->publish(gui_path);
}

TebVisualization::TebVisualization(const rclcpp_lifecycle::LifecycleNode::SharedPtr & nh, const TebConfig& cfg) : nh_(nh), cfg_(&cfg), initialized_(false)
{
}

void TebVisualization::publishGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan) const
{
  if ( printErrorWhenNotInitialized() )
    return;
  publishPlan(global_plan, global_plan_pub_.get());
}

void TebVisualization::publishLocalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& local_plan) const
{
  if ( printErrorWhenNotInitialized() )
    return;
  publishPlan(local_plan, local_plan_pub_.get());
}

void TebVisualization::publishLocalPlanAndPoses(const TimedElasticBand& teb) const
{
  if ( printErrorWhenNotInitialized() )
    return;
  
    // create path msg
    nav_msgs::msg::Path teb_path;
    teb_path.header.frame_id = cfg_->map_frame;
    teb_path.header.stamp = nh_->now();
    
    // create pose_array (along trajectory)
    geometry_msgs::msg::PoseArray teb_poses;
    teb_poses.header.frame_id = teb_path.header.frame_id;
    teb_poses.header.stamp = teb_path.header.stamp;
    
    // fill path msgs with teb configurations
    for (int i=0; i < teb.sizePoses(); i++)
    {
      geometry_msgs::msg::PoseStamped pose;

      pose.header.frame_id = teb_path.header.frame_id;
      pose.header.stamp = teb_path.header.stamp;
      teb.Pose(i).toPoseMsg(pose.pose);
      pose.pose.position.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*teb.getSumOfTimeDiffsUpToIdx(i);
      teb_path.poses.push_back(pose);
      teb_poses.poses.push_back(pose.pose);
    }
    local_plan_pub_->publish(teb_path);
    teb_poses_pub_->publish(teb_poses);
}



void TebVisualization::publishRobotFootprintModel(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model, const std::string& ns,
                                                  const std_msgs::msg::ColorRGBA &color)
{
  if ( printErrorWhenNotInitialized() )
    return;
  
  std::vector<visualization_msgs::msg::Marker> markers;
  robot_model.visualizeRobot(current_pose, markers, color);
  if (markers.empty())
    return;
  
  int idx = 1000000;  // avoid overshadowing by obstacles
  for (std::vector<visualization_msgs::msg::Marker>::iterator marker_it = markers.begin(); marker_it != markers.end(); ++marker_it, ++idx)
  {
    marker_it->header.frame_id = cfg_->map_frame;
    marker_it->header.stamp = nh_->now();
    marker_it->action = visualization_msgs::msg::Marker::ADD;
    marker_it->ns = ns;
    marker_it->id = idx;
    marker_it->lifetime = rclcpp::Duration(2, 0);
    teb_marker_pub_->publish(*marker_it);
  }
  
}

void TebVisualization::publishInfeasibleRobotPose(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model)
{
  publishRobotFootprintModel(current_pose, robot_model, "InfeasibleRobotPoses", toColorMsg(0.5, 0.8, 0.0, 0.0));
}


void TebVisualization::publishObstacles(const ObstContainer& obstacles) const
{
  if ( obstacles.empty() || printErrorWhenNotInitialized() )
    return;
  
  // Visualize point obstacles
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = cfg_->map_frame;
    marker.header.stamp = nh_->now();
    marker.ns = "PointObstacles";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration(2, 0);
    marker.pose.orientation.w = 1.0;
    
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {
      std::shared_ptr<PointObstacle> pobst = std::dynamic_pointer_cast<PointObstacle>(*obst);
      if (!pobst)
        continue;

      if (cfg_->hcp.visualize_with_time_as_z_axis_scale < 0.001)
      {
        geometry_msgs::msg::Point point;
        point.x = pobst->x();
        point.y = pobst->y();
        point.z = 0;
        marker.points.push_back(point);
      }
      else // Spatiotemporally point obstacles become a line
      {
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        geometry_msgs::msg::Point start;
        start.x = pobst->x();
        start.y = pobst->y();
        start.z = 0;
        marker.points.push_back(start);

        geometry_msgs::msg::Point end;
        double t = 20;
        Eigen::Vector2d pred;
        pobst->predictCentroidConstantVelocity(t, pred);
        end.x = pred[0];
        end.y = pred[1];
        end.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*t;
        marker.points.push_back(end);
      }
    }
    
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    teb_marker_pub_->publish( marker );
  }
  
  // Visualize circular obstacles
  {
    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {
      std::shared_ptr<CircularObstacle> pobst = std::dynamic_pointer_cast<CircularObstacle>(*obst);
      if (!pobst)
        continue;

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = nh_->now();
      marker.ns = "CircularObstacles";
      marker.id = idx++;
      marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration(2, 0);
      geometry_msgs::msg::Point point;
      point.x = pobst->x();
      point.y = pobst->y();
      point.z = 0;
      marker.points.push_back(point);

      marker.scale.x = pobst->radius();
      marker.scale.y = pobst->radius();
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      teb_marker_pub_->publish( marker );
    }
  }

  // Visualize line obstacles
  {
    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {	
      std::shared_ptr<LineObstacle> pobst = std::dynamic_pointer_cast<LineObstacle>(*obst);
      if (!pobst)
        continue;
      
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = nh_->now();
      marker.ns = "LineObstacles";
      marker.id = idx++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration(2, 0);
      marker.pose.orientation.w = 1.0;

      geometry_msgs::msg::Point start;
      start.x = pobst->start().x();
      start.y = pobst->start().y();
      start.z = 0;
      marker.points.push_back(start);
      geometry_msgs::msg::Point end;
      end.x = pobst->end().x();
      end.y = pobst->end().y();
      end.z = 0;
      marker.points.push_back(end);
  
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      
      teb_marker_pub_->publish( marker );
    }
  }
  

  // Visualize polygon obstacles
  {
    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {	
      std::shared_ptr<PolygonObstacle> pobst = std::dynamic_pointer_cast<PolygonObstacle>(*obst);
      if (!pobst)
        continue;
      
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = nh_->now();
      marker.ns = "PolyObstacles";
      marker.id = idx++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration(2, 0);
      marker.pose.orientation.w = 1.0;
      
      for (Point2dContainer::const_iterator vertex = pobst->vertices().begin(); vertex != pobst->vertices().end(); ++vertex)
      {
        geometry_msgs::msg::Point point;
        point.x = vertex->x();
        point.y = vertex->y();
        point.z = 0;
        marker.points.push_back(point);
      }
      
      // Also add last point to close the polygon
      // but only if polygon has more than 2 points (it is not a line)
      if (pobst->vertices().size() > 2)
      {
        geometry_msgs::msg::Point point;
        point.x = pobst->vertices().front().x();
        point.y = pobst->vertices().front().y();
        point.z = 0;
        marker.points.push_back(point);
      }
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      
      teb_marker_pub_->publish( marker );
    }
  }
}


void TebVisualization::publishViaPoints(const std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns) const
{
  if ( via_points.empty() || printErrorWhenNotInitialized() )
    return;
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = nh_->now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(2, 0);
  marker.pose.orientation.w = 1.0;
  
  for (std::size_t i=0; i < via_points.size(); ++i)
  {
    geometry_msgs::msg::Point point;
    point.x = via_points[i].x();
    point.y = via_points[i].y();
    point.z = 0;
    marker.points.push_back(point);
  }
  
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  teb_marker_pub_->publish( marker );
}

void TebVisualization::publishTebContainer(const TebOptPlannerContainer& teb_planner, const std::string& ns)
{
if ( printErrorWhenNotInitialized() )
    return;
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = nh_->now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  
  // Iterate through teb pose sequence
  for( TebOptPlannerContainer::const_iterator it_teb = teb_planner.begin(); it_teb != teb_planner.end(); ++it_teb )
  {	  
    // iterate single poses
    PoseSequence::const_iterator it_pose = it_teb->get()->teb().poses().begin();
    TimeDiffSequence::const_iterator it_timediff = it_teb->get()->teb().timediffs().begin();
    PoseSequence::const_iterator it_pose_end = it_teb->get()->teb().poses().end();
    std::advance(it_pose_end, -1); // since we are interested in line segments, reduce end iterator by one.
    double time = 0;

    while (it_pose != it_pose_end)
    {
      geometry_msgs::msg::Point point_start;
      point_start.x = (*it_pose)->x();
      point_start.y = (*it_pose)->y();
      point_start.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*time;
      marker.points.push_back(point_start);

      time += (*it_timediff)->dt();

      geometry_msgs::msg::Point point_end;
      point_end.x = (*boost::next(it_pose))->x();
      point_end.y = (*boost::next(it_pose))->y();
      point_end.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*time;
      marker.points.push_back(point_end);
      ++it_pose;
      ++it_timediff;
    }
  }
  marker.scale.x = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.5;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  teb_marker_pub_->publish( marker );
}

void TebVisualization::publishFeedbackMessage(const std::vector< std::shared_ptr<TebOptimalPlanner> >& teb_planners,
                                              unsigned int selected_trajectory_idx, const ObstContainer& obstacles)
{
  teb_msgs::msg::FeedbackMsg msg;
  msg.header.stamp = nh_->now();
  msg.header.frame_id = cfg_->map_frame;
  msg.selected_trajectory_idx = selected_trajectory_idx;
  
  
  msg.trajectories.resize(teb_planners.size());
  
  // Iterate through teb pose sequence
  std::size_t idx_traj = 0;
  for( TebOptPlannerContainer::const_iterator it_teb = teb_planners.begin(); it_teb != teb_planners.end(); ++it_teb, ++idx_traj )
  {   
    msg.trajectories[idx_traj].header = msg.header;
    it_teb->get()->getFullTrajectory(msg.trajectories[idx_traj].trajectory);
  }
  
  // add obstacles
  msg.obstacles_msg.obstacles.resize(obstacles.size());
  for (std::size_t i=0; i<obstacles.size(); ++i)
  {
    msg.obstacles_msg.header = msg.header;

    // copy polygon
    msg.obstacles_msg.obstacles[i].header = msg.header;
    obstacles[i]->toPolygonMsg(msg.obstacles_msg.obstacles[i].polygon);

    // copy id
    msg.obstacles_msg.obstacles[i].id = i; // TODO: we do not have any id stored yet

    // orientation
    //msg.obstacles_msg.obstacles[i].orientation =; // TODO

    // copy velocities
    obstacles[i]->toTwistWithCovarianceMsg(msg.obstacles_msg.obstacles[i].velocities);
  }
  
  feedback_pub_->publish(msg);
}

void TebVisualization::publishFeedbackMessage(const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles)
{
  teb_msgs::msg::FeedbackMsg msg;
  msg.header.stamp = nh_->now();
  msg.header.frame_id = cfg_->map_frame;
  msg.selected_trajectory_idx = 0;
  
  msg.trajectories.resize(1);
  msg.trajectories.front().header = msg.header;
  teb_planner.getFullTrajectory(msg.trajectories.front().trajectory);
 
  // add obstacles
  msg.obstacles_msg.obstacles.resize(obstacles.size());
  for (std::size_t i=0; i<obstacles.size(); ++i)
  {
    msg.obstacles_msg.header = msg.header;

    // copy polygon
    msg.obstacles_msg.obstacles[i].header = msg.header;
    obstacles[i]->toPolygonMsg(msg.obstacles_msg.obstacles[i].polygon);

    // copy id
    msg.obstacles_msg.obstacles[i].id = i; // TODO: we do not have any id stored yet

    // orientation
    //msg.obstacles_msg.obstacles[i].orientation =; // TODO

    // copy velocities
    obstacles[i]->toTwistWithCovarianceMsg(msg.obstacles_msg.obstacles[i].velocities);
  }
  
  feedback_pub_->publish(msg);
}

std_msgs::msg::ColorRGBA TebVisualization::toColorMsg(double a, double r, double g, double b)
{
  std_msgs::msg::ColorRGBA color;
  color.a = a;
  color.r = r;
  color.g = g;
  color.b = b;
  return color;
}

bool TebVisualization::printErrorWhenNotInitialized() const
{
  if (!initialized_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "TebVisualization class not initialized. You must call initialize or an appropriate constructor");
    return true;
  }
  return false;
}

nav2_util::CallbackReturn TebVisualization::on_configure()
{
  // register topics
  global_plan_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("global_plan", 1);;
  local_plan_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("local_plan",1);
  teb_poses_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseArray>("teb_poses", 1);
  teb_marker_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("teb_markers", 1);
  feedback_pub_ = nh_->create_publisher<teb_msgs::msg::FeedbackMsg>("teb_feedback", 1);

  initialized_ = true;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
TebVisualization::on_activate()
{
  global_plan_pub_->on_activate();
  local_plan_pub_->on_activate();
  teb_poses_pub_->on_activate();
  teb_marker_pub_->on_activate();
  feedback_pub_->on_activate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
TebVisualization::on_deactivate()
{
  global_plan_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
  teb_poses_pub_->on_deactivate();
  teb_marker_pub_->on_deactivate();
  feedback_pub_->on_deactivate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn 
TebVisualization::on_cleanup()
{
  global_plan_pub_.reset();
  local_plan_pub_.reset();
  teb_poses_pub_.reset();
  teb_marker_pub_.reset();
  feedback_pub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

} // namespace teb_local_planner
