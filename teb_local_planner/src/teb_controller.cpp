// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "teb_local_planner/teb_controller.hpp"

#include <chrono>
#include <memory>
#include <string>

#include <dwb_core/exceptions.hpp>
#include <nav_2d_utils/conversions.hpp>
#include <nav2_util/node_utils.hpp>


using namespace std::chrono_literals;

namespace teb_local_planner
{

TebController::TebController()
: LifecycleNode("teb_controller", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");

  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
  costmap_thread_ = std::make_unique<std::thread>(
    [](rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    {rclcpp::spin(node->get_node_base_interface());}, costmap_ros_);
}

TebController::~TebController()
{
  RCLCPP_INFO(get_logger(), "Destroying");
  costmap_thread_->join();
}

nav2_util::CallbackReturn
TebController::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  costmap_ros_->on_configure(state);

  auto node = shared_from_this();
  
  progress_checker_ = std::make_unique<dwb_controller::ProgressChecker>(rclcpp_node_);

  planner_ = std::make_unique<TebLocalPlannerROS>(node, costmap_ros_->getTfBuffer(), costmap_ros_);
  planner_->on_configure(state);

  odom_sub_ = std::make_shared<nav_2d_utils::OdomSubscriber>(*this);
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  // Create the action server that we implement with our followPath method
  action_server_ = std::make_unique<ActionServer>(rclcpp_node_, "FollowPath",
      std::bind(&TebController::followPath, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
TebController::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  planner_->on_activate(state);
  costmap_ros_->on_activate(state);
  vel_pub_->on_activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
TebController::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  planner_->on_deactivate(state);
  costmap_ros_->on_deactivate(state);
  vel_pub_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
TebController::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  planner_->on_cleanup(state);
  costmap_ros_->on_cleanup(state);

  // Release any allocated resources
  planner_.reset();
  odom_sub_.reset();
  vel_pub_.reset();
  action_server_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
TebController::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
TebController::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
TebController::followPath()
{
  using namespace dwb_controller;

  try {
    setPlannerPath(action_server_->get_current_goal()->path);
    progress_checker_->reset();

    rclcpp::Rate loop_rate(100ms);
    while (rclcpp::ok()) {
      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Cancelling and stopping.");
        action_server_->cancel_all();
        publishZeroVelocity();
        return;
      }

      updateGlobalPath();

      computeAndPublishVelocity();

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      loop_rate.sleep();
    }
  } catch (nav_core2::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    publishZeroVelocity();
    action_server_->abort_all();
    return;
  }

  RCLCPP_DEBUG(get_logger(), "TEB succeeded, setting result");

  publishZeroVelocity();

  // TODO(orduno) #861 Handle a pending preemption.
  action_server_->succeeded_current();
}

void TebController::publishVelocity(const nav_2d_msgs::msg::Twist2DStamped & velocity)
{
  auto cmd_vel = nav_2d_utils::twist2Dto3D(velocity.velocity);
  vel_pub_->publish(cmd_vel);
}

void TebController::publishZeroVelocity()
{
  nav_2d_msgs::msg::Twist2DStamped velocity;
  velocity.velocity.x = 0;
  velocity.velocity.y = 0;
  velocity.velocity.theta = 0;

  publishVelocity(velocity);
}

bool TebController::isGoalReached()
{
  return planner_->isGoalReached();
}

bool TebController::getRobotPose(nav_2d_msgs::msg::Pose2DStamped & pose2d)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Could not get robot pose");
    return false;
  }
  pose2d = nav_2d_utils::poseStampedToPose2D(current_pose);
  return true;
}

std::vector<geometry_msgs::msg::PoseStamped> 
TebController::pathToPoseVec(const nav2_msgs::msg::Path & path) {
  std::vector<geometry_msgs::msg::PoseStamped> pose_stamped_vec;

  for(const auto &p : path.poses) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = path.header;
      pose_stamped.pose = p;
      pose_stamped_vec.push_back(pose_stamped);
  }

  return pose_stamped_vec;
}

void TebController::setPlannerPath(const nav2_msgs::msg::Path & path)
{
  auto path2d = pathToPoseVec(path);

  RCLCPP_DEBUG(get_logger(), "Providing path to the local planner");
  planner_->setPlan(path2d);

  auto end_pose = *(path.poses.end() - 1);

  RCLCPP_DEBUG(get_logger(), "Path end point is (%.2f, %.2f)",
    end_pose.position.x, end_pose.position.y);
}

void TebController::computeAndPublishVelocity()
{
  nav_2d_msgs::msg::Pose2DStamped pose2d;

  if (!getRobotPose(pose2d)) {
    throw nav_core2::PlannerException("Failed to obtain robot pose");
  }

  progress_checker_->check(pose2d);

  auto cmd_vel_2d = planner_->computeVelocityCommands(pose2d, odom_sub_->getTwist());

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  publishVelocity(cmd_vel_2d);
}

void TebController::updateGlobalPath()
{
  if (action_server_->preempt_requested()) {
    RCLCPP_INFO(get_logger(), "Preempting the goal. Passing the new path to the planner.");
    setPlannerPath(action_server_->accept_pending_goal()->path);
  }
}
}  // namespace dwb_controller
