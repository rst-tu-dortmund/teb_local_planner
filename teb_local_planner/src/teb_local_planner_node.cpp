#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <nav2_tasks/follow_path_task.hpp>
#include <nav_2d_utils/conversions.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "teb_local_planner/teb_local_planner_ros.h"

#include <string>

nav2_tasks::TaskStatus followPath(const nav2_tasks::FollowPathCommand::SharedPtr path) {

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  auto node = std::make_shared<rclcpp::Node>("teb_local_planner_node");
  auto tf2_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf2_listner = tf2_ros::TransformListener(*tf2_buffer.get(), true);
  auto costmap2d_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("teb_local_planner_costmap2d", *tf2_buffer.get());
  auto teb_local_planner = std::make_shared<teb_local_planner::TebLocalPlannerROS>();
  teb_local_planner->initialize(node, tf2_buffer, costmap2d_ros);

  std::unique_ptr<nav2_tasks::FollowPathTaskServer> task_server_ = std::make_unique<nav2_tasks::FollowPathTaskServer>(node);

  auto callback_func = [&](const nav2_tasks::FollowPathCommand::SharedPtr path) {
      nav2_tasks::TaskStatus status;

      std::vector<geometry_msgs::msg::PoseStamped> pose_stamped_vec;

      std::stringstream ss;
      ss << "Recived plan : ";

      for(auto &pose : path->poses) {
          geometry_msgs::msg::PoseStamped pose_stamped;
          pose_stamped.pose = pose;
          pose_stamped.header = path->header;
          pose_stamped_vec.push_back(pose_stamped);
          ss << "(" << pose.position.x << ", " << pose.position.y << ") ";
      }

      RCLCPP_INFO(node->get_logger(), ss.str().c_str());

      teb_local_planner->setPlan(pose_stamped_vec);

      geometry_msgs::msg::Twist twist;

      teb_local_planner->computeVelocityCommands(twist);
      return status;
    };

  task_server_->setExecuteCallback(callback_func);

  exec.add_node(costmap2d_ros);
  exec.add_node(node);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
