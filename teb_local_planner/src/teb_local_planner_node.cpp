#include <dwb_core/exceptions.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <motion_msg/msg/motion_wheel.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_tasks/follow_path_task.hpp>
#include <nav_2d_utils/conversions.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "teb_local_planner/teb_local_planner_ros.h"

#include <string>

std::vector<geometry_msgs::msg::PoseStamped> getPoseStampedVec(
        const nav2_tasks::FollowPathCommand::SharedPtr path) {

    std::vector<geometry_msgs::msg::PoseStamped> pose_stamped_vec;

    for(auto &pose : path->poses) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose = pose;
        pose_stamped.header = path->header;
        pose_stamped_vec.push_back(pose_stamped);
    }

    return pose_stamped_vec;
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

    const std::string cmd_topic = "cmd_vel";
    const auto wheel_pub =
          node->create_publisher<geometry_msgs::msg::Twist>(cmd_topic);

    auto publishVelocity = [&wheel_pub](const geometry_msgs::msg::Twist &twist) {
        wheel_pub->publish(twist);
    };

    auto publishZeroVelocity = [&wheel_pub]() {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = 0;
        wheel_pub->publish(twist);
    };

    std::unique_ptr<nav2_tasks::FollowPathTaskServer> task_server_ = std::make_unique<nav2_tasks::FollowPathTaskServer>(node);

    auto callback_func = [&](nav2_tasks::FollowPathCommand::SharedPtr path) {
        RCLCPP_INFO(node->get_logger(), "Starting controller");
        try {
            const auto pose_stamped_vec = getPoseStampedVec(path);
            teb_local_planner->setPlan(pose_stamped_vec);
            RCLCPP_INFO(node->get_logger(), "Initialized");

            rclcpp::Rate loop_rate(10);
            while (rclcpp::ok()) {
                geometry_msgs::msg::PoseStamped pose2d;

                if (!costmap2d_ros->getRobotPose(pose2d)) {
                    RCLCPP_INFO(node->get_logger(), "No pose. Stopping robot");
                    publishZeroVelocity();
                } else {
                    if (teb_local_planner->isGoalReached()) {
                        break;
                    }
                    geometry_msgs::msg::Twist twist;

                    const bool is_success = teb_local_planner->computeVelocityCommands(twist);

                    if(is_success)
                        publishVelocity(twist);

                    RCLCPP_INFO(node->get_logger(), "Publishing velocity at time %.2f", node->now().seconds());

                    // Check if this task has been canceled
                    if (task_server_->cancelRequested()) {
                        RCLCPP_INFO(node->get_logger(), "execute: task has been canceled");
                        task_server_->setCanceled();
                        publishZeroVelocity();
                        return nav2_tasks::TaskStatus::CANCELED;
                    }

                    // Check if there is an update to the path to follow
                    if (task_server_->updateRequested()) {
                        // Get the new, updated path
                        auto path_cmd = std::make_shared<nav2_tasks::FollowPathCommand>();
                        task_server_->getCommandUpdate(path_cmd);
                        task_server_->setUpdated();

                        // and pass it to the local planner
                        const auto pose_stamped_vec = getPoseStampedVec(path_cmd);
                        teb_local_planner->setPlan(pose_stamped_vec);
                    }
                }
                loop_rate.sleep();
            }
        } catch (nav_core2::PlannerException & e) {
            RCLCPP_INFO(node->get_logger(), e.what());
            publishZeroVelocity();
            return nav2_tasks::TaskStatus::FAILED;
        }

        nav2_tasks::FollowPathResult result;
        task_server_->setResult(result);
        publishZeroVelocity();

        return nav2_tasks::TaskStatus::SUCCEEDED;
    };

    task_server_->setExecuteCallback(callback_func);

    exec.add_node(costmap2d_ros);
    exec.add_node(node);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
