#include "teb_local_planner/homotopy_class_planner.h"
#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <gtest/gtest.h>

class HomotopyClassPlannerTest : public teb_local_planner::HomotopyClassPlanner {
    public:
    void SetUp(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
        teb_local_planner::RobotFootprintModelPtr robot_model;
        teb_local_planner::TebVisualizationPtr visualization;
        teb_local_planner::HomotopyClassPlannerPtr homotopy_class_planner;

        robot_model.reset(new teb_local_planner::CircularRobotFootprint(0.25));
        
        obstacles.push_back(
            teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(1, 2))
        );

        obstacles.push_back(
            teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(2, 3))
        );

        obstacles.push_back(
            teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(2, 4))
        );

        visualization.reset(
            new teb_local_planner::TebVisualization(node, cfg)
        );

        visualization->on_configure();
        visualization->on_activate();

        cfg.hcp.visualize_hc_graph = true;

        initialize(node, cfg, &obstacles, robot_model, visualization);
    }
    teb_local_planner::ObstContainer obstacles;
    teb_local_planner::TebConfig cfg;
};

class VisualizationSubscriber : public rclcpp::Node {
    public:
    VisualizationSubscriber() : rclcpp::Node("visualization_subscriber") {
      teb_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("teb_poses", 1, std::bind(&VisualizationSubscriber::tebPosesCallback, this, std::placeholders::_1));
    }

    geometry_msgs::msg::PoseArray teb_poses;
    bool teb_poses_fetched;
    void tebPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr input_poses) {
      if (input_poses) teb_poses = *input_poses;
      if (teb_poses.poses.size() > 0) {
          teb_poses_fetched = true;
      }
    }

    private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr teb_poses_sub_;
};

bool isPoseEqual(geometry_msgs::msg::Pose poseA, geometry_msgs::msg::Pose poseB, double xy_goal_tolerance = 0.2) {
    double dx = abs(poseA.position.x - poseB.position.x);
    double dy = abs(poseA.position.y - poseB.position.y);

    if (sqrt(dx*dx+dy*dy) <= xy_goal_tolerance) return true;

    return false;
}

TEST(HomotopyPlannerTest, goToGoal) {
    HomotopyClassPlannerTest test;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node(new rclcpp_lifecycle::LifecycleNode("test"));
    std::shared_ptr<VisualizationSubscriber> visualize_sub_node = std::make_shared<VisualizationSubscriber>();
    test.SetUp(node);

    using namespace teb_local_planner;
    PoseSE2 start(0, 0, 0);
    PoseSE2 via(2, 0, 0);
    PoseSE2 goal(5, 5, 0);
    
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.angular.z = 0;

    std::vector<geometry_msgs::msg::PoseStamped> initial_plan;
    
    geometry_msgs::msg::PoseStamped start_pose;
    geometry_msgs::msg::PoseStamped via_pose;
    geometry_msgs::msg::PoseStamped goal_pose;

    start.toPoseMsg(start_pose.pose);
    via.toPoseMsg(via_pose.pose);
    goal.toPoseMsg(goal_pose.pose);
    
    initial_plan.push_back(start_pose);
    initial_plan.push_back(via_pose);
    initial_plan.push_back(goal_pose);

    test.plan(initial_plan, &twist, false);
    //homotopy_class_planner_->exploreEquivalenceClassesAndInitTebs(start, goal, 0.3, &twist);
    
    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(node->get_node_base_interface());
    exe.add_node(visualize_sub_node);
    bool goal_reached = false;

    while(!goal_reached) {
        test.visualize();
        exe.spin_some();
        if (visualize_sub_node->teb_poses_fetched) {
            goal_reached = isPoseEqual(visualize_sub_node->teb_poses.poses.back(), goal_pose.pose);
        }
    }

    ASSERT_TRUE(visualize_sub_node->teb_poses_fetched);
    ASSERT_TRUE(goal_reached);

    exe.cancel();        
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}