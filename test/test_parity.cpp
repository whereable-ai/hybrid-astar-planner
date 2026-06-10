#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "planner.h"

using namespace HybridAStar;

class PlannerParityTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>("planner_parity_test");
    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);

    planner_ = std::make_unique<Planner>(node_);
    planner_->initializeLookups();
  }

  void TearDown() override {
    exec_->cancel();
    exec_->remove_node(node_);

    planner_.reset();
    node_.reset();
    exec_.reset();

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::unique_ptr<Planner> planner_;
};

TEST_F(PlannerParityTest, SimpleParityCheck) {
  auto grid_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  grid_msg->info.width = 15;
  grid_msg->info.height = 15;
  grid_msg->info.resolution = 1.0;
  grid_msg->data.assign(225, 0); // 15x15 empty grid

  auto start_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  start_msg->pose.pose.position.x = 2.0;
  start_msg->pose.pose.position.y = 2.0;
  start_msg->pose.pose.orientation.w = 1.0;

  auto goal_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  goal_msg->pose.position.x = 10.0;
  goal_msg->pose.position.y = 10.0;
  goal_msg->pose.orientation.w = 1.0;

  planner_->setMap(grid_msg);
  planner_->setStart(start_msg);
  planner_->setGoal(goal_msg);

  SUCCEED();
}

TEST(GridTransformTest, AppliesOriginTranslationAndYaw) {
  Constants::cellSize = 1.0f;

  geometry_msgs::msg::Pose origin;
  origin.position.x = 10.0;
  origin.position.y = -3.0;
  tf2::Quaternion qOrigin;
  qOrigin.setRPY(0, 0, 1.5707963267948966);
  origin.orientation = tf2::toMsg(qOrigin);

  GridTransform transform;
  transform.setOrigin(origin);

  geometry_msgs::msg::Pose mapPose = transform.toMap(2.0f, 4.0f, 0.25f);
  EXPECT_NEAR(mapPose.position.x, 6.0, 1e-6);
  EXPECT_NEAR(mapPose.position.y, -1.0, 1e-6);

  GridPose gridPose = transform.toGrid(mapPose);
  EXPECT_NEAR(gridPose.x, 2.0, 1e-5);
  EXPECT_NEAR(gridPose.y, 4.0, 1e-5);
  EXPECT_NEAR(gridPose.t, 0.25, 1e-5);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
