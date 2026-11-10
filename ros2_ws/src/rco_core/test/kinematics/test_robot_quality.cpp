#include "rco_core/kinematics/quality.hpp"
#include "rco_core/kinematics/robot_quality.hpp"
#include "rco_core/robot_model_context.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/node.hpp>
#include <stdexcept>
#include <vector>

namespace {
rco_core::RobotModelContext qualityRobot() {
  const auto node = std::make_shared<rclcpp::Node>(
      "rco_quality_test",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  return {node, "ur_manipulator", "tool0"};
}

std::vector<double> qualityReferenceJoints() {
  return {0.2, -1.0, 1.2, -1.4, -0.8, 0.3};
}

TEST(RobotQualityTest, NormalUr5eStateHasFullRankAndPreservesInput) {
  const auto robot = qualityRobot();
  auto state = robot.makeDefaultState();
  const auto joints = qualityReferenceJoints();
  state.setJointGroupPositions(robot.jointModelGroup(), joints);
  const auto quality = rco_core::kinematics::analyzeRobotState(robot, state);
  EXPECT_FALSE(quality.near_singular);
  EXPECT_EQ(quality.numerical_rank, 6U);
  EXPECT_GT(quality.minimum_singular_value, 1e-6);
  EXPECT_GT(quality.manipulability, 0.0);
  std::vector<double> after;
  state.copyJointGroupPositions(robot.jointModelGroup(), after);
  EXPECT_EQ(after, joints);
}

TEST(RobotQualityTest, DetectsWristSingularityAndPathWorstSample) {
  const auto robot = qualityRobot();
  const auto normal = qualityReferenceJoints();
  auto singular = normal;
  singular.at(4) = 0.0;
  auto state = robot.makeDefaultState();
  state.setJointGroupPositions(robot.jointModelGroup(), singular);
  const auto quality = rco_core::kinematics::analyzeRobotState(robot, state);
  EXPECT_TRUE(quality.near_singular);
  EXPECT_LT(quality.minimum_singular_value, 1e-8);
  EXPECT_FALSE(quality.condition_number.has_value());
  const auto path = rco_core::kinematics::analyzeJointPath(robot, {normal, singular});
  EXPECT_EQ(path.sample_count, 2U);
  EXPECT_EQ(path.near_singular_count, 1U);
  ASSERT_TRUE(path.minimum_singular_value.has_value());
  EXPECT_LT(path.minimum_singular_value.value().minimum, 1e-8);
}

TEST(RobotQualityTest, RejectsInvalidStateAndJointVectors) {
  const auto robot = qualityRobot();
  auto state = robot.makeDefaultState();
  state.setVariablePosition("shoulder_pan_joint", std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(static_cast<void>(rco_core::kinematics::tcpJacobian(robot, state)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(rco_core::kinematics::analyzeJointPath(robot, {{0.0}})),
               std::invalid_argument);
  auto outside = qualityReferenceJoints();
  outside.front() = 100.0;
  EXPECT_THROW(static_cast<void>(rco_core::kinematics::analyzeJointPath(robot, {outside})),
               std::invalid_argument);
}

TEST(RobotQualityTest, RejectsStateFromDifferentRobotModel) {
  const auto robot = qualityRobot();
  const auto other = qualityRobot();
  EXPECT_THROW(static_cast<void>(rco_core::kinematics::tcpJacobian(robot, other.makeDefaultState())),
               std::invalid_argument);
}
} // namespace
