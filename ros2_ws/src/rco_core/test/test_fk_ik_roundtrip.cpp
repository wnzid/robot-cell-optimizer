#include "rco_core/robot_model_context.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <random>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <vector>

namespace {

constexpr std::size_t kSampleCount = 50;
constexpr double kPositionTolerance = 1e-4;
constexpr double kOrientationTolerance = 1e-4;
constexpr double kIkTimeoutSeconds = 0.1;

struct RoundTripResult {
  bool source_within_bounds{false};
  bool ik_solved{false};
  double position_error{std::numeric_limits<double>::infinity()};
  double orientation_error{std::numeric_limits<double>::infinity()};
};

struct VerificationResult {
  bool success{true};
  std::string message;
};

std::vector<double> sampleJointValues(const rco_core::RobotModelContext& robot,
                                      const moveit::core::JointModelGroup& group,
                                      std::mt19937& rng) {
  std::vector<double> values;
  values.reserve(group.getVariableCount());

  for (const auto& variable_name : group.getVariableNames()) {
    const auto& bounds = robot.robotModel()->getVariableBounds(variable_name);

    std::uniform_real_distribution<double> distribution(bounds.min_position_, bounds.max_position_);

    values.push_back(distribution(rng));
  }

  return values;
}

double calculateOrientationError(const Eigen::Isometry3d& target_pose,
                                 const Eigen::Isometry3d& solved_pose) {
  const Eigen::Matrix3d rotation_error =
      target_pose.rotation().transpose() * solved_pose.rotation();

  return std::abs(Eigen::AngleAxisd(rotation_error).angle());
}

RoundTripResult runRoundTrip(const rco_core::RobotModelContext& robot,
                             const moveit::core::JointModelGroup& group, std::mt19937& rng) {
  auto source_state = robot.makeDefaultState();

  const auto joint_values = sampleJointValues(robot, group, rng);

  source_state.setJointGroupPositions(&group, joint_values);
  source_state.enforceBounds(&group);
  source_state.update();

  RoundTripResult result;
  result.source_within_bounds = source_state.satisfiesBounds(&group);

  const Eigen::Isometry3d target_pose = source_state.getGlobalLinkTransform(robot.tcpLink());

  auto solved_state = robot.makeDefaultState();

  result.ik_solved =
      solved_state.setFromIK(&group, target_pose, robot.tcpLink(), kIkTimeoutSeconds);

  if (!result.ik_solved) {
    return result;
  }

  solved_state.update();

  const Eigen::Isometry3d solved_pose = solved_state.getGlobalLinkTransform(robot.tcpLink());

  result.position_error = (target_pose.translation() - solved_pose.translation()).norm();

  result.orientation_error = calculateOrientationError(target_pose, solved_pose);

  return result;
}

bool allJointVariablesHaveBounds(const rco_core::RobotModelContext& robot,
                                 const moveit::core::JointModelGroup& group) {
  const auto& names = group.getVariableNames();

  return std::ranges::all_of(names, [&robot](const auto& variable_name) {
    return robot.robotModel()->getVariableBounds(variable_name).position_bounded_;
  });
}

VerificationResult verifyRoundTrips(const rco_core::RobotModelContext& robot,
                                    const moveit::core::JointModelGroup& group) {
  std::mt19937 rng(20260909U);

  for (std::size_t sample = 0; sample < kSampleCount; ++sample) {
    const auto result = runRoundTrip(robot, group, rng);

    if (!result.source_within_bounds) {
      return {
          false,
          "Source state outside bounds at sample " + std::to_string(sample),
      };
    }

    if (!result.ik_solved) {
      return {
          false,
          "IK failed at sample " + std::to_string(sample),
      };
    }

    if (result.position_error >= kPositionTolerance) {
      return {
          false,
          "Position tolerance exceeded at sample " + std::to_string(sample),
      };
    }

    if (result.orientation_error >= kOrientationTolerance) {
      return {
          false,
          "Orientation tolerance exceeded at sample " + std::to_string(sample),
      };
    }
  }

  return {};
}

// GoogleTest assertion macros expand into control flow internally.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
TEST(FkIkRoundTripTest, Ur5eRandomStatesRoundTrip) {
  const auto node = std::make_shared<rclcpp::Node>(
      "rco_fk_ik_roundtrip_test",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const rco_core::RobotModelContext robot{
      node,
      "ur_manipulator",
      "tool0",
  };

  const auto* group = robot.jointModelGroup();

  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->getVariableCount(), 6U);
  ASSERT_TRUE(allJointVariablesHaveBounds(robot, *group));

  const auto verification = verifyRoundTrips(robot, *group);

  EXPECT_TRUE(verification.success) << verification.message;
}

} // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
