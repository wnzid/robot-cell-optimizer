#include "rco_core/domain/model.hpp"
#include "rco_core/reachability/ik.hpp"
#include "rco_core/reachability/result.hpp"
#include "rco_core/reachability/seed.hpp"
#include "rco_core/robot_model_context.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/node.hpp>
#include <stdexcept>
#include <vector>

namespace {

using rco_core::domain::TargetPose;
using rco_core::reachability::FailureReason;
using rco_core::reachability::SampleResult;
using rco_core::reachability::solveSeededIk;

rco_core::RobotModelContext makeRobot(const char* tcp = "tool0") {
  const auto node = std::make_shared<rclcpp::Node>(
      "rco_seeded_ik_test",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  return {node, "ur_manipulator", tcp};
}

std::vector<double> referenceJoints() {
  return {0.2, -1.0, 1.2, -1.4, -0.8, 0.3};
}

TargetPose targetFromJoints(const rco_core::RobotModelContext& robot,
                            const std::vector<double>& joints) {
  auto state = robot.makeDefaultState();
  state.setJointGroupPositions(robot.jointModelGroup(), joints);
  state.update();
  const auto& transform = state.getGlobalLinkTransform(robot.tcpLink());
  const Eigen::Quaterniond rotation(transform.rotation());
  TargetPose target;
  target.pose.frame_id = robot.modelFrame();
  target.pose.position = {transform.translation().x(), transform.translation().y(),
                          transform.translation().z()};
  target.pose.orientation = {rotation.x(), rotation.y(), rotation.z(), rotation.w()};
  return target;
}

void expectFailure(const SampleResult& result, FailureReason reason) {
  EXPECT_FALSE(result.reachable);
  EXPECT_EQ(result.failure_reason, reason);
  EXPECT_TRUE(result.joint_positions_rad.empty());
  EXPECT_TRUE(rco_core::reachability::validate(result).empty());
}

void expectPoseMatches(const TargetPose& actual, const TargetPose& target) {
  const Eigen::Vector3d actual_position(actual.pose.position.x, actual.pose.position.y,
                                        actual.pose.position.z);
  const Eigen::Vector3d target_position(target.pose.position.x, target.pose.position.y,
                                        target.pose.position.z);
  const auto& lhs = actual.pose.orientation;
  const auto& rhs = target.pose.orientation;
  EXPECT_LE((actual_position - target_position).norm(), 1e-4);
  EXPECT_LE(Eigen::Quaterniond(lhs.w, lhs.x, lhs.y, lhs.z)
                .angularDistance(Eigen::Quaterniond(rhs.w, rhs.x, rhs.y, rhs.z)),
            1e-4);
}

TEST(SeededIkTest, SolvesNearbyTargetAndVerifiesForwardKinematics) {
  const auto robot = makeRobot();
  const auto target = targetFromJoints(robot, referenceJoints());
  auto seed = referenceJoints();
  seed.front() += 0.02;

  const auto result = solveSeededIk(robot, target, seed, 7U);

  ASSERT_TRUE(result.reachable);
  EXPECT_EQ(result.sample_index, 7U);
  EXPECT_TRUE(rco_core::reachability::validate(result).empty());
  expectPoseMatches(targetFromJoints(robot, result.joint_positions_rad), target);
}

TEST(SeededIkTest, RepeatsIdenticallyForSameSeedAndTarget) {
  const auto robot = makeRobot();
  const auto target = targetFromJoints(robot, referenceJoints());
  auto seed = referenceJoints();
  seed.back() += 0.02;
  const auto first = solveSeededIk(robot, target, seed, 2U);

  ASSERT_TRUE(first.reachable);
  for (int repeat = 0; repeat < 5; ++repeat) {
    EXPECT_EQ(solveSeededIk(robot, target, seed, 2U), first);
  }
}

TEST(SeededIkTest, ReportsUnreachableTargetWithoutChangingContinuitySeed) {
  const auto robot = makeRobot();
  auto target = targetFromJoints(robot, referenceJoints());
  target.pose.position.x = 100.0;
  rco_core::reachability::ContinuitySeed seed(referenceJoints());

  const auto result = solveSeededIk(robot, target, seed.current(), 1U);
  seed.observe(result);

  expectFailure(result, FailureReason::kIkFailed);
  EXPECT_EQ(seed.current(), referenceJoints());
}

TEST(SeededIkTest, RejectsUnknownTargetFrame) {
  const auto robot = makeRobot();
  auto target = targetFromJoints(robot, referenceJoints());
  target.pose.frame_id = "unresolved_workpiece";

  expectFailure(solveSeededIk(robot, target, referenceJoints(), 0U), FailureReason::kInvalidTarget);
}

TEST(SeededIkTest, RejectsNonNormalizedOrientation) {
  const auto robot = makeRobot();
  auto target = targetFromJoints(robot, referenceJoints());
  target.pose.orientation = {0.0, 0.0, 0.0, 2.0};

  expectFailure(solveSeededIk(robot, target, referenceJoints(), 0U), FailureReason::kInvalidTarget);
}

TEST(SeededIkTest, RejectsNonFiniteTarget) {
  const auto robot = makeRobot();
  auto target = targetFromJoints(robot, referenceJoints());
  target.pose.position.x = std::numeric_limits<double>::quiet_NaN();

  expectFailure(solveSeededIk(robot, target, referenceJoints(), 0U), FailureReason::kInvalidTarget);
}

void expectInvalidSeed(const rco_core::RobotModelContext& robot, const std::vector<double>& seed) {
  const auto target = targetFromJoints(robot, referenceJoints());
  EXPECT_THROW(static_cast<void>(solveSeededIk(robot, target, seed, 0U)), std::invalid_argument);
}

TEST(SeededIkTest, RejectsWrongSizedAndEmptySeeds) {
  const auto robot = makeRobot();
  expectInvalidSeed(robot, {});
  expectInvalidSeed(robot, {0.0, 1.0});
}

TEST(SeededIkTest, RejectsNonFiniteAndOutOfBoundsSeeds) {
  const auto robot = makeRobot();
  auto seed = referenceJoints();
  seed.front() = std::numeric_limits<double>::infinity();
  expectInvalidSeed(robot, seed);
  seed.front() = 100.0;
  expectInvalidSeed(robot, seed);
}

TEST(SeededIkTest, RejectsTcpThatDoesNotMatchSolverTip) {
  const auto robot = makeRobot("wrist_3_link");
  expectInvalidSeed(robot, referenceJoints());
}

} // namespace
