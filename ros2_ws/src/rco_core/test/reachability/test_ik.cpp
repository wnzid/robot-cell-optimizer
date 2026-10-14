#include "rco_core/domain/model.hpp"
#include "rco_core/reachability/ik.hpp"
#include "rco_core/reachability/path.hpp"
#include "rco_core/reachability/result.hpp"
#include "rco_core/reachability/seed.hpp"
#include "rco_core/robot_model_context.hpp"
#include "rco_core/scene/collision_object.hpp"
#include "rco_core/scene/planning_scene.hpp"

#include <Eigen/Geometry>
#include <cstddef>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
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

TEST(SeededPathTest, EvaluatesOrderedTargetsAndRepeatsIdentically) {
  const auto robot = makeRobot();
  auto joints = referenceJoints();
  const auto first = targetFromJoints(robot, joints);
  joints.front() += 0.04;
  const std::vector<TargetPose> targets{first, targetFromJoints(robot, joints), first};

  const auto results =
      rco_core::reachability::solveSeededPath(robot, targets, referenceJoints(), 0.2);

  ASSERT_EQ(results.size(), targets.size());
  for (std::size_t index = 0; index < results.size(); ++index) {
    ASSERT_TRUE(results[index].reachable);
    EXPECT_EQ(results[index].sample_index, index);
    expectPoseMatches(targetFromJoints(robot, results[index].joint_positions_rad), targets[index]);
  }
  EXPECT_EQ(rco_core::reachability::solveSeededPath(robot, targets, referenceJoints(), 0.2),
            results);
}

TEST(SeededPathTest, RetainsAcceptedSeedAcrossInvalidAndUnreachableSamples) {
  const auto robot = makeRobot();
  const auto valid = targetFromJoints(robot, referenceJoints());
  auto invalid = valid;
  invalid.pose.frame_id = "unresolved_workpiece";
  auto unreachable = valid;
  unreachable.pose.position.x = 100.0;

  const auto results = rco_core::reachability::solveSeededPath(
      robot, {valid, invalid, unreachable, valid}, referenceJoints(), 0.2);

  ASSERT_EQ(results.size(), 4U);
  ASSERT_TRUE(results.front().reachable);
  expectFailure(results[1], FailureReason::kInvalidTarget);
  expectFailure(results[2], FailureReason::kIkFailed);
  EXPECT_EQ(results.back(), solveSeededIk(robot, valid, results.front().joint_positions_rad, 3U));
}

TEST(SeededPathTest, RejectsJointJumpAndRecoversFromLastAcceptedSeed) {
  const auto robot = makeRobot();
  auto joints = referenceJoints();
  const auto first = targetFromJoints(robot, joints);
  joints.front() += 0.1;
  const auto jumped = targetFromJoints(robot, joints);

  const auto results = rco_core::reachability::solveSeededPath(robot, {first, jumped, first},
                                                               referenceJoints(), 0.001);

  ASSERT_EQ(results.size(), 3U);
  ASSERT_TRUE(results.front().reachable);
  EXPECT_FALSE(results[1].reachable);
  EXPECT_EQ(results[1].failure_reason, FailureReason::kDiscontinuity);
  EXPECT_FALSE(results[1].joint_positions_rad.empty());
  EXPECT_EQ(results.back(), solveSeededIk(robot, first, results.front().joint_positions_rad, 2U));
}

void expectInvalidPath(const rco_core::RobotModelContext& robot,
                       const std::vector<TargetPose>& targets, double limit) {
  EXPECT_THROW(static_cast<void>(rco_core::reachability::solveSeededPath(robot, targets,
                                                                         referenceJoints(), limit)),
               std::invalid_argument);
}

TEST(SeededPathTest, RejectsEmptyPathAndInvalidStepLimits) {
  const auto robot = makeRobot();
  const std::vector<TargetPose> targets{targetFromJoints(robot, referenceJoints())};
  expectInvalidPath(robot, {}, 0.2);
  expectInvalidPath(robot, targets, 0.0);
  expectInvalidPath(robot, targets, -0.1);
  expectInvalidPath(robot, targets, std::numeric_limits<double>::infinity());
  expectInvalidPath(robot, targets, std::numeric_limits<double>::quiet_NaN());
}

rco_core::domain::CellEntity obstacleAt(const TargetPose& target, double size) {
  rco_core::domain::CellEntity obstacle;
  obstacle.id = "test_obstacle";
  obstacle.pose = target.pose;
  obstacle.geometry.type = rco_core::domain::GeometryType::kBox;
  obstacle.geometry.size_m = {size, size, size};
  return obstacle;
}

void expectCollision(const SampleResult& result) {
  EXPECT_FALSE(result.reachable);
  EXPECT_EQ(result.failure_reason, FailureReason::kCollision);
  EXPECT_FALSE(result.joint_positions_rad.empty());
  EXPECT_TRUE(rco_core::reachability::validate(result).empty());
}

TEST(SceneReachabilityTest, AcceptsClearSceneAndPreservesSceneSnapshot) {
  const auto robot = makeRobot();
  const planning_scene::PlanningScene scene(robot.robotModel());
  const auto target = targetFromJoints(robot, referenceJoints());
  moveit_msgs::msg::PlanningScene before;
  scene.getPlanningSceneMsg(before);

  const auto results = rco_core::reachability::solveSeededPath(robot, scene, {target, target},
                                                               referenceJoints(), 0.2);

  ASSERT_EQ(results.size(), 2U);
  EXPECT_TRUE(results.front().reachable);
  EXPECT_TRUE(results.back().reachable);
  moveit_msgs::msg::PlanningScene after;
  scene.getPlanningSceneMsg(after);
  EXPECT_EQ(after, before);
}

TEST(SceneReachabilityTest, RejectsWorldCollisionAndHonorsAllowedCollisionMatrix) {
  const auto robot = makeRobot();
  planning_scene::PlanningScene scene(robot.robotModel());
  const auto target = targetFromJoints(robot, referenceJoints());
  ASSERT_TRUE(scene.processCollisionObjectMsg(
      rco_core::scene::makeCollisionObject(obstacleAt(target, 0.3))));

  const auto blocked =
      rco_core::reachability::solveSeededPath(robot, scene, {target}, referenceJoints(), 0.2);
  expectCollision(blocked.front());

  scene.getAllowedCollisionMatrixNonConst().setEntry("test_obstacle", true);
  const auto allowed =
      rco_core::reachability::solveSeededPath(robot, scene, {target}, referenceJoints(), 0.2);
  EXPECT_TRUE(allowed.front().reachable);
}

TEST(SceneReachabilityTest, RejectsCollidingAttachedTool) {
  const auto robot = makeRobot();
  planning_scene::PlanningScene scene(robot.robotModel());
  const auto target = targetFromJoints(robot, referenceJoints());
  auto obstacle = obstacleAt(target, 0.02);
  obstacle.pose.position.z += 0.23;
  ASSERT_TRUE(scene.processCollisionObjectMsg(rco_core::scene::makeCollisionObject(obstacle)));
  const auto without_tool =
      rco_core::reachability::solveSeededPath(robot, scene, {target}, referenceJoints(), 0.2);
  ASSERT_TRUE(without_tool.front().reachable);

  rco_core::domain::ToolDefinition tool;
  tool.id = "test_tool";
  tool.name = "large_test_tool";
  tool.tcp.frame_id = "tool0";
  rco_core::domain::GeometryDefinition sphere;
  sphere.type = rco_core::domain::GeometryType::kSphere;
  sphere.radius_m = 0.25;
  tool.geometry.push_back(sphere);
  ASSERT_TRUE(
      rco_core::scene::applyAttachedTool(scene, tool, robot.robotModel()->getLinkModelNames()));

  const auto with_tool =
      rco_core::reachability::solveSeededPath(robot, scene, {target}, referenceJoints(), 0.2);
  expectCollision(with_tool.front());
}

TEST(SceneReachabilityTest, RejectsSceneWithDifferentRobotModelInstance) {
  const auto robot = makeRobot();
  const auto other_robot = makeRobot();
  const planning_scene::PlanningScene other_scene(other_robot.robotModel());
  const auto target = targetFromJoints(robot, referenceJoints());
  EXPECT_THROW(static_cast<void>(rco_core::reachability::solveSeededPath(
                   robot, other_scene, {target}, referenceJoints(), 0.2)),
               std::invalid_argument);
}

TEST(SceneReachabilityTest, RejectsWrongSizedSeedBeforeSettingRobotState) {
  const auto robot = makeRobot();
  const planning_scene::PlanningScene scene(robot.robotModel());
  const auto target = targetFromJoints(robot, referenceJoints());
  EXPECT_THROW(static_cast<void>(
                   rco_core::reachability::solveSeededPath(robot, scene, {target}, {0.0}, 0.2)),
               std::invalid_argument);
}

TEST(SceneReachabilityTest, RetainsLastAcceptedSeedAcrossWorldCollision) {
  const auto robot = makeRobot();
  planning_scene::PlanningScene scene(robot.robotModel());
  auto joints = referenceJoints();
  const auto clear_target = targetFromJoints(robot, joints);
  joints.front() += 0.7;
  const auto blocked_target = targetFromJoints(robot, joints);
  ASSERT_TRUE(scene.processCollisionObjectMsg(
      rco_core::scene::makeCollisionObject(obstacleAt(blocked_target, 0.08))));

  const auto results = rco_core::reachability::solveSeededPath(
      robot, scene, {clear_target, blocked_target, clear_target}, referenceJoints(), 1.0);

  ASSERT_EQ(results.size(), 3U);
  ASSERT_TRUE(results.front().reachable);
  expectCollision(results[1]);
  EXPECT_EQ(results.back(),
            solveSeededIk(robot, clear_target, results.front().joint_positions_rad, 2U));
}

TEST(SceneReachabilityTest, RejectsAttachedToolSelfCollisionWithoutWorldObstacles) {
  const auto robot = makeRobot();
  planning_scene::PlanningScene scene(robot.robotModel());
  rco_core::domain::ToolDefinition tool;
  tool.id = "oversized_tool";
  tool.name = "self_collision_fixture";
  tool.tcp.frame_id = "tool0";
  rco_core::domain::GeometryDefinition sphere;
  sphere.type = rco_core::domain::GeometryType::kSphere;
  sphere.radius_m = 1.0;
  tool.geometry.push_back(sphere);
  ASSERT_TRUE(rco_core::scene::applyAttachedTool(scene, tool, {"tool0"}));
  const auto target = targetFromJoints(robot, referenceJoints());

  const auto results =
      rco_core::reachability::solveSeededPath(robot, scene, {target}, referenceJoints(), 0.2);

  expectCollision(results.front());
}

TEST(SeededIkStateTest, RejectsStateFromDifferentRobotModel) {
  const auto robot = makeRobot();
  const auto other_robot = makeRobot();
  const auto target = targetFromJoints(robot, referenceJoints());
  const auto other_state = other_robot.makeDefaultState();
  EXPECT_THROW(static_cast<void>(solveSeededIk(robot, target, other_state, 0U)),
               std::invalid_argument);
}

TEST(SeededIkStateTest, RejectsNonFiniteContextState) {
  const auto robot = makeRobot();
  const auto target = targetFromJoints(robot, referenceJoints());
  auto state = robot.makeDefaultState();
  state.setVariablePosition("shoulder_pan_joint", std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(static_cast<void>(solveSeededIk(robot, target, state, 0U)), std::invalid_argument);
}

} // namespace
