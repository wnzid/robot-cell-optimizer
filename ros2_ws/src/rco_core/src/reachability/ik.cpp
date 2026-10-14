#include "rco_core/reachability/ik.hpp"

#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/domain/validation.hpp"
#include "rco_core/reachability/result.hpp"
#include "rco_core/robot_model_context.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <vector>

namespace rco_core::reachability {
namespace {

constexpr double kPositionToleranceM = 1e-4;
constexpr double kOrientationToleranceRad = 1e-4;

bool finitePositions(const std::vector<double>& positions) {
  return std::ranges::all_of(positions, [](double value) { return std::isfinite(value); });
}

void validateSolver(const RobotModelContext& robot, const kinematics::KinematicsBase& solver) {
  if (solver.getTipFrames().size() != 1U || solver.getTipFrames().front() != robot.tcpLink()) {
    throw std::invalid_argument("Single-tip IK solver must match the configured TCP link.");
  }
  if (solver.getBaseFrame() != robot.modelFrame() &&
      !robot.robotModel()->hasLinkModel(solver.getBaseFrame())) {
    throw std::invalid_argument("IK solver base frame must be a known robot link or model frame.");
  }
  auto indices = robot.jointModelGroup()->getKinematicsSolverJointBijection();
  std::ranges::sort(indices);
  std::vector<std::size_t> expected(robot.jointModelGroup()->getVariableCount());
  std::iota(expected.begin(), expected.end(), 0U);
  if (indices != expected) {
    throw std::invalid_argument("IK solver mapping must cover every group variable exactly once.");
  }
}

moveit::core::RobotState makeSeedState(const RobotModelContext& robot,
                                       const std::vector<double>& seed_rad) {
  if (seed_rad.empty() || seed_rad.size() != robot.jointModelGroup()->getVariableCount() ||
      !finitePositions(seed_rad)) {
    throw std::invalid_argument("IK seed must contain one finite value per group variable.");
  }
  auto state = robot.makeDefaultState();
  state.setJointGroupPositions(robot.jointModelGroup(), seed_rad);
  state.update();
  if (!state.satisfiesBounds(robot.jointModelGroup())) {
    throw std::invalid_argument("IK seed must satisfy the robot joint limits.");
  }
  return state;
}

Eigen::Isometry3d targetTransform(const domain::Pose& pose) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  const auto& rotation = pose.orientation;
  transform.linear() = Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z)
                           .normalized()
                           .toRotationMatrix();
  return transform;
}

geometry_msgs::msg::Pose poseMessage(const Eigen::Isometry3d& transform) {
  geometry_msgs::msg::Pose message;
  message.position.x = transform.translation().x();
  message.position.y = transform.translation().y();
  message.position.z = transform.translation().z();
  const Eigen::Quaterniond orientation(transform.rotation());
  message.orientation.x = orientation.x();
  message.orientation.y = orientation.y();
  message.orientation.z = orientation.z();
  message.orientation.w = orientation.w();
  return message;
}

SampleResult checkSolution(const RobotModelContext& robot, moveit::core::RobotState& state,
                           const Eigen::Isometry3d& target, SampleResult result) {
  state.setJointGroupPositions(robot.jointModelGroup(), result.joint_positions_rad);
  state.update();
  if (!state.satisfiesBounds(robot.jointModelGroup())) {
    result.failure_reason = FailureReason::kJointLimit;
    return result;
  }
  const auto& actual = state.getGlobalLinkTransform(robot.tcpLink());
  const double position_error = (actual.translation() - target.translation()).norm();
  const double orientation_error =
      Eigen::Quaterniond(actual.rotation()).angularDistance(Eigen::Quaterniond(target.rotation()));
  if (!std::isfinite(position_error) || !std::isfinite(orientation_error) ||
      position_error > kPositionToleranceM || orientation_error > kOrientationToleranceRad) {
    result.failure_reason = FailureReason::kIkFailed;
    return result;
  }
  result.reachable = true;
  result.failure_reason.reset();
  return result;
}

} // namespace

SampleResult solveSeededIk(const RobotModelContext& robot, const domain::TargetPose& target,
                           const std::vector<double>& seed_rad, std::size_t sample_index) {
  const auto* group = robot.jointModelGroup();
  const auto solver = group->getSolverInstance();
  if (!solver) {
    throw std::invalid_argument("Planning group has no IK solver.");
  }
  validateSolver(robot, *solver);
  auto state = makeSeedState(robot, seed_rad);
  if (!domain::validate(target).empty() || target.pose.frame_id != robot.modelFrame()) {
    return {sample_index, false, {}, FailureReason::kInvalidTarget};
  }

  const auto model_target = targetTransform(target.pose);
  const Eigen::Isometry3d base_transform =
      solver->getBaseFrame() == robot.modelFrame()
          ? Eigen::Isometry3d::Identity()
          : state.getGlobalLinkTransform(solver->getBaseFrame());
  const auto& mapping = group->getKinematicsSolverJointBijection();
  std::vector<double> solver_seed;
  solver_seed.reserve(mapping.size());
  for (const auto index : mapping) {
    solver_seed.push_back(seed_rad.at(index));
  }

  std::vector<double> solution;
  moveit_msgs::msg::MoveItErrorCodes error;
  const bool solved = solver->getPositionIK(poseMessage(base_transform.inverse() * model_target),
                                            solver_seed, solution, error);
  if (!solved || error.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
      solution.size() != mapping.size() || !finitePositions(solution)) {
    return {sample_index, false, {}, FailureReason::kIkFailed};
  }

  SampleResult result{sample_index, false, std::vector<double>(seed_rad.size()), std::nullopt};
  for (std::size_t index = 0; index < mapping.size(); ++index) {
    result.joint_positions_rad.at(mapping[index]) = solution[index];
  }
  return checkSolution(robot, state, model_target, result);
}

} // namespace rco_core::reachability
