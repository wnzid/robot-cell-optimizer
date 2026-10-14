#include "rco_core/reachability/path.hpp"

#include "rco_core/domain/model.hpp"
#include "rco_core/reachability/ik.hpp"
#include "rco_core/reachability/result.hpp"
#include "rco_core/reachability/seed.hpp"
#include "rco_core/robot_model_context.hpp"

#include <cmath>
#include <cstddef>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rco_core::reachability {
namespace {

bool exceedsJointStep(const std::vector<double>& solution, const std::vector<double>& seed,
                      double max_joint_step_rad) {
  for (std::size_t index = 0; index < solution.size(); ++index) {
    if (std::abs(solution[index] - seed.at(index)) > max_joint_step_rad) {
      return true;
    }
  }
  return false;
}

SampleResult solveInScene(const RobotModelContext& robot,
                          const planning_scene::PlanningScene& scene,
                          const domain::TargetPose& target, const std::vector<double>& seed,
                          std::size_t index) {
  auto state = scene.getCurrentState();
  state.setJointGroupPositions(robot.jointModelGroup(), seed);
  auto result = solveSeededIk(robot, target, state, index);
  if (!result.reachable) {
    return result;
  }
  state.setJointGroupPositions(robot.jointModelGroup(), result.joint_positions_rad);
  state.update();
  const collision_detection::CollisionRequest request;
  collision_detection::CollisionResult collision;
  scene.checkCollision(request, collision, state);
  if (collision.collision) {
    result.reachable = false;
    result.failure_reason = FailureReason::kCollision;
  }
  return result;
}

std::vector<SampleResult> evaluatePath(const RobotModelContext& robot,
                                       const std::vector<domain::TargetPose>& targets,
                                       const std::vector<double>& initial_seed_rad,
                                       double max_joint_step_rad,
                                       const planning_scene::PlanningScene* scene) {
  if (targets.empty()) {
    throw std::invalid_argument("Reachability path must contain at least one target.");
  }
  if (!std::isfinite(max_joint_step_rad) || max_joint_step_rad <= 0.0) {
    throw std::invalid_argument("Maximum joint step must be finite and greater than zero.");
  }
  if (initial_seed_rad.size() != robot.jointModelGroup()->getVariableCount()) {
    throw std::invalid_argument("Initial seed must contain one value per group variable.");
  }
  if (scene != nullptr && scene->getRobotModel() != robot.robotModel()) {
    throw std::invalid_argument("Planning scene must share the configured robot model.");
  }

  ContinuitySeed seed(initial_seed_rad);
  std::vector<SampleResult> results;
  results.reserve(targets.size());
  for (std::size_t index = 0; index < targets.size(); ++index) {
    auto result = scene == nullptr
                      ? solveSeededIk(robot, targets[index], seed.current(), index)
                      : solveInScene(robot, *scene, targets[index], seed.current(), index);
    if (result.reachable &&
        exceedsJointStep(result.joint_positions_rad, seed.current(), max_joint_step_rad)) {
      result.reachable = false;
      result.failure_reason = FailureReason::kDiscontinuity;
    }
    seed.observe(result);
    results.push_back(std::move(result));
  }
  return results;
}

} // namespace

std::vector<SampleResult> solveSeededPath(const RobotModelContext& robot,
                                          const std::vector<domain::TargetPose>& targets,
                                          const std::vector<double>& initial_seed_rad,
                                          double max_joint_step_rad) {
  return evaluatePath(robot, targets, initial_seed_rad, max_joint_step_rad, nullptr);
}

std::vector<SampleResult> solveSeededPath(const RobotModelContext& robot,
                                          const planning_scene::PlanningScene& scene,
                                          const std::vector<domain::TargetPose>& targets,
                                          const std::vector<double>& initial_seed_rad,
                                          double max_joint_step_rad) {
  return evaluatePath(robot, targets, initial_seed_rad, max_joint_step_rad, &scene);
}

} // namespace rco_core::reachability
