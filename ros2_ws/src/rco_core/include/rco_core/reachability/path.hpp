#pragma once

#include "rco_core/domain/model.hpp"
#include "rco_core/reachability/result.hpp"
#include "rco_core/robot_model_context.hpp"

#include <moveit/planning_scene/planning_scene.hpp>
#include <vector>

namespace rco_core::reachability {

// Evaluate a nonempty ordered path with one IK attempt per sample. Every sample
// receives a result, including failures. Only accepted solutions advance the seed.
// Reject any absolute per-joint displacement exceeding max_joint_step_rad,
// measured from the last accepted seed (or initial seed for the first sample).
// Joint coordinates are unwrapped: winding changes are not reduced modulo 2*pi.
// max_joint_step_rad must be finite and positive. Inputs otherwise follow
// solveSeededIk's contract. This does not check collisions or swept motion.
[[nodiscard]] std::vector<SampleResult>
solveSeededPath(const RobotModelContext& robot, const std::vector<domain::TargetPose>& targets,
                const std::vector<double>& initial_seed_rad, double max_joint_step_rad);

// Also rejects self/world/attached-tool collisions using scene's allowed
// collision matrix and padding. The scene must share robot.robotModel().
// Non-group joints and attached bodies come from scene's current state.
// Neither the scene nor its state is modified. Collision precedes the joint-step
// check; rejected solutions retain diagnostic joints but never advance the seed.
// Checks discrete samples only, not swept motion between them.
[[nodiscard]] std::vector<SampleResult>
solveSeededPath(const RobotModelContext& robot, const planning_scene::PlanningScene& scene,
                const std::vector<domain::TargetPose>& targets,
                const std::vector<double>& initial_seed_rad, double max_joint_step_rad);

} // namespace rco_core::reachability
