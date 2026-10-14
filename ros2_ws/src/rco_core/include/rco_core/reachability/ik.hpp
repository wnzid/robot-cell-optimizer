#pragma once

#include "rco_core/domain/model.hpp"
#include "rco_core/reachability/result.hpp"
#include "rco_core/robot_model_context.hpp"

#include <cstddef>
#include <moveit/robot_state/robot_state.hpp>
#include <vector>

namespace rco_core::reachability {

// Kinematic reachability only: no collision or inter-sample continuity checks.
// Targets refer to the configured TCP link in robot.modelFrame(), in SI units.
// Seed and result positions use JointModelGroup variable order, in radians.
// Uses one getPositionIK call (no random reseeding); failure does not prove that
// every possible seed would fail. FK acceptance: 1e-4 m and 1e-4 rad.
// Requires a single-tip solver matching the configured TCP link. Invalid robot
// configuration or empty/nonfinite/out-of-bounds/wrong-sized seeds throw.
// Invalid target data returns INVALID_TARGET; unsupported frames are not guessed.
[[nodiscard]] SampleResult solveSeededIk(const RobotModelContext& robot,
                                         const domain::TargetPose& target,
                                         const std::vector<double>& seed_rad,
                                         std::size_t sample_index);

// Uses a caller-supplied state as the seed and context for all robot variables.
// The state must share robot.robotModel() and satisfy finite-value/joint bounds
// checks. It is copied, never mutated; attached bodies and other joints survive.
[[nodiscard]] SampleResult solveSeededIk(const RobotModelContext& robot,
                                         const domain::TargetPose& target,
                                         const moveit::core::RobotState& seed_state,
                                         std::size_t sample_index);

} // namespace rco_core::reachability
