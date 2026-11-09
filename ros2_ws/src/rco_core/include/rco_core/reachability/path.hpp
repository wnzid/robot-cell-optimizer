#pragma once

#include "rco_core/domain/model.hpp"
#include "rco_core/reachability/result.hpp"

#include <functional>
#include <vector>

namespace rco_core::reachability {

// The solver owns frame resolution, IK, joint-limit and collision checks.
// Its sample_index is replaced with the position in the input path.
using SampleSolver =
    std::function<SampleResult(const domain::TargetPose&, const std::vector<double>&)>;

// Evaluates every sample in order, retaining the last successful seed across
// failures. Invalid targets are recorded without calling the solver. Malformed
// solver results and solver exceptions propagate to the caller. Each invocation
// starts from initial_joint_positions_rad; no state is shared between paths.
// This reports sampled reachability, not collision-free trajectory feasibility.
[[nodiscard]] std::vector<SampleResult>
evaluatePath(const std::vector<domain::TargetPose>& targets,
             const std::vector<double>& initial_joint_positions_rad, const SampleSolver& solver);

} // namespace rco_core::reachability
