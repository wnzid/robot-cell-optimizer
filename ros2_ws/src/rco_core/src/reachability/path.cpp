#include "rco_core/reachability/path.hpp"

#include "rco_core/domain/model.hpp"
#include "rco_core/reachability/ik.hpp"
#include "rco_core/reachability/result.hpp"
#include "rco_core/reachability/seed.hpp"
#include "rco_core/robot_model_context.hpp"

#include <cmath>
#include <cstddef>
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

} // namespace

std::vector<SampleResult> solveSeededPath(const RobotModelContext& robot,
                                          const std::vector<domain::TargetPose>& targets,
                                          const std::vector<double>& initial_seed_rad,
                                          double max_joint_step_rad) {
  if (targets.empty()) {
    throw std::invalid_argument("Reachability path must contain at least one target.");
  }
  if (!std::isfinite(max_joint_step_rad) || max_joint_step_rad <= 0.0) {
    throw std::invalid_argument("Maximum joint step must be finite and greater than zero.");
  }

  ContinuitySeed seed(initial_seed_rad);
  std::vector<SampleResult> results;
  results.reserve(targets.size());
  for (std::size_t index = 0; index < targets.size(); ++index) {
    auto result = solveSeededIk(robot, targets[index], seed.current(), index);
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

} // namespace rco_core::reachability
