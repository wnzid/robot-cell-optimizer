#include "rco_core/reachability/path.hpp"

#include "rco_core/domain/validation.hpp"
#include "rco_core/reachability/result.hpp"
#include "rco_core/reachability/seed.hpp"

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rco_core::reachability {

std::vector<SampleResult> evaluatePath(const std::vector<domain::TargetPose>& targets,
                                       const std::vector<double>& initial_joint_positions_rad,
                                       const SampleSolver& solver) {
  if (!solver) {
    throw std::invalid_argument("Reachability sample solver must be provided.");
  }

  ContinuitySeed seed(initial_joint_positions_rad);
  std::vector<SampleResult> results;
  results.reserve(targets.size());

  for (std::size_t index = 0; index < targets.size(); ++index) {
    if (!domain::validate(targets[index]).empty()) {
      results.push_back(SampleResult{index, false, {}, FailureReason::kInvalidTarget});
      continue;
    }

    auto result = solver(targets[index], seed.current());
    result.sample_index = index;
    seed.observe(result);
    results.push_back(std::move(result));
  }

  return results;
}

} // namespace rco_core::reachability
