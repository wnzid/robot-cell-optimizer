#include "rco_core/reachability/seed.hpp"

#include "rco_core/reachability/result.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rco_core::reachability {
namespace {

void validateInitialSeed(const std::vector<double>& seed) {
  if (seed.empty()) {
    throw std::invalid_argument(
        "Initial continuity seed must contain at least one joint position.");
  }

  for (std::size_t index = 0; index < seed.size(); ++index) {
    if (!std::isfinite(seed[index])) {
      throw std::invalid_argument("Initial continuity seed joint position at index " +
                                  std::to_string(index) + " must be finite.");
    }
  }
}

} // namespace

ContinuitySeed::ContinuitySeed(std::vector<double> initial_joint_positions_rad)
    : initial_joint_positions_rad_(std::move(initial_joint_positions_rad)),
      current_joint_positions_rad_(initial_joint_positions_rad_) {
  validateInitialSeed(initial_joint_positions_rad_);
}

const std::vector<double>& ContinuitySeed::current() const noexcept {
  return current_joint_positions_rad_;
}

void ContinuitySeed::observe(const SampleResult& result) {
  const auto errors = validate(result);
  if (!errors.empty()) {
    throw std::invalid_argument("Reachability sample result is invalid at " + errors.front().path +
                                ": " + errors.front().message + ".");
  }

  if (!result.reachable) {
    return;
  }

  if (result.joint_positions_rad.size() != initial_joint_positions_rad_.size()) {
    throw std::invalid_argument("Reachability sample joint count must match the continuity seed.");
  }

  current_joint_positions_rad_ = result.joint_positions_rad;
}

void ContinuitySeed::reset() {
  current_joint_positions_rad_ = initial_joint_positions_rad_;
}

} // namespace rco_core::reachability
