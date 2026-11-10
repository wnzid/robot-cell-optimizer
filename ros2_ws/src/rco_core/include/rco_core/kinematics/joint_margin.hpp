#pragma once

#include "rco_core/domain/types.hpp"

#include <vector>

namespace rco_core::kinematics {

// One normalized distance to the nearer limit per joint, in input order.
// margin = 2 * min(q - lower, upper - q) / (upper - lower).
// 0 means at a limit; 1 means centered. Requires nonempty equal-sized arrays,
// unique nonempty joint names, finite radian values, finite positive ranges,
// and positions inside their limits. Invalid inputs throw invalid_argument.
// This describes bounded joints; continuous joints need an explicit policy.
[[nodiscard]] std::vector<double>
normalizedJointMargins(const std::vector<domain::JointLimits>& limits,
                       const std::vector<double>& positions_rad);

} // namespace rco_core::kinematics
