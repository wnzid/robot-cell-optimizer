#include "rco_core/kinematics/joint_margin.hpp"

#include "rco_core/domain/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace rco_core::kinematics {

std::vector<double> normalizedJointMargins(const std::vector<domain::JointLimits>& limits,
                                           const std::vector<double>& positions_rad) {
  if (limits.empty() || limits.size() != positions_rad.size()) {
    throw std::invalid_argument("Joint limits and positions must have the same nonzero size.");
  }
  std::set<std::string> names;
  std::vector<double> margins;
  margins.reserve(limits.size());
  for (std::size_t index = 0; index < limits.size(); ++index) {
    const auto& joint = limits[index];
    const double position = positions_rad[index];
    const double lower = joint.minimum_position_rad;
    const double upper = joint.maximum_position_rad;
    const double range = upper - lower;
    if (joint.joint_name.empty() || !names.insert(joint.joint_name).second) {
      throw std::invalid_argument("Joint names must be nonempty and unique.");
    }
    if (!std::isfinite(position) || !std::isfinite(lower) || !std::isfinite(upper) ||
        !std::isfinite(range) || range <= 0.0) {
      throw std::invalid_argument("Joint positions and positive joint ranges must be finite.");
    }
    if (position < lower || position > upper) {
      throw std::invalid_argument("Joint position at index " + std::to_string(index) +
                                  " is outside its limits.");
    }
    margins.push_back(2.0 * (std::min(position - lower, upper - position) / range));
  }
  return margins;
}

} // namespace rco_core::kinematics
