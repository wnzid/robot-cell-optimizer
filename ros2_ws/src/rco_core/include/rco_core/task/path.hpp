#pragma once

#include "rco_core/domain/model.hpp"

#include <vector>

namespace rco_core::task {

// Interpolates position linearly and orientation along the shortest quaternion
// arc. Both targets must be valid and expressed in the same frame.
[[nodiscard]] domain::TargetPose interpolateTargetPose(const domain::TargetPose& start,
                                                       const domain::TargetPose& finish,
                                                       double fraction);

// Preserves every authored target and inserts interpolated targets so adjacent
// output poses do not exceed either requested spacing.
[[nodiscard]] std::vector<domain::TargetPose>
resampleTargetPath(const std::vector<domain::TargetPose>& targets, double max_linear_spacing_m,
                   double max_angular_spacing_rad);

} // namespace rco_core::task
