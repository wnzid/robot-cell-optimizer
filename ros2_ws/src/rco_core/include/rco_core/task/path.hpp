#pragma once

#include "rco_core/domain/model.hpp"

namespace rco_core::task {

// Interpolates position linearly and orientation along the shortest quaternion
// arc. Both targets must be valid and expressed in the same frame.
[[nodiscard]] domain::TargetPose interpolateTargetPose(const domain::TargetPose& start,
                                                       const domain::TargetPose& finish,
                                                       double fraction);

} // namespace rco_core::task
