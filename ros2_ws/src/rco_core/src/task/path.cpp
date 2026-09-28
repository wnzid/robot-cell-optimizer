#include "rco_core/task/path.hpp"

#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/domain/validation.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rco_core::task {
namespace {

void requireValidTarget(const domain::TargetPose& target, std::string_view label) {
  const auto errors = domain::validate(target);
  if (!errors.empty()) {
    const auto& error = errors.front();
    throw std::invalid_argument(std::string(label) + " target is invalid at " + error.path + ": " +
                                error.message);
  }
}

Eigen::Quaterniond toEigen(const domain::Quaternion& quaternion) {
  return {quaternion.w, quaternion.x, quaternion.y, quaternion.z};
}

domain::Quaternion fromEigen(const Eigen::Quaterniond& quaternion) {
  return {
      quaternion.x(),
      quaternion.y(),
      quaternion.z(),
      quaternion.w(),
  };
}

} // namespace

domain::TargetPose interpolateTargetPose(const domain::TargetPose& start,
                                         const domain::TargetPose& finish, double fraction) {
  requireValidTarget(start, "start");
  requireValidTarget(finish, "finish");
  if (start.pose.frame_id != finish.pose.frame_id) {
    throw std::invalid_argument("target frames must match for interpolation");
  }
  if (!std::isfinite(fraction) || fraction < 0.0 || fraction > 1.0) {
    throw std::invalid_argument("interpolation fraction must be finite and within [0, 1]");
  }
  if (fraction == 0.0) {
    return start;
  }
  if (fraction == 1.0) {
    return finish;
  }

  domain::TargetPose interpolated;
  interpolated.pose.frame_id = start.pose.frame_id;
  interpolated.pose.position.x =
      start.pose.position.x + ((finish.pose.position.x - start.pose.position.x) * fraction);
  interpolated.pose.position.y =
      start.pose.position.y + ((finish.pose.position.y - start.pose.position.y) * fraction);
  interpolated.pose.position.z =
      start.pose.position.z + ((finish.pose.position.z - start.pose.position.z) * fraction);

  const Eigen::Quaterniond orientation = toEigen(start.pose.orientation)
                                             .slerp(fraction, toEigen(finish.pose.orientation))
                                             .normalized();
  interpolated.pose.orientation = fromEigen(orientation);
  return interpolated;
}

} // namespace rco_core::task
