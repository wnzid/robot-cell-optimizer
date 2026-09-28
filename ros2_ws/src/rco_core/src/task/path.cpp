#include "rco_core/task/path.hpp"

#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/domain/validation.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace rco_core::task {
namespace {

constexpr std::size_t kMaximumResampledTargets = 1'000'000U;

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

domain::TargetPose interpolateValidTargets(const domain::TargetPose& start,
                                           const domain::TargetPose& finish, double fraction) {
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

double linearDistance(const domain::TargetPose& start, const domain::TargetPose& finish) {
  const double difference_x = finish.pose.position.x - start.pose.position.x;
  const double difference_y = finish.pose.position.y - start.pose.position.y;
  const double difference_z = finish.pose.position.z - start.pose.position.z;
  return std::sqrt((difference_x * difference_x) + (difference_y * difference_y) +
                   (difference_z * difference_z));
}

double angularDistance(const domain::TargetPose& start, const domain::TargetPose& finish) {
  const double absolute_dot =
      std::abs(toEigen(start.pose.orientation).dot(toEigen(finish.pose.orientation)));
  return 2.0 * std::acos(std::clamp(absolute_dot, 0.0, 1.0));
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
  return interpolateValidTargets(start, finish, fraction);
}

std::vector<domain::TargetPose> resampleTargetPath(const std::vector<domain::TargetPose>& targets,
                                                   double max_linear_spacing_m,
                                                   double max_angular_spacing_rad) {
  if (targets.empty()) {
    throw std::invalid_argument("target path must contain at least one target");
  }
  if (targets.size() > kMaximumResampledTargets) {
    throw std::length_error("target path exceeds the resampling target limit");
  }
  if (!std::isfinite(max_linear_spacing_m) || max_linear_spacing_m <= 0.0) {
    throw std::invalid_argument("maximum linear spacing must be finite and greater than zero");
  }
  if (!std::isfinite(max_angular_spacing_rad) || max_angular_spacing_rad <= 0.0) {
    throw std::invalid_argument("maximum angular spacing must be finite and greater than zero");
  }

  for (std::size_t index = 0U; index < targets.size(); ++index) {
    requireValidTarget(targets.at(index), "path[" + std::to_string(index) + "]");
  }

  std::vector<domain::TargetPose> resampled{targets.front()};
  for (std::size_t index = 1U; index < targets.size(); ++index) {
    const auto& start = targets.at(index - 1U);
    const auto& finish = targets.at(index);
    if (start.pose.frame_id != finish.pose.frame_id) {
      throw std::invalid_argument("adjacent target frames must match for resampling");
    }

    const double linear_ratio = linearDistance(start, finish) / max_linear_spacing_m;
    const double angular_ratio = angularDistance(start, finish) / max_angular_spacing_rad;
    if (linear_ratio > static_cast<double>(kMaximumResampledTargets) ||
        angular_ratio > static_cast<double>(kMaximumResampledTargets)) {
      throw std::length_error("target path exceeds the resampling target limit");
    }

    const auto linear_intervals = static_cast<std::size_t>(std::ceil(linear_ratio));
    const auto angular_intervals = static_cast<std::size_t>(std::ceil(angular_ratio));
    const std::size_t intervals = std::max({std::size_t{1U}, linear_intervals, angular_intervals});
    if (resampled.size() + intervals > kMaximumResampledTargets) {
      throw std::length_error("target path exceeds the resampling target limit");
    }

    for (std::size_t interval = 1U; interval <= intervals; ++interval) {
      const double fraction = static_cast<double>(interval) / static_cast<double>(intervals);
      resampled.push_back(interpolateValidTargets(start, finish, fraction));
    }
  }
  return resampled;
}

} // namespace rco_core::task
