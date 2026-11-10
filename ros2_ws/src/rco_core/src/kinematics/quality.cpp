#include "rco_core/kinematics/quality.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <vector>

namespace rco_core::kinematics {
namespace {

void validateOptions(const QualityOptions& options) {
  if (!std::isfinite(options.characteristic_length_m) || options.characteristic_length_m <= 0.0 ||
      !std::isfinite(options.relative_singular_threshold) ||
      options.relative_singular_threshold <= 0.0 || options.relative_singular_threshold >= 1.0) {
    throw std::invalid_argument("Quality options require positive length and threshold in (0, 1).");
  }
}

double percentile(const std::vector<double>& sorted, double probability) {
  const double position = static_cast<double>(sorted.size() - 1U) * probability;
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  return std::lerp(sorted.at(lower), sorted.at(upper), position - static_cast<double>(lower));
}

std::optional<MetricSummary> summarize(std::vector<double> values) {
  if (values.empty()) {
    return std::nullopt;
  }
  std::ranges::sort(values);
  double mean = 0.0;
  const double count = static_cast<double>(values.size());
  for (const auto value : values) {
    mean += value / count;
  }
  if (!std::isfinite(mean)) {
    throw std::overflow_error("Quality metric mean is not representable.");
  }
  return MetricSummary{values.size(), values.front(), values.back(), mean,
                       percentile(values, 0.05), percentile(values, 0.95)};
}

} // namespace

JacobianQuality analyzeJacobian(const Eigen::MatrixXd& jacobian, const QualityOptions& options) {
  validateOptions(options);
  if (jacobian.rows() != 6 || jacobian.cols() != 6 || !jacobian.allFinite()) {
    throw std::invalid_argument("Quality analysis requires a finite 6x6 geometric Jacobian.");
  }
  Eigen::MatrixXd scaled = jacobian;
  scaled.topRows(3) /= options.characteristic_length_m;
  if (!scaled.allFinite()) {
    throw std::overflow_error("Scaled Jacobian is not representable.");
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(scaled);
  const auto& singular = svd.singularValues();
  if (svd.info() != Eigen::Success || !singular.allFinite()) {
    throw std::overflow_error("Jacobian SVD failed to produce finite singular values.");
  }
  JacobianQuality result;
  result.minimum_singular_value = singular(5);
  result.maximum_singular_value = singular(0);
  result.manipulability = singular.prod();
  if (!std::isfinite(result.manipulability)) {
    throw std::overflow_error("Jacobian manipulability is not representable.");
  }
  const double cutoff = options.relative_singular_threshold * result.maximum_singular_value;
  for (Eigen::Index index = 0; index < singular.size(); ++index) {
    if (singular(index) > cutoff) {
      ++result.numerical_rank;
    }
  }
  result.near_singular = result.numerical_rank < 6U;
  if (!result.near_singular && result.minimum_singular_value > 0.0) {
    const double condition = result.maximum_singular_value / result.minimum_singular_value;
    if (!std::isfinite(condition)) {
      throw std::overflow_error("Jacobian condition number is not representable.");
    }
    result.condition_number = condition;
  }
  return result;
}

PathQuality analyzeJacobianPath(const std::vector<Eigen::MatrixXd>& jacobians,
                                const QualityOptions& options) {
  validateOptions(options);
  PathQuality result;
  result.sample_count = jacobians.size();
  std::vector<double> minimum_values;
  std::vector<double> manipulabilities;
  std::vector<double> conditions;
  minimum_values.reserve(jacobians.size());
  manipulabilities.reserve(jacobians.size());
  conditions.reserve(jacobians.size());
  for (const auto& jacobian : jacobians) {
    const auto quality = analyzeJacobian(jacobian, options);
    minimum_values.push_back(quality.minimum_singular_value);
    manipulabilities.push_back(quality.manipulability);
    if (quality.near_singular) {
      ++result.near_singular_count;
    }
    if (quality.condition_number.has_value()) {
      conditions.push_back(quality.condition_number.value());
    }
  }
  result.minimum_singular_value = summarize(minimum_values);
  result.manipulability = summarize(manipulabilities);
  result.condition_number = summarize(conditions);
  return result;
}

} // namespace rco_core::kinematics
