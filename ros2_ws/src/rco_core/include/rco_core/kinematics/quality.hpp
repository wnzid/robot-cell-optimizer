#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <optional>
#include <vector>

namespace rco_core::kinematics {

// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
struct QualityOptions {
  double characteristic_length_m{1.0};
  double relative_singular_threshold{1e-6};
};

struct JacobianQuality {
  double minimum_singular_value{0.0};
  double maximum_singular_value{0.0};
  double manipulability{0.0};
  // Absent for a zero or rank-deficient Jacobian under the configured threshold.
  std::optional<double> condition_number;
  std::size_t numerical_rank{0U};
  bool near_singular{true};
};

struct MetricSummary {
  std::size_t count{0U};
  double minimum{0.0};
  double maximum{0.0};
  double mean{0.0};
  double percentile_05{0.0};
  double percentile_95{0.0};
};

struct PathQuality {
  std::size_t sample_count{0U};
  std::size_t near_singular_count{0U};
  // Empty input produces absent summaries, never an implicit success.
  std::optional<MetricSummary> minimum_singular_value;
  std::optional<MetricSummary> manipulability;
  // Only defined condition numbers contribute; count can be < sample_count.
  std::optional<MetricSummary> condition_number;
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

// Six-axis geometric Jacobian: rows [linear m/rad; angular rad/rad]. Divide
// linear rows by characteristic_length_m before SVD. Inputs must be finite,
// 6x6, with positive finite length and threshold strictly between 0 and 1.
// Rank counts sigma > threshold * sigma_max. Manipulability is the product
// of all six scaled singular values, independent of the rank cutoff.
// Invalid input throws invalid_argument; unrepresentable metrics throw overflow_error.
[[nodiscard]] JacobianQuality analyzeJacobian(const Eigen::MatrixXd& jacobian,
                                             const QualityOptions& options = {});

// Analyze every matrix with identical options; preserve all samples in counts.
// Percentiles use linear interpolation at (count - 1) * probability.
[[nodiscard]] PathQuality analyzeJacobianPath(const std::vector<Eigen::MatrixXd>& jacobians,
                                             const QualityOptions& options = {});

} // namespace rco_core::kinematics
