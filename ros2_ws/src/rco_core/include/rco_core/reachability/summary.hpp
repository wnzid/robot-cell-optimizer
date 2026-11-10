#pragma once

#include "rco_core/reachability/result.hpp"

#include <cstddef>
#include <map>
#include <optional>
#include <vector>

namespace rco_core::reachability {

// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
struct PathSummary {
  std::size_t sample_count{0U};
  std::size_t reachable_count{0U};
  std::map<FailureReason, std::size_t> failure_counts;
  // Empty input has no reachability fraction and is not a successful path.
  std::optional<double> reachable_fraction;
  bool fully_reachable{false};

  bool operator==(const PathSummary&) const = default;
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

// Summarizes sampled reachability only, not continuous trajectory feasibility.
// Requires valid results with unique sample indices; input order is immaterial.
// Throws std::invalid_argument for malformed results or duplicate indices.
[[nodiscard]] PathSummary summarizePath(const std::vector<SampleResult>& results);

} // namespace rco_core::reachability
