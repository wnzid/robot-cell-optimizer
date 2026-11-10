#include "rco_core/reachability/summary.hpp"

#include "rco_core/reachability/result.hpp"

#include <cstddef>
#include <optional>
#include <set>
#include <stdexcept>
#include <vector>

namespace rco_core::reachability {

PathSummary summarizePath(const std::vector<SampleResult>& results) {
  PathSummary summary;
  summary.sample_count = results.size();
  std::set<std::size_t> indices;

  for (const auto& result : results) {
    if (!validate(result).empty()) {
      throw std::invalid_argument("Cannot summarize an invalid reachability sample.");
    }
    if (!indices.insert(result.sample_index).second) {
      throw std::invalid_argument("Reachability sample indices must be unique.");
    }
    if (result.reachable) {
      ++summary.reachable_count;
    } else if (result.failure_reason.has_value()) {
      ++summary.failure_counts[result.failure_reason.value()];
    } else {
      throw std::invalid_argument("Unreachable sample must provide a failure reason.");
    }
  }

  if (!results.empty()) {
    summary.reachable_fraction = static_cast<double>(summary.reachable_count) /
                                 static_cast<double>(summary.sample_count);
    summary.fully_reachable = summary.reachable_count == summary.sample_count;
  }
  return summary;
}

} // namespace rco_core::reachability
