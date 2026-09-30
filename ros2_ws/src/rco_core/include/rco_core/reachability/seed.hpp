#pragma once

#include "rco_core/reachability/result.hpp"

#include <vector>

namespace rco_core::reachability {

class ContinuitySeed {
public:
  explicit ContinuitySeed(std::vector<double> initial_joint_positions_rad);

  [[nodiscard]] const std::vector<double>& current() const noexcept;

  void observe(const SampleResult& result);

  void reset();

private:
  std::vector<double> initial_joint_positions_rad_;
  std::vector<double> current_joint_positions_rad_;
};

} // namespace rco_core::reachability
