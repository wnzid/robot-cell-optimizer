#pragma once

#include "rco_core/domain/validation.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace rco_core::reachability {

enum class FailureReason : std::uint8_t {
  kIkFailed,
  kJointLimit,
  kCollision,
  kDiscontinuity,
  kInvalidTarget,
};

// Reachability results are intentionally plain data records so evaluation
// stages can compose and compare them deterministically.
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)

struct SampleResult {
  std::size_t sample_index{0U};
  bool reachable{false};
  std::vector<double> joint_positions_rad;
  std::optional<FailureReason> failure_reason;

  bool operator==(const SampleResult&) const = default;
};

// NOLINTEND(misc-non-private-member-variables-in-classes)

[[nodiscard]] std::string_view failureReasonCode(FailureReason reason);

[[nodiscard]] std::optional<FailureReason> failureReasonFromCode(std::string_view code);

[[nodiscard]] domain::ValidationErrors validate(const SampleResult& result);

} // namespace rco_core::reachability
