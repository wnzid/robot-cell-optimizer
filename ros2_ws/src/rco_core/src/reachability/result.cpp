#include "rco_core/reachability/result.hpp"

#include "rco_core/domain/validation.hpp"

#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rco_core::reachability {
namespace {

bool isSupportedFailureReason(FailureReason reason) {
  switch (reason) {
  case FailureReason::kIkFailed:
  case FailureReason::kJointLimit:
  case FailureReason::kCollision:
  case FailureReason::kDiscontinuity:
  case FailureReason::kInvalidTarget:
    return true;
  default:
    return false;
  }
}

} // namespace

std::string_view failureReasonCode(FailureReason reason) {
  switch (reason) {
  case FailureReason::kIkFailed:
    return "IK_FAILED";
  case FailureReason::kJointLimit:
    return "JOINT_LIMIT";
  case FailureReason::kCollision:
    return "COLLISION";
  case FailureReason::kDiscontinuity:
    return "DISCONTINUITY";
  case FailureReason::kInvalidTarget:
    return "INVALID_TARGET";
  default:
    throw std::invalid_argument("Unsupported reachability failure reason.");
  }
}

std::optional<FailureReason> failureReasonFromCode(std::string_view code) {
  if (code == "IK_FAILED") {
    return FailureReason::kIkFailed;
  }
  if (code == "JOINT_LIMIT") {
    return FailureReason::kJointLimit;
  }
  if (code == "COLLISION") {
    return FailureReason::kCollision;
  }
  if (code == "DISCONTINUITY") {
    return FailureReason::kDiscontinuity;
  }
  if (code == "INVALID_TARGET") {
    return FailureReason::kInvalidTarget;
  }
  return std::nullopt;
}

domain::ValidationErrors validate(const SampleResult& result) {
  domain::ValidationErrors errors;

  if (result.reachable && result.joint_positions_rad.empty()) {
    errors.push_back({
        ".joint_positions_rad",
        "must contain at least one joint position when the sample is reachable",
    });
  }

  for (std::size_t index = 0; index < result.joint_positions_rad.size(); ++index) {
    if (!std::isfinite(result.joint_positions_rad[index])) {
      errors.push_back({
          ".joint_positions_rad[" + std::to_string(index) + "]",
          "must be a finite radian value",
      });
    }
  }

  if (result.reachable && result.failure_reason.has_value()) {
    errors.push_back({
        ".failure_reason",
        "must be absent when the sample is reachable",
    });
  } else if (!result.reachable && !result.failure_reason.has_value()) {
    errors.push_back({
        ".failure_reason",
        "must be present when the sample is unreachable",
    });
  } else if (result.failure_reason.has_value() &&
             !isSupportedFailureReason(*result.failure_reason)) {
    errors.push_back({
        ".failure_reason",
        "unsupported reachability failure reason",
    });
  }

  return errors;
}

} // namespace rco_core::reachability
