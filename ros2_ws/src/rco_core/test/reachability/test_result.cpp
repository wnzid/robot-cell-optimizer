#include "rco_core/domain/validation.hpp"
#include "rco_core/reachability/result.hpp"

#include <algorithm>
#include <array>
#include <gtest/gtest.h>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace {

using rco_core::domain::ValidationErrors;
using rco_core::reachability::FailureReason;
using rco_core::reachability::SampleResult;

struct ReasonCodeCase {
  FailureReason reason;
  std::string_view code;
};

constexpr std::array kReasonCodeCases{
    ReasonCodeCase{FailureReason::kIkFailed, "IK_FAILED"},
    ReasonCodeCase{FailureReason::kJointLimit, "JOINT_LIMIT"},
    ReasonCodeCase{FailureReason::kCollision, "COLLISION"},
    ReasonCodeCase{FailureReason::kDiscontinuity, "DISCONTINUITY"},
    ReasonCodeCase{FailureReason::kInvalidTarget, "INVALID_TARGET"},
};

bool containsPath(const ValidationErrors& errors, const std::string& expected_path) {
  return std::ranges::any_of(
      errors, [&expected_path](const auto& error) { return error.path == expected_path; });
}

TEST(ReachabilityResultTest, MapsEveryFailureReasonToStableCodeAndBack) {
  for (const auto& test_case : kReasonCodeCases) {
    EXPECT_EQ(rco_core::reachability::failureReasonCode(test_case.reason), test_case.code);
    EXPECT_EQ(rco_core::reachability::failureReasonFromCode(test_case.code), test_case.reason);
  }
}

TEST(ReachabilityResultTest, RejectsUnknownFailureReasonCode) {
  EXPECT_EQ(rco_core::reachability::failureReasonFromCode("UNKNOWN"), std::nullopt);
}

TEST(ReachabilityResultTest, AcceptsReachableSampleWithFiniteJointSolution) {
  const SampleResult result{3U, true, {0.1, -0.2, 0.3}, std::nullopt};

  EXPECT_TRUE(rco_core::reachability::validate(result).empty());
}

TEST(ReachabilityResultTest, AcceptsFailedSampleWithoutJointSolution) {
  const SampleResult result{4U, false, {}, FailureReason::kIkFailed};

  EXPECT_TRUE(rco_core::reachability::validate(result).empty());
}

TEST(ReachabilityResultTest, AcceptsFailedSampleWithDiagnosticJointSolution) {
  const SampleResult result{5U, false, {0.1, -0.2, 0.3}, FailureReason::kCollision};

  EXPECT_TRUE(rco_core::reachability::validate(result).empty());
}

TEST(ReachabilityResultTest, RejectsReachableSampleWithoutJointSolution) {
  const SampleResult result{0U, true, {}, std::nullopt};

  EXPECT_TRUE(containsPath(rco_core::reachability::validate(result), ".joint_positions_rad"));
}

TEST(ReachabilityResultTest, RejectsReachableSampleWithFailureReason) {
  const SampleResult result{0U, true, {0.1}, FailureReason::kJointLimit};

  EXPECT_TRUE(containsPath(rco_core::reachability::validate(result), ".failure_reason"));
}

TEST(ReachabilityResultTest, RejectsUnreachableSampleWithoutFailureReason) {
  const SampleResult result{0U, false, {}, std::nullopt};

  EXPECT_TRUE(containsPath(rco_core::reachability::validate(result), ".failure_reason"));
}

TEST(ReachabilityResultTest, ReportsIndexedNonFiniteJointPosition) {
  const SampleResult result{0U, true, {0.1, std::numeric_limits<double>::infinity()}, std::nullopt};

  EXPECT_TRUE(containsPath(rco_core::reachability::validate(result), ".joint_positions_rad[1]"));
}

} // namespace
