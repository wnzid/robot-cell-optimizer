#include "rco_core/reachability/result.hpp"
#include "rco_core/reachability/summary.hpp"

#include <algorithm>
#include <gtest/gtest.h>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace {
using rco_core::reachability::FailureReason;
using rco_core::reachability::SampleResult;
using rco_core::reachability::summarizePath;

TEST(ReachabilitySummaryTest, EmptyPathDoesNotClaimSuccess) {
  const auto summary = summarizePath({});
  EXPECT_EQ(summary.sample_count, 0U);
  EXPECT_EQ(summary.reachable_count, 0U);
  EXPECT_FALSE(summary.reachable_fraction.has_value());
  EXPECT_FALSE(summary.fully_reachable);
  EXPECT_TRUE(summary.failure_counts.empty());
}

TEST(ReachabilitySummaryTest, FullyReachablePath) {
  const auto summary =
      summarizePath({{2U, true, {0.1}, std::nullopt}, {0U, true, {0.2}, std::nullopt}});
  EXPECT_EQ(summary.sample_count, 2U);
  EXPECT_EQ(summary.reachable_count, 2U);
  EXPECT_EQ(summary.reachable_fraction, 1.0);
  EXPECT_TRUE(summary.fully_reachable);
  EXPECT_TRUE(summary.failure_counts.empty());
}

TEST(ReachabilitySummaryTest, CountsMixedFailuresWithoutLosingReasons) {
  const auto summary = summarizePath({{0U, true, {0.1}, std::nullopt},
                                     {1U, false, {}, FailureReason::kIkFailed},
                                     {2U, false, {}, FailureReason::kDiscontinuity},
                                     {3U, false, {}, FailureReason::kIkFailed}});
  EXPECT_EQ(summary.sample_count, 4U);
  EXPECT_EQ(summary.reachable_count, 1U);
  EXPECT_EQ(summary.reachable_fraction, 0.25);
  EXPECT_FALSE(summary.fully_reachable);
  EXPECT_EQ(summary.failure_counts.at(FailureReason::kIkFailed), 2U);
  EXPECT_EQ(summary.failure_counts.at(FailureReason::kDiscontinuity), 1U);
}

TEST(ReachabilitySummaryTest, EntirelyUnreachablePath) {
  const auto summary = summarizePath({{0U, false, {}, FailureReason::kInvalidTarget}});
  EXPECT_EQ(summary.reachable_fraction, 0.0);
  EXPECT_FALSE(summary.fully_reachable);
}

TEST(ReachabilitySummaryTest, RejectsMalformedResult) {
  EXPECT_THROW(static_cast<void>(summarizePath({{0U, false, {}, std::nullopt}})),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(summarizePath({{0U, true, {}, std::nullopt}})),
               std::invalid_argument);
}

TEST(ReachabilitySummaryTest, RejectsDuplicateSampleIndices) {
  const std::vector<SampleResult> results{{0U, true, {0.1}, std::nullopt},
                                        {0U, false, {}, FailureReason::kIkFailed}};
  EXPECT_THROW(static_cast<void>(summarizePath(results)), std::invalid_argument);
}

TEST(ReachabilitySummaryTest, IsIndependentOfInputOrder) {
  std::vector<SampleResult> results{{2U, true, {0.1}, std::nullopt},
                                  {7U, false, {}, FailureReason::kCollision}};
  const auto expected = summarizePath(results);
  std::reverse(results.begin(), results.end());
  EXPECT_EQ(summarizePath(results), expected);
}

TEST(ReachabilitySummaryTest, CountsEverySupportedFailureReason) {
  for (const auto reason : {FailureReason::kIkFailed, FailureReason::kJointLimit,
                            FailureReason::kCollision, FailureReason::kDiscontinuity,
                            FailureReason::kInvalidTarget}) {
    const auto summary = summarizePath({{0U, false, {0.1}, reason}});
    EXPECT_EQ(summary.failure_counts.at(reason), 1U);
    EXPECT_EQ(summary.reachable_count, 0U);
  }
}

TEST(ReachabilitySummaryTest, RejectsNonFiniteJointsAndUnsupportedReason) {
  const SampleResult nonfinite{0U, true, {std::numeric_limits<double>::infinity()}, std::nullopt};
  EXPECT_THROW(static_cast<void>(summarizePath({nonfinite})), std::invalid_argument);
  // Deliberately construct an unsupported value to exercise runtime validation.
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  const SampleResult unsupported{0U, false, {}, static_cast<FailureReason>(99)};
  EXPECT_THROW(static_cast<void>(summarizePath({unsupported})), std::invalid_argument);
}
} // namespace
