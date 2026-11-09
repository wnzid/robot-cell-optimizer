#include "rco_core/domain/model.hpp"
#include "rco_core/reachability/path.hpp"
#include "rco_core/reachability/result.hpp"

#include <cstddef>
#include <gtest/gtest.h>
#include <optional>
#include <stdexcept>
#include <vector>

namespace {

using rco_core::domain::TargetPose;
using rco_core::reachability::evaluatePath;
using rco_core::reachability::FailureReason;
using rco_core::reachability::SampleResult;

TargetPose validTarget() {
  TargetPose target;
  target.pose.frame_id = "world";
  return target;
}

TEST(ReachabilityPathTest, PropagatesSuccessfulSeedAndPreservesItAcrossFailures) {
  std::vector<std::vector<double>> seeds;
  const auto results =
      evaluatePath(std::vector<TargetPose>(3U, validTarget()), {0.0, 0.0},
                   [&seeds](const TargetPose&, const std::vector<double>& seed) {
                     seeds.push_back(seed);
                     if (seeds.size() == 2U) {
                       return SampleResult{99U, false, {}, FailureReason::kIkFailed};
                     }
                     return SampleResult{99U, true, {0.1, 0.2}, std::nullopt};
                   });

  ASSERT_EQ(results.size(), 3U);
  EXPECT_EQ(seeds, (std::vector<std::vector<double>>{{0.0, 0.0}, {0.1, 0.2}, {0.1, 0.2}}));
  for (std::size_t index = 0; index < results.size(); ++index) {
    EXPECT_EQ(results[index].sample_index, index);
  }
  EXPECT_EQ(results[1U].failure_reason, FailureReason::kIkFailed);
  EXPECT_TRUE(results[2U].reachable);
}

TEST(ReachabilityPathTest, SkipsInvalidTargetsWithoutChangingSeed) {
  std::size_t calls = 0U;
  const auto results = evaluatePath({TargetPose{}, validTarget()}, {0.0},
                                    [&calls](const TargetPose&, const std::vector<double>& seed) {
                                      ++calls;
                                      EXPECT_EQ(seed, (std::vector<double>{0.0}));
                                      return SampleResult{0U, true, {0.1}, std::nullopt};
                                    });

  EXPECT_EQ(calls, 1U);
  ASSERT_EQ(results.size(), 2U);
  EXPECT_EQ(results[0U].failure_reason, FailureReason::kInvalidTarget);
  EXPECT_TRUE(results[1U].reachable);
}

TEST(ReachabilityPathTest, EmptyPathNeverCallsSolver) {
  const auto results = evaluatePath({}, {0.0}, [](const TargetPose&, const std::vector<double>&) {
    ADD_FAILURE() << "Empty path must not invoke solver";
    return SampleResult{};
  });
  EXPECT_TRUE(results.empty());
}

TEST(ReachabilityPathTest, RejectsMissingSolverAndInvalidSeed) {
  EXPECT_THROW(static_cast<void>(evaluatePath({}, {0.0}, {})), std::invalid_argument);
  const auto solver = [](const TargetPose&, const std::vector<double>&) {
    return SampleResult{0U, true, {0.1}, std::nullopt};
  };
  EXPECT_THROW(static_cast<void>(evaluatePath({}, {}, solver)), std::invalid_argument);
}

TEST(ReachabilityPathTest, RejectsMalformedResultsAndWrongJointCount) {
  const auto malformed = [](const TargetPose&, const std::vector<double>&) {
    return SampleResult{0U, true, {}, std::nullopt};
  };
  EXPECT_THROW(static_cast<void>(evaluatePath({validTarget()}, {0.0}, malformed)),
               std::invalid_argument);
  const auto wrong_count = [](const TargetPose&, const std::vector<double>&) {
    return SampleResult{0U, true, {0.1, 0.2}, std::nullopt};
  };
  EXPECT_THROW(static_cast<void>(evaluatePath({validTarget()}, {0.0}, wrong_count)),
               std::invalid_argument);
}

TEST(ReachabilityPathTest, PropagatesSolverErrors) {
  const auto solver = [](const TargetPose&, const std::vector<double>&) -> SampleResult {
    throw std::runtime_error("IK backend unavailable");
  };
  EXPECT_THROW(static_cast<void>(evaluatePath({validTarget()}, {0.0}, solver)), std::runtime_error);
}

TEST(ReachabilityPathTest, ResetsSeedBetweenInvocations) {
  const auto solver = [](const TargetPose&, const std::vector<double>& seed) {
    EXPECT_EQ(seed, (std::vector<double>{0.0}));
    return SampleResult{0U, true, {0.1}, std::nullopt};
  };
  const auto first = evaluatePath({validTarget()}, {0.0}, solver);
  const auto second = evaluatePath({validTarget()}, {0.0}, solver);
  EXPECT_EQ(first, second);
}

} // namespace
