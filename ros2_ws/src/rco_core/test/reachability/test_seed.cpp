#include "rco_core/reachability/result.hpp"
#include "rco_core/reachability/seed.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace {

using rco_core::reachability::ContinuitySeed;
using rco_core::reachability::FailureReason;
using rco_core::reachability::SampleResult;

TEST(ContinuitySeedTest, StartsFromConfiguredInitialJointPositions) {
  const ContinuitySeed seed({0.1, -0.2, 0.3});

  EXPECT_EQ(seed.current(), (std::vector<double>{0.1, -0.2, 0.3}));
}

TEST(ContinuitySeedTest, AdvancesAfterReachableSample) {
  ContinuitySeed seed({0.1, -0.2, 0.3});
  seed.observe(SampleResult{0U, true, {0.4, -0.5, 0.6}, std::nullopt});

  EXPECT_EQ(seed.current(), (std::vector<double>{0.4, -0.5, 0.6}));
}

TEST(ContinuitySeedTest, RetainsLastReachableSolutionAcrossFailure) {
  ContinuitySeed seed({0.1, -0.2, 0.3});
  seed.observe(SampleResult{0U, true, {0.4, -0.5, 0.6}, std::nullopt});
  seed.observe(SampleResult{1U, false, {0.7, -0.8, 0.9}, FailureReason::kCollision});

  EXPECT_EQ(seed.current(), (std::vector<double>{0.4, -0.5, 0.6}));
}

TEST(ContinuitySeedTest, ResetsToConfiguredInitialJointPositions) {
  ContinuitySeed seed({0.1, -0.2, 0.3});
  seed.observe(SampleResult{0U, true, {0.4, -0.5, 0.6}, std::nullopt});

  seed.reset();

  EXPECT_EQ(seed.current(), (std::vector<double>{0.1, -0.2, 0.3}));
}

TEST(ContinuitySeedTest, RejectsEmptyInitialSeed) {
  EXPECT_THROW(static_cast<void>(ContinuitySeed({})), std::invalid_argument);
}

TEST(ContinuitySeedTest, RejectsNonFiniteInitialSeed) {
  EXPECT_THROW(static_cast<void>(ContinuitySeed({0.1, std::numeric_limits<double>::infinity()})),
               std::invalid_argument);
}

TEST(ContinuitySeedTest, RejectsInvalidSampleResult) {
  ContinuitySeed seed({0.1, -0.2, 0.3});

  EXPECT_THROW(seed.observe(SampleResult{0U, true, {}, std::nullopt}), std::invalid_argument);
}

TEST(ContinuitySeedTest, RejectsReachableSolutionWithDifferentJointCount) {
  ContinuitySeed seed({0.1, -0.2, 0.3});

  EXPECT_THROW(seed.observe(SampleResult{0U, true, {0.4, -0.5}, std::nullopt}),
               std::invalid_argument);
}

} // namespace
