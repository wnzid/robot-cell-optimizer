#include "rco_core/domain/types.hpp"
#include "rco_core/kinematics/joint_margin.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

using rco_core::domain::JointLimits;
using rco_core::kinematics::normalizedJointMargins;

TEST(JointMarginTest, MeasuresCentersLimitsAndAsymmetricRanges) {
  const std::vector<JointLimits> limits{
      {"center", -2.0, 2.0}, {"lower", -1.0, 3.0}, {"upper", 2.0, 8.0}, {"quarter", 2.0, 10.0}};
  const auto margins = normalizedJointMargins(limits, {0.0, -1.0, 8.0, 4.0});

  EXPECT_EQ(margins, (std::vector<double>{1.0, 0.0, 0.0, 0.5}));
}

TEST(JointMarginTest, IsInvariantToPositiveScaleAndOffset) {
  const auto original = normalizedJointMargins({{"joint", -2.0, 6.0}}, {1.0});
  const auto transformed = normalizedJointMargins({{"joint", 4.0, 20.0}}, {10.0});

  EXPECT_EQ(original, transformed);
}

void expectInvalid(const std::vector<JointLimits>& limits, const std::vector<double>& positions) {
  EXPECT_THROW(static_cast<void>(normalizedJointMargins(limits, positions)), std::invalid_argument);
}

TEST(JointMarginTest, RejectsEmptyMismatchedAndAmbiguousInputs) {
  expectInvalid({}, {});
  expectInvalid({{"joint", -1.0, 1.0}}, {});
  expectInvalid({{"", -1.0, 1.0}}, {0.0});
  expectInvalid({{"joint", -1.0, 1.0}, {"joint", -2.0, 2.0}}, {0.0, 0.0});
}

TEST(JointMarginTest, RejectsInvalidBoundsAndInfeasiblePositions) {
  expectInvalid({{"joint", 1.0, 1.0}}, {1.0});
  expectInvalid({{"joint", 2.0, -2.0}}, {0.0});
  expectInvalid({{"joint", -1.0, 1.0}}, {-1.1});
  expectInvalid({{"joint", -1.0, 1.0}}, {1.1});
}

TEST(JointMarginTest, RejectsNonFiniteValuesAndOverflowingRanges) {
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double largest = std::numeric_limits<double>::max();
  expectInvalid({{"joint", -1.0, 1.0}}, {nan});
  expectInvalid({{"joint", -1.0, 1.0}}, {infinity});
  expectInvalid({{"joint", -infinity, 1.0}}, {0.0});
  expectInvalid({{"joint", -1.0, nan}}, {0.0});
  expectInvalid({{"joint", -largest, largest}}, {0.0});
}

TEST(JointMarginTest, HandlesSmallAndLargeFiniteRanges) {
  EXPECT_EQ(normalizedJointMargins({{"joint", 0.0, 1e-200}}, {5e-201}), (std::vector<double>{1.0}));
  EXPECT_EQ(normalizedJointMargins({{"joint", 0.0, 1e200}}, {5e199}), (std::vector<double>{1.0}));
}

} // namespace
