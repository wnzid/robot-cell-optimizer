#include <Eigen/Core>
#include <cmath>
#include <gtest/gtest.h>

namespace {

TEST(Toolchain, UsesCxx20OrNewer) {
  static_assert(__cplusplus >= 202002L);
  SUCCEED();
}

TEST(EigenIntegration, ComputesExpectedVectorNorm) {
  const Eigen::Vector3d vector{1.0, 2.0, 3.0};

  EXPECT_DOUBLE_EQ(vector.squaredNorm(), 14.0);
  EXPECT_NEAR(vector.norm(), std::sqrt(14.0), 1e-12);
}

} // namespace
