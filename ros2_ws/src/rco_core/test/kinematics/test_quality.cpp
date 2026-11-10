#include "rco_core/kinematics/quality.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <initializer_list>
#include <limits>
#include <stdexcept>

namespace {
using rco_core::kinematics::analyzeJacobian;
using rco_core::kinematics::analyzeJacobianPath;
using rco_core::kinematics::QualityOptions;

TEST(JacobianQualityTest, IdentityHasUnitMetrics) {
  const auto result = analyzeJacobian(Eigen::MatrixXd::Identity(6, 6));
  EXPECT_DOUBLE_EQ(result.minimum_singular_value, 1.0);
  EXPECT_DOUBLE_EQ(result.manipulability, 1.0);
  ASSERT_TRUE(result.condition_number.has_value());
  EXPECT_DOUBLE_EQ(result.condition_number.value(), 1.0);
  EXPECT_EQ(result.numerical_rank, 6U);
  EXPECT_FALSE(result.near_singular);
}

TEST(JacobianQualityTest, DiagonalHasAnalyticMetrics) {
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(6, 6);
  matrix(0, 0) = 2.0;
  matrix(5, 5) = 0.25;
  const auto result = analyzeJacobian(matrix);
  EXPECT_DOUBLE_EQ(result.minimum_singular_value, 0.25);
  EXPECT_DOUBLE_EQ(result.maximum_singular_value, 2.0);
  EXPECT_DOUBLE_EQ(result.manipulability, 0.5);
  ASSERT_TRUE(result.condition_number.has_value());
  EXPECT_DOUBLE_EQ(result.condition_number.value(), 8.0);
}

TEST(JacobianQualityTest, ZeroAndRankDeficientMatricesHaveNoConditionNumber) {
  const auto zero = analyzeJacobian(Eigen::MatrixXd::Zero(6, 6));
  EXPECT_EQ(zero.numerical_rank, 0U);
  EXPECT_TRUE(zero.near_singular);
  EXPECT_FALSE(zero.condition_number.has_value());
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(6, 6);
  matrix(5, 5) = 0.0;
  const auto result = analyzeJacobian(matrix);
  EXPECT_EQ(result.numerical_rank, 5U);
  EXPECT_DOUBLE_EQ(result.manipulability, 0.0);
  EXPECT_FALSE(result.condition_number.has_value());
}

TEST(JacobianQualityTest, ThresholdControlsNearSingularClassification) {
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(6, 6);
  matrix(5, 5) = 1e-8;
  EXPECT_TRUE(analyzeJacobian(matrix).near_singular);
  EXPECT_FALSE(analyzeJacobian(matrix, QualityOptions{1.0, 1e-10}).near_singular);
  EXPECT_NEAR(analyzeJacobian(matrix).manipulability, 1e-8, 1e-20);
}

TEST(JacobianQualityTest, CharacteristicLengthScalesOnlyLinearRows) {
  const auto result = analyzeJacobian(Eigen::MatrixXd::Identity(6, 6), QualityOptions{2.0, 1e-6});
  EXPECT_DOUBLE_EQ(result.minimum_singular_value, 0.5);
  EXPECT_DOUBLE_EQ(result.maximum_singular_value, 1.0);
  EXPECT_DOUBLE_EQ(result.manipulability, 0.125);
}

TEST(JacobianQualityTest, OrthogonalReferenceRotationPreservesMetrics) {
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(6, 6);
  matrix(0, 0) = 0.25;
  Eigen::MatrixXd rotation = Eigen::MatrixXd::Identity(6, 6);
  rotation(0, 0) = 0.0;
  rotation(0, 1) = -1.0;
  rotation(1, 0) = 1.0;
  rotation(1, 1) = 0.0;
  rotation.bottomRightCorner(3, 3) = rotation.topLeftCorner(3, 3).eval();
  EXPECT_NEAR(analyzeJacobian(rotation * matrix).minimum_singular_value,
              analyzeJacobian(matrix).minimum_singular_value, 1e-12);
}

TEST(JacobianQualityTest, RejectsInvalidMatricesAndOptions) {
  EXPECT_THROW(static_cast<void>(analyzeJacobian(Eigen::MatrixXd::Zero(6, 5))),
               std::invalid_argument);
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(6, 6);
  matrix(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(static_cast<void>(analyzeJacobian(matrix)), std::invalid_argument);
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(6, 6);
  for (const double length : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(static_cast<void>(analyzeJacobian(identity, QualityOptions{length, 1e-6})),
                 std::invalid_argument);
  }
  for (const double threshold : {0.0, 1.0, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(static_cast<void>(analyzeJacobian(identity, QualityOptions{1.0, threshold})),
                 std::invalid_argument);
  }
}

TEST(JacobianQualityTest, RejectsUnrepresentableScaling) {
  const Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(6, 6) * 10.0;
  EXPECT_THROW(static_cast<void>(analyzeJacobian(
                   matrix, QualityOptions{std::numeric_limits<double>::min(), 1e-6})),
               std::overflow_error);
}

TEST(JacobianQualityTest, PathSummariesIncludeWorstMeanAndInterpolatedPercentiles) {
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(6, 6);
  const auto result = analyzeJacobianPath({identity, 2.0 * identity, 3.0 * identity});
  ASSERT_TRUE(result.minimum_singular_value.has_value());
  const auto metric = result.minimum_singular_value.value();
  EXPECT_EQ(metric.count, 3U);
  EXPECT_DOUBLE_EQ(metric.minimum, 1.0);
  EXPECT_DOUBLE_EQ(metric.maximum, 3.0);
  EXPECT_DOUBLE_EQ(metric.mean, 2.0);
  EXPECT_NEAR(metric.percentile_05, 1.1, 1e-12);
  EXPECT_NEAR(metric.percentile_95, 2.9, 1e-12);
}

TEST(JacobianQualityTest, SingularPathSamplesRemainVisible) {
  const auto result = analyzeJacobianPath(
      {Eigen::MatrixXd::Identity(6, 6), Eigen::MatrixXd::Zero(6, 6)});
  EXPECT_EQ(result.sample_count, 2U);
  EXPECT_EQ(result.near_singular_count, 1U);
  ASSERT_TRUE(result.condition_number.has_value());
  EXPECT_EQ(result.condition_number.value().count, 1U);
  ASSERT_TRUE(result.minimum_singular_value.has_value());
  EXPECT_EQ(result.minimum_singular_value.value().count, 2U);
  EXPECT_DOUBLE_EQ(result.minimum_singular_value.value().minimum, 0.0);
}

TEST(JacobianQualityTest, EmptyAndEntirelySingularPathsHaveExplicitAbsence) {
  const auto empty = analyzeJacobianPath({});
  EXPECT_EQ(empty.sample_count, 0U);
  EXPECT_FALSE(empty.minimum_singular_value.has_value());
  EXPECT_FALSE(empty.condition_number.has_value());
  const auto singular = analyzeJacobianPath({Eigen::MatrixXd::Zero(6, 6)});
  EXPECT_FALSE(singular.condition_number.has_value());
  EXPECT_EQ(singular.near_singular_count, 1U);
}
} // namespace
