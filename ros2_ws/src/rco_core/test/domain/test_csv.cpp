#include "rco_core/domain/csv.hpp"
#include "rco_core/domain/model.hpp"
#include "rco_core/domain/validation.hpp"

#include <algorithm>
#include <gtest/gtest.h>
#include <string>
#include <string_view>
#include <vector>

namespace {

using rco_core::domain::TargetPathCsvParseResult;
using rco_core::domain::TargetPose;
using rco_core::domain::ValidationErrors;

bool containsError(const ValidationErrors& errors, const std::string& path,
                   const std::string& message) {
  return std::ranges::any_of(errors, [&path, &message](const auto& error) {
    return error.path == path && error.message == message;
  });
}

TEST(TargetPathCsvTest, ParsesValidatedTargetsWithCrLfAndWhitespace) {
  constexpr std::string_view csv = "frame_id,x_m,y_m,z_m,qx,qy,qz,qw\r\n"
                                   " world, 1.25, -0.5, 0.125, 0, 0, 0, 1 \r\n"
                                   "workpiece,0,0.25,0.5,0.5,-0.5,0.5,-0.5\r\n";

  const TargetPathCsvParseResult result = rco_core::domain::parseTargetPathCsv(csv);
  const auto targets = result.value.value_or(std::vector<TargetPose>{});

  EXPECT_TRUE(result.ok());
  EXPECT_TRUE(result.errors.empty());
  ASSERT_EQ(targets.size(), 2U);
  EXPECT_EQ(targets[0].pose.frame_id, "world");
  EXPECT_DOUBLE_EQ(targets[0].pose.position.x, 1.25);
  EXPECT_DOUBLE_EQ(targets[0].pose.position.y, -0.5);
  EXPECT_DOUBLE_EQ(targets[0].pose.position.z, 0.125);
  EXPECT_DOUBLE_EQ(targets[0].pose.orientation.w, 1.0);
  EXPECT_EQ(targets[1].pose.frame_id, "workpiece");
  EXPECT_DOUBLE_EQ(targets[1].pose.position.y, 0.25);
  EXPECT_DOUBLE_EQ(targets[1].pose.orientation.x, 0.5);
  EXPECT_DOUBLE_EQ(targets[1].pose.orientation.w, -0.5);
}

TEST(TargetPathCsvTest, RejectsUnexpectedHeader) {
  const auto result =
      rco_core::domain::parseTargetPathCsv("frame,x_m,y_m,z_m,qx,qy,qz,qw\nworld,0,0,0,0,0,0,1\n");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(
      containsError(result.errors, ".header", "must equal: frame_id,x_m,y_m,z_m,qx,qy,qz,qw"));
}

TEST(TargetPathCsvTest, RejectsRowsWithWrongColumnCount) {
  const auto result =
      rco_core::domain::parseTargetPathCsv("frame_id,x_m,y_m,z_m,qx,qy,qz,qw\nworld,0,0,0,0,0,1\n");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".rows[0]", "must contain exactly 8 columns"));
}

TEST(TargetPathCsvTest, ReportsInvalidNumericField) {
  const auto result = rco_core::domain::parseTargetPathCsv(
      "frame_id,x_m,y_m,z_m,qx,qy,qz,qw\nworld,not-a-number,0,0,0,0,0,1\n");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".rows[0].x_m", "must be a finite number"));
}

TEST(TargetPathCsvTest, ReportsPoseValidationPath) {
  const auto result = rco_core::domain::parseTargetPathCsv(
      "frame_id,x_m,y_m,z_m,qx,qy,qz,qw\nworld,0,0,0,0,0,0,2\n");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(
      containsError(result.errors, ".rows[0].pose.orientation", "quaternion must be normalized"));
}

TEST(TargetPathCsvTest, RejectsPathWithoutTargets) {
  const auto result = rco_core::domain::parseTargetPathCsv("frame_id,x_m,y_m,z_m,qx,qy,qz,qw\n");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".rows", "must contain at least one target"));
}

} // namespace
