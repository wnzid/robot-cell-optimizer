#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/task/path.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

using rco_core::domain::Quaternion;
using rco_core::domain::TargetPose;
using rco_core::domain::Vector3;

TargetPose makeTarget(std::string frame_id, Vector3 position, Quaternion orientation) {
  TargetPose target;
  target.pose.frame_id = std::move(frame_id);
  target.pose.position = position;
  target.pose.orientation = orientation;
  return target;
}

TEST(TargetPathTest, InterpolatesPositionAndOrientation) {
  const auto start = makeTarget("world", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0});
  const auto finish = makeTarget("world", {2.0, -1.0, 0.5}, {0.0, 0.0, 1.0, 0.0});

  const auto target = rco_core::task::interpolateTargetPose(start, finish, 0.5);

  const double half_sqrt_two = std::sqrt(0.5);
  EXPECT_EQ(target.pose.frame_id, "world");
  EXPECT_DOUBLE_EQ(target.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(target.pose.position.y, -0.5);
  EXPECT_DOUBLE_EQ(target.pose.position.z, 0.25);
  EXPECT_DOUBLE_EQ(target.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(target.pose.orientation.y, 0.0);
  EXPECT_NEAR(target.pose.orientation.z, half_sqrt_two, 1e-12);
  EXPECT_NEAR(target.pose.orientation.w, half_sqrt_two, 1e-12);
}

TEST(TargetPathTest, PreservesExactEndpoints) {
  const auto start = makeTarget("world", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0});
  const auto finish = makeTarget("world", {1.0, 2.0, 3.0}, {0.0, 0.0, 1.0, 0.0});

  EXPECT_EQ(rco_core::task::interpolateTargetPose(start, finish, 0.0), start);
  EXPECT_EQ(rco_core::task::interpolateTargetPose(start, finish, 1.0), finish);
}

TEST(TargetPathTest, UsesShortestQuaternionArc) {
  const auto start = makeTarget("world", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0});
  const auto finish = makeTarget("world", {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, -1.0});

  const auto target = rco_core::task::interpolateTargetPose(start, finish, 0.5);

  EXPECT_DOUBLE_EQ(target.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(target.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(target.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(std::abs(target.pose.orientation.w), 1.0);
}

TEST(TargetPathTest, RejectsMismatchedFrames) {
  const auto start = makeTarget("world", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0});
  const auto finish = makeTarget("workpiece", {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0});

  EXPECT_THROW(static_cast<void>(rco_core::task::interpolateTargetPose(start, finish, 0.5)),
               std::invalid_argument);
}

TEST(TargetPathTest, RejectsInvalidFraction) {
  const auto start = makeTarget("world", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0});
  const auto finish = makeTarget("world", {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0});

  EXPECT_THROW(static_cast<void>(rco_core::task::interpolateTargetPose(start, finish, -0.1)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(rco_core::task::interpolateTargetPose(start, finish, 1.1)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(rco_core::task::interpolateTargetPose(
                   start, finish, std::numeric_limits<double>::infinity())),
               std::invalid_argument);
}

TEST(TargetPathTest, RejectsInvalidTarget) {
  const auto start = makeTarget("world", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 2.0});
  const auto finish = makeTarget("world", {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0});

  EXPECT_THROW(static_cast<void>(rco_core::task::interpolateTargetPose(start, finish, 0.5)),
               std::invalid_argument);
}

} // namespace
