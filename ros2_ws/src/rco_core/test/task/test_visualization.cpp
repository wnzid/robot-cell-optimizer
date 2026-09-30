#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/task/visualization.hpp"

#include <array>
#include <cstddef>
#include <gtest/gtest.h>
#include <stdexcept>
#include <string_view>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace {

using rco_core::domain::ProcessSegment;
using rco_core::domain::ProcessSegmentType;
using rco_core::domain::TargetPose;
using rco_core::domain::TaskDefinition;

TargetPose makeTarget(double position_x) {
  TargetPose target;
  target.pose.frame_id = "workpiece";
  target.pose.position.x = position_x;
  target.pose.orientation.w = 1.0;
  return target;
}

ProcessSegment makeSegment(ProcessSegmentType type, double position_x) {
  ProcessSegment segment;
  segment.type = type;
  segment.targets = {makeTarget(position_x)};
  segment.tcp_speed_mps = 0.1;
  return segment;
}

TaskDefinition makeTask() {
  TaskDefinition task;
  task.id = "weld_task";
  auto segment = makeSegment(ProcessSegmentType::kProcess, 0.0);
  segment.targets.push_back(makeTarget(0.25));
  segment.targets.back().pose.orientation = {0.0, 0.0, 1.0, 0.0};
  task.segments = {segment};
  return task;
}

TaskDefinition makeAllSegmentTypesTask() {
  TaskDefinition task;
  task.id = "all_segments";
  const std::array types{ProcessSegmentType::kProcess, ProcessSegmentType::kApproach,
                         ProcessSegmentType::kRetract, ProcessSegmentType::kTransition};
  for (std::size_t index = 0U; index < types.size(); ++index) {
    task.segments.push_back(makeSegment(types.at(index), static_cast<double>(index)));
  }
  return task;
}

void expectStableColorsAndMarkerIds(const visualization_msgs::msg::MarkerArray& markers) {
  ASSERT_EQ(markers.markers.size(), 12U);
  EXPECT_FLOAT_EQ(markers.markers[0].color.g, 0.8F);
  EXPECT_FLOAT_EQ(markers.markers[3].color.b, 1.0F);
  EXPECT_FLOAT_EQ(markers.markers[6].color.r, 1.0F);
  EXPECT_FLOAT_EQ(markers.markers[9].color.r, 0.65F);
  EXPECT_EQ(markers.markers[11].id, 11);
}

TEST(TaskVisualizationTest, BuildsOrderedPathWaypointAndOrientationMarkers) {
  const auto markers = rco_core::task::makeTaskPathMarkers(makeTask(), "weld_path");

  ASSERT_EQ(markers.markers.size(), 4U);
  const auto& line = markers.markers[0];
  EXPECT_EQ(line.header.frame_id, "workpiece");
  EXPECT_EQ(line.ns, "weld_path");
  EXPECT_EQ(line.id, 0);
  EXPECT_EQ(line.type, visualization_msgs::msg::Marker::LINE_STRIP);
  ASSERT_EQ(line.points.size(), 2U);
  EXPECT_DOUBLE_EQ(line.points[0].x, 0.0);
  EXPECT_DOUBLE_EQ(line.points[1].x, 0.25);
  EXPECT_FLOAT_EQ(line.color.r, 0.1F);
  EXPECT_FLOAT_EQ(line.color.g, 0.8F);
  EXPECT_FLOAT_EQ(line.color.b, 0.2F);
  EXPECT_FLOAT_EQ(line.color.a, 1.0F);

  const auto& waypoints = markers.markers[1];
  EXPECT_EQ(waypoints.id, 1);
  EXPECT_EQ(waypoints.type, visualization_msgs::msg::Marker::SPHERE_LIST);
  EXPECT_EQ(waypoints.points, line.points);

  EXPECT_EQ(markers.markers[2].id, 2);
  EXPECT_EQ(markers.markers[2].type, visualization_msgs::msg::Marker::ARROW);
  EXPECT_DOUBLE_EQ(markers.markers[2].pose.orientation.w, 1.0);
  EXPECT_EQ(markers.markers[3].id, 3);
  EXPECT_DOUBLE_EQ(markers.markers[3].pose.position.x, 0.25);
  EXPECT_DOUBLE_EQ(markers.markers[3].pose.orientation.z, 1.0);
  EXPECT_DOUBLE_EQ(markers.markers[3].pose.orientation.w, 0.0);
}

TEST(TaskVisualizationTest, UsesStableSegmentColorsAndMarkerIds) {
  const auto task = makeAllSegmentTypesTask();
  const auto first = rco_core::task::makeTaskPathMarkers(task, "task");
  const auto second = rco_core::task::makeTaskPathMarkers(task, "task");

  EXPECT_EQ(first, second);
  expectStableColorsAndMarkerIds(first);
}

TEST(TaskVisualizationTest, RejectsInvalidInputBeforeRendering) {
  auto invalid_task = makeTask();
  invalid_task.id.clear();

  EXPECT_THROW(static_cast<void>(rco_core::task::makeTaskPathMarkers(invalid_task, "weld_path")),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(rco_core::task::makeTaskPathMarkers(makeTask(), "")),
               std::invalid_argument);
}

} // namespace
