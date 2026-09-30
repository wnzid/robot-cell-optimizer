#include "rco_core/task/visualization.hpp"

#include "rco_core/domain/model.hpp"
#include "rco_core/domain/validation.hpp"

#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <limits>
#include <std_msgs/msg/color_rgba.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rco_core::task {
namespace {

constexpr double kPathLineWidthM = 0.005;
constexpr double kWaypointDiameterM = 0.012;
constexpr double kOrientationArrowLengthM = 0.04;
constexpr double kOrientationArrowShaftDiameterM = 0.008;
constexpr double kOrientationArrowHeadDiameterM = 0.012;

void requireValidTask(const domain::TaskDefinition& task) {
  const auto errors = domain::validate(task);
  if (!errors.empty()) {
    const auto& error = errors.front();
    throw std::invalid_argument("task is invalid at " + error.path + ": " + error.message);
  }
}

std_msgs::msg::ColorRGBA segmentColor(domain::ProcessSegmentType type) {
  std_msgs::msg::ColorRGBA color;
  color.a = 1.0F;
  switch (type) {
  case domain::ProcessSegmentType::kProcess:
    color.r = 0.1F;
    color.g = 0.8F;
    color.b = 0.2F;
    break;
  case domain::ProcessSegmentType::kApproach:
    color.r = 0.1F;
    color.g = 0.4F;
    color.b = 1.0F;
    break;
  case domain::ProcessSegmentType::kRetract:
    color.r = 1.0F;
    color.g = 0.55F;
    color.b = 0.1F;
    break;
  case domain::ProcessSegmentType::kTransition:
    color.r = 0.65F;
    color.g = 0.65F;
    color.b = 0.65F;
    break;
  }
  return color;
}

std::int32_t nextMarkerId(std::int32_t& marker_id) {
  if (marker_id == std::numeric_limits<std::int32_t>::max()) {
    throw std::length_error("task requires too many visualization markers");
  }
  const std::int32_t next = marker_id;
  ++marker_id;
  return next;
}

geometry_msgs::msg::Point targetPoint(const domain::TargetPose& target) {
  geometry_msgs::msg::Point point;
  point.x = target.pose.position.x;
  point.y = target.pose.position.y;
  point.z = target.pose.position.z;
  return point;
}

visualization_msgs::msg::Marker makeLineMarker(const domain::ProcessSegment& segment,
                                               std::string_view marker_namespace,
                                               std::int32_t marker_id,
                                               const std_msgs::msg::ColorRGBA& color) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = segment.targets.front().pose.frame_id;
  marker.ns = std::string(marker_namespace);
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = kPathLineWidthM;
  marker.color = color;
  for (const auto& target : segment.targets) {
    marker.points.push_back(targetPoint(target));
  }
  return marker;
}

visualization_msgs::msg::Marker makeWaypointMarker(const domain::ProcessSegment& segment,
                                                   std::string_view marker_namespace,
                                                   std::int32_t marker_id,
                                                   const std_msgs::msg::ColorRGBA& color) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = segment.targets.front().pose.frame_id;
  marker.ns = std::string(marker_namespace);
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = kWaypointDiameterM;
  marker.scale.y = kWaypointDiameterM;
  marker.scale.z = kWaypointDiameterM;
  marker.color = color;
  for (const auto& target : segment.targets) {
    marker.points.push_back(targetPoint(target));
  }
  return marker;
}

visualization_msgs::msg::Marker makeOrientationMarker(const domain::TargetPose& target,
                                                      std::string_view marker_namespace,
                                                      std::int32_t marker_id,
                                                      const std_msgs::msg::ColorRGBA& color) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = target.pose.frame_id;
  marker.ns = std::string(marker_namespace);
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = target.pose.position.x;
  marker.pose.position.y = target.pose.position.y;
  marker.pose.position.z = target.pose.position.z;
  marker.pose.orientation.x = target.pose.orientation.x;
  marker.pose.orientation.y = target.pose.orientation.y;
  marker.pose.orientation.z = target.pose.orientation.z;
  marker.pose.orientation.w = target.pose.orientation.w;
  marker.scale.x = kOrientationArrowLengthM;
  marker.scale.y = kOrientationArrowShaftDiameterM;
  marker.scale.z = kOrientationArrowHeadDiameterM;
  marker.color = color;
  return marker;
}

} // namespace

visualization_msgs::msg::MarkerArray makeTaskPathMarkers(const domain::TaskDefinition& task,
                                                         std::string_view marker_namespace) {
  if (marker_namespace.empty()) {
    throw std::invalid_argument("marker namespace must not be empty");
  }
  requireValidTask(task);

  visualization_msgs::msg::MarkerArray result;
  std::int32_t marker_id = 0;
  for (const auto& segment : task.segments) {
    const auto color = segmentColor(segment.type);
    result.markers.push_back(
        makeLineMarker(segment, marker_namespace, nextMarkerId(marker_id), color));
    result.markers.push_back(
        makeWaypointMarker(segment, marker_namespace, nextMarkerId(marker_id), color));
    for (const auto& target : segment.targets) {
      result.markers.push_back(
          makeOrientationMarker(target, marker_namespace, nextMarkerId(marker_id), color));
    }
  }
  return result;
}

} // namespace rco_core::task
