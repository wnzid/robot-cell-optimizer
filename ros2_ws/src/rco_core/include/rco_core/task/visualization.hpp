#pragma once

#include "rco_core/domain/model.hpp"

#include <string_view>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rco_core::task {

// Produces deterministic, zero-timestamp markers without requiring a ROS node.
[[nodiscard]] visualization_msgs::msg::MarkerArray
makeTaskPathMarkers(const domain::TaskDefinition& task, std::string_view marker_namespace);

} // namespace rco_core::task
