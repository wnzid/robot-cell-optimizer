#pragma once

#include "rco_core/domain/model.hpp"

#include <moveit_msgs/msg/planning_scene.hpp>

namespace rco_core::scene {

// Builds a deterministic MoveIt world diff from a validated cell definition.
// Applying or publishing the diff belongs to a later ROS-facing increment.
[[nodiscard]] moveit_msgs::msg::PlanningScene
makePlanningSceneWorldDiff(const domain::CellDefinition& cell);

} // namespace rco_core::scene
