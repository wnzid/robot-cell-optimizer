#pragma once

#include "rco_core/domain/model.hpp"

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <string>
#include <vector>

namespace rco_core::scene {

// Builds a deterministic MoveIt world diff from a validated cell definition.
// Applying or publishing the diff belongs to a later ROS-facing increment.
[[nodiscard]] moveit_msgs::msg::PlanningScene
makePlanningSceneWorldDiff(const domain::CellDefinition& cell);

// Applies the validated cell as a world diff to an in-memory MoveIt scene.
// Returns MoveIt's application result; validation failures throw before mutation.
[[nodiscard]] bool applyCellWorldDiff(planning_scene::PlanningScene& scene,
                                      const domain::CellDefinition& cell);

// Replaces all world collision objects with the validated cell contents.
// Validation failures throw before any existing world object is removed.
[[nodiscard]] bool replaceCellWorld(planning_scene::PlanningScene& scene,
                                    const domain::CellDefinition& cell);

// Applies validated tool geometry to the configured TCP parent link.
// Validation failures throw before the target scene is mutated.
[[nodiscard]] bool applyAttachedTool(planning_scene::PlanningScene& scene,
                                     const domain::ToolDefinition& tool,
                                     const std::vector<std::string>& touch_links);

// Replaces one explicitly identified attached tool. The replacement is fully
// validated and its attachment link checked before the previous body is removed.
[[nodiscard]] bool replaceAttachedTool(planning_scene::PlanningScene& scene,
                                       const std::string& previous_tool_id,
                                       const domain::ToolDefinition& tool,
                                       const std::vector<std::string>& touch_links);

} // namespace rco_core::scene
