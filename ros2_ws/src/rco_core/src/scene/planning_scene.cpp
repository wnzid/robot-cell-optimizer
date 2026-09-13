#include "rco_core/scene/planning_scene.hpp"

#include "rco_core/domain/model.hpp"
#include "rco_core/scene/collision_object.hpp"

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <string>
#include <vector>

namespace rco_core::scene {

moveit_msgs::msg::PlanningScene makePlanningSceneWorldDiff(const domain::CellDefinition& cell) {
  moveit_msgs::msg::PlanningScene scene;
  scene.name = cell.id;
  scene.world.collision_objects = makeCollisionObjects(cell);
  scene.robot_state.is_diff = true;
  scene.is_diff = true;
  return scene;
}

bool applyCellWorldDiff(planning_scene::PlanningScene& scene, const domain::CellDefinition& cell) {
  return scene.setPlanningSceneDiffMsg(makePlanningSceneWorldDiff(cell));
}

bool replaceCellWorld(planning_scene::PlanningScene& scene, const domain::CellDefinition& cell) {
  const auto world_diff = makePlanningSceneWorldDiff(cell);
  scene.removeAllCollisionObjects();
  return scene.setPlanningSceneDiffMsg(world_diff);
}

bool applyAttachedTool(planning_scene::PlanningScene& scene, const domain::ToolDefinition& tool,
                       const std::vector<std::string>& touch_links) {
  return scene.processAttachedCollisionObjectMsg(
      makeAttachedToolCollisionObject(tool, touch_links));
}

bool replaceAttachedTool(planning_scene::PlanningScene& scene, const std::string& previous_tool_id,
                         const domain::ToolDefinition& tool,
                         const std::vector<std::string>& touch_links) {
  const auto attached_tool = makeAttachedToolCollisionObject(tool, touch_links);
  if (!scene.getRobotModel()->hasLinkModel(attached_tool.link_name)) {
    return false;
  }

  if (!previous_tool_id.empty()) {
    scene.getCurrentStateNonConst().clearAttachedBody(previous_tool_id);
  }
  return scene.processAttachedCollisionObjectMsg(attached_tool);
}

} // namespace rco_core::scene
