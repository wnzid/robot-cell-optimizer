#include "rco_core/scene/planning_scene.hpp"

#include "rco_core/domain/model.hpp"
#include "rco_core/scene/collision_object.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <stdexcept>
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

moveit_msgs::msg::PlanningScene makePlanningSceneWorldDiff(const domain::CellDefinition& cell,
                                                           const std::string& workpiece_frame) {
  if (workpiece_frame.empty()) {
    throw std::invalid_argument("Workpiece frame must not be empty.");
  }

  auto scene = makePlanningSceneWorldDiff(cell);
  geometry_msgs::msg::TransformStamped transform;
  // MoveIt stores fixed transforms from the named source frame into its target
  // planning frame, represented here with the planning frame as child_frame_id.
  transform.header.frame_id = workpiece_frame;
  transform.child_frame_id = cell.workpiece_pose.frame_id;
  transform.transform.translation.x = cell.workpiece_pose.position.x;
  transform.transform.translation.y = cell.workpiece_pose.position.y;
  transform.transform.translation.z = cell.workpiece_pose.position.z;
  transform.transform.rotation.x = cell.workpiece_pose.orientation.x;
  transform.transform.rotation.y = cell.workpiece_pose.orientation.y;
  transform.transform.rotation.z = cell.workpiece_pose.orientation.z;
  transform.transform.rotation.w = cell.workpiece_pose.orientation.w;
  scene.fixed_frame_transforms.push_back(transform);
  return scene;
}

bool applyCellWorldDiff(planning_scene::PlanningScene& scene, const domain::CellDefinition& cell) {
  return scene.setPlanningSceneDiffMsg(makePlanningSceneWorldDiff(cell));
}

bool applyCellWorldDiff(planning_scene::PlanningScene& scene, const domain::CellDefinition& cell,
                        const std::string& workpiece_frame) {
  return scene.setPlanningSceneDiffMsg(makePlanningSceneWorldDiff(cell, workpiece_frame));
}

bool replaceCellWorld(planning_scene::PlanningScene& scene, const domain::CellDefinition& cell) {
  const auto world_diff = makePlanningSceneWorldDiff(cell);
  scene.removeAllCollisionObjects();
  return scene.setPlanningSceneDiffMsg(world_diff);
}

bool replaceCellWorld(planning_scene::PlanningScene& scene, const domain::CellDefinition& cell,
                      const std::string& workpiece_frame) {
  const auto world_diff = makePlanningSceneWorldDiff(cell, workpiece_frame);
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
