#include "rco_core/scene/planning_scene.hpp"

#include "rco_core/domain/model.hpp"
#include "rco_core/scene/collision_object.hpp"

#include <moveit_msgs/msg/planning_scene.hpp>

namespace rco_core::scene {

moveit_msgs::msg::PlanningScene makePlanningSceneWorldDiff(const domain::CellDefinition& cell) {
  moveit_msgs::msg::PlanningScene scene;
  scene.name = cell.id;
  scene.world.collision_objects = makeCollisionObjects(cell);
  scene.robot_state.is_diff = true;
  scene.is_diff = true;
  return scene;
}

} // namespace rco_core::scene
