#pragma once

#include "rco_core/domain/model.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
#include <vector>

namespace rco_core::scene {

// Converts one validated cell entity into a MoveIt collision object.
[[nodiscard]] moveit_msgs::msg::CollisionObject
makeCollisionObject(const domain::CellEntity& entity);

// Converts validated tool geometry into one MoveIt collision object in the
// configured TCP parent frame. Geometry order is preserved within each MoveIt
// shape collection and all local shape poses are identity transforms.
[[nodiscard]] moveit_msgs::msg::CollisionObject
makeToolCollisionObject(const domain::ToolDefinition& tool);

// Preserves CellDefinition entity order and validates the complete cell before
// producing any collision objects.
[[nodiscard]] std::vector<moveit_msgs::msg::CollisionObject>
makeCollisionObjects(const domain::CellDefinition& cell);

} // namespace rco_core::scene
