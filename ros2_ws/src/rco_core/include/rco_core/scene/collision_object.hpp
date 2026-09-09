#pragma once

#include "rco_core/domain/model.hpp"

#include <moveit_msgs/msg/collision_object.hpp>
#include <vector>

namespace rco_core::scene {

// Converts one validated primitive cell entity into a MoveIt collision object.
// Mesh geometry is intentionally deferred to a later scene-engine increment.
[[nodiscard]] moveit_msgs::msg::CollisionObject
makeCollisionObject(const domain::CellEntity& entity);

// Preserves CellDefinition entity order and validates the complete cell before
// producing any collision objects.
[[nodiscard]] std::vector<moveit_msgs::msg::CollisionObject>
makeCollisionObjects(const domain::CellDefinition& cell);

} // namespace rco_core::scene
