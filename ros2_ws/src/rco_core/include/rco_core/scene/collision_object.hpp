#pragma once

#include "rco_core/domain/model.hpp"

#include <moveit_msgs/msg/collision_object.hpp>

namespace rco_core::scene {

// Converts one validated primitive cell entity into a MoveIt collision object.
// Mesh geometry is intentionally deferred to a later scene-engine increment.
[[nodiscard]] moveit_msgs::msg::CollisionObject
makeCollisionObject(const domain::CellEntity& entity);

} // namespace rco_core::scene
