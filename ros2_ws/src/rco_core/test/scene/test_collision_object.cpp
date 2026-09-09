#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/scene/collision_object.hpp"

#include <gtest/gtest.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <stdexcept>

namespace {

using rco_core::domain::CellEntity;
using rco_core::domain::CellEntityType;
using rco_core::domain::GeometryType;

CellEntity makeEntity(GeometryType geometry_type) {
  CellEntity entity;
  entity.id = "obstacle_1";
  entity.type = CellEntityType::kObstacle;
  entity.pose.frame_id = "world";
  entity.pose.position = {1.0, 2.0, 3.0};
  entity.pose.orientation.w = 1.0;
  entity.geometry.type = geometry_type;
  return entity;
}

TEST(CollisionObjectTest, ConvertsBoxWithPoseAndIdentity) {
  auto entity = makeEntity(GeometryType::kBox);
  entity.geometry.size_m = {0.5, 0.25, 0.125};

  const auto object = rco_core::scene::makeCollisionObject(entity);

  EXPECT_EQ(object.id, "obstacle_1");
  EXPECT_EQ(object.header.frame_id, "world");
  EXPECT_EQ(object.operation, moveit_msgs::msg::CollisionObject::ADD);
  ASSERT_EQ(object.primitives.size(), 1U);
  ASSERT_EQ(object.primitive_poses.size(), 1U);
  EXPECT_EQ(object.primitives.front().type, shape_msgs::msg::SolidPrimitive::BOX);
  EXPECT_DOUBLE_EQ(object.primitives.front().dimensions[shape_msgs::msg::SolidPrimitive::BOX_X],
                   0.5);
  EXPECT_DOUBLE_EQ(object.primitives.front().dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y],
                   0.25);
  EXPECT_DOUBLE_EQ(object.primitives.front().dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z],
                   0.125);
  EXPECT_DOUBLE_EQ(object.primitive_poses.front().position.x, 1.0);
  EXPECT_DOUBLE_EQ(object.primitive_poses.front().position.y, 2.0);
  EXPECT_DOUBLE_EQ(object.primitive_poses.front().position.z, 3.0);
  EXPECT_DOUBLE_EQ(object.primitive_poses.front().orientation.w, 1.0);
}

TEST(CollisionObjectTest, ConvertsCylinderDimensionsInRosOrder) {
  auto entity = makeEntity(GeometryType::kCylinder);
  entity.geometry.radius_m = 0.2;
  entity.geometry.height_m = 0.8;

  const auto object = rco_core::scene::makeCollisionObject(entity);

  ASSERT_EQ(object.primitives.size(), 1U);
  EXPECT_EQ(object.primitives.front().type, shape_msgs::msg::SolidPrimitive::CYLINDER);
  EXPECT_DOUBLE_EQ(
      object.primitives.front().dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT], 0.8);
  EXPECT_DOUBLE_EQ(
      object.primitives.front().dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS], 0.2);
}

TEST(CollisionObjectTest, ConvertsSphereRadius) {
  auto entity = makeEntity(GeometryType::kSphere);
  entity.geometry.radius_m = 0.3;

  const auto object = rco_core::scene::makeCollisionObject(entity);

  ASSERT_EQ(object.primitives.size(), 1U);
  EXPECT_EQ(object.primitives.front().type, shape_msgs::msg::SolidPrimitive::SPHERE);
  EXPECT_DOUBLE_EQ(
      object.primitives.front().dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS], 0.3);
}

TEST(CollisionObjectTest, RejectsInvalidEntityBeforeConversion) {
  auto entity = makeEntity(GeometryType::kBox);
  entity.geometry.size_m = {0.5, 0.0, 0.125};

  EXPECT_THROW(static_cast<void>(rco_core::scene::makeCollisionObject(entity)),
               std::invalid_argument);
}

TEST(CollisionObjectTest, DefersMeshGeometryExplicitly) {
  auto entity = makeEntity(GeometryType::kMesh);
  entity.geometry.mesh_uri = "package://rco_description/meshes/fixture.stl";

  EXPECT_THROW(static_cast<void>(rco_core::scene::makeCollisionObject(entity)),
               std::invalid_argument);
}

} // namespace
