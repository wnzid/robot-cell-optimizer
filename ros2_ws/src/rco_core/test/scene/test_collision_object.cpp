#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/scene/collision_object.hpp"

#include <filesystem>
#include <gtest/gtest.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using rco_core::domain::CellDefinition;
using rco_core::domain::CellEntity;
using rco_core::domain::CellEntityType;
using rco_core::domain::GeometryType;
using rco_core::domain::ToolDefinition;

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

CellDefinition makeCell() {
  CellDefinition cell;
  cell.id = "cell_1";
  cell.robot_base.frame_id = "world";
  cell.robot_base.orientation.w = 1.0;
  cell.workpiece_pose.frame_id = "world";
  cell.workpiece_pose.orientation.w = 1.0;
  return cell;
}

ToolDefinition makeTool() {
  ToolDefinition tool;
  tool.id = "tool_1";
  tool.name = "reference_tool";
  tool.mass_kg = 1.0;
  tool.tcp.frame_id = "tool0";
  tool.tcp.orientation.w = 1.0;
  return tool;
}

std::string tetrahedronMeshUri() {
  const auto mesh_path =
      std::filesystem::path(__FILE__).parent_path() / "resources" / "tetrahedron.stl";
  return "file://" + mesh_path.string();
}

std::string packageTetrahedronMeshUri() {
  return "package://rco_description/meshes/tetrahedron.stl";
}

std::string packageTetrahedronObjMeshUri() {
  return "package://rco_description/meshes/tetrahedron.obj";
}

std::string packageTetrahedronDaeMeshUri() {
  return "package://rco_description/meshes/tetrahedron.dae";
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

TEST(CollisionObjectTest, ConvertsLocalStlMeshWithPose) {
  auto entity = makeEntity(GeometryType::kMesh);
  entity.geometry.mesh_uri = tetrahedronMeshUri();

  const auto object = rco_core::scene::makeCollisionObject(entity);

  EXPECT_TRUE(object.primitives.empty());
  EXPECT_TRUE(object.primitive_poses.empty());
  ASSERT_EQ(object.meshes.size(), 1U);
  ASSERT_EQ(object.mesh_poses.size(), 1U);
  EXPECT_FALSE(object.meshes.front().vertices.empty());
  EXPECT_EQ(object.meshes.front().triangles.size(), 4U);
  EXPECT_DOUBLE_EQ(object.mesh_poses.front().position.x, 1.0);
  EXPECT_DOUBLE_EQ(object.mesh_poses.front().position.y, 2.0);
  EXPECT_DOUBLE_EQ(object.mesh_poses.front().position.z, 3.0);
  EXPECT_DOUBLE_EQ(object.mesh_poses.front().orientation.w, 1.0);
}

TEST(CollisionObjectTest, ConvertsPackageStlMesh) {
  auto entity = makeEntity(GeometryType::kMesh);
  entity.geometry.mesh_uri = packageTetrahedronMeshUri();

  const auto object = rco_core::scene::makeCollisionObject(entity);

  ASSERT_EQ(object.meshes.size(), 1U);
  EXPECT_FALSE(object.meshes.front().vertices.empty());
  EXPECT_EQ(object.meshes.front().triangles.size(), 4U);
}

TEST(CollisionObjectTest, ConvertsPackageObjMesh) {
  auto entity = makeEntity(GeometryType::kMesh);
  entity.geometry.mesh_uri = packageTetrahedronObjMeshUri();

  const auto object = rco_core::scene::makeCollisionObject(entity);

  ASSERT_EQ(object.meshes.size(), 1U);
  EXPECT_FALSE(object.meshes.front().vertices.empty());
  EXPECT_EQ(object.meshes.front().triangles.size(), 4U);
}

TEST(CollisionObjectTest, ConvertsPackageDaeMesh) {
  auto entity = makeEntity(GeometryType::kMesh);
  entity.geometry.mesh_uri = packageTetrahedronDaeMeshUri();

  const auto object = rco_core::scene::makeCollisionObject(entity);

  ASSERT_EQ(object.meshes.size(), 1U);
  EXPECT_FALSE(object.meshes.front().vertices.empty());
  EXPECT_EQ(object.meshes.front().triangles.size(), 4U);
}

TEST(CollisionObjectTest, RejectsUnreadableMeshUri) {
  auto entity = makeEntity(GeometryType::kMesh);
  entity.geometry.mesh_uri = "file:///definitely/not/a/mesh.stl";

  EXPECT_THROW(static_cast<void>(rco_core::scene::makeCollisionObject(entity)),
               std::invalid_argument);
}

TEST(CollisionObjectTest, ConvertsToolPrimitivesInDefinitionOrder) {
  auto tool = makeTool();
  auto box = rco_core::domain::GeometryDefinition{};
  box.type = GeometryType::kBox;
  box.size_m = {0.2, 0.1, 0.05};
  auto cylinder = rco_core::domain::GeometryDefinition{};
  cylinder.type = GeometryType::kCylinder;
  cylinder.radius_m = 0.03;
  cylinder.height_m = 0.15;
  tool.geometry = {box, cylinder};

  const auto object = rco_core::scene::makeToolCollisionObject(tool);

  EXPECT_EQ(object.id, "tool_1");
  EXPECT_EQ(object.header.frame_id, "tool0");
  EXPECT_EQ(object.operation, moveit_msgs::msg::CollisionObject::ADD);
  ASSERT_EQ(object.primitives.size(), 2U);
  ASSERT_EQ(object.primitive_poses.size(), 2U);
  EXPECT_EQ(object.primitives[0].type, shape_msgs::msg::SolidPrimitive::BOX);
  EXPECT_EQ(object.primitives[1].type, shape_msgs::msg::SolidPrimitive::CYLINDER);
  EXPECT_DOUBLE_EQ(object.primitive_poses[0].orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(object.primitive_poses[1].orientation.w, 1.0);
}

TEST(CollisionObjectTest, RejectsInvalidToolBeforeConversion) {
  auto tool = makeTool();
  auto box = rco_core::domain::GeometryDefinition{};
  box.type = GeometryType::kBox;
  box.size_m = {0.2, 0.0, 0.05};
  tool.geometry = {box};

  EXPECT_THROW(static_cast<void>(rco_core::scene::makeToolCollisionObject(tool)),
               std::invalid_argument);
}

TEST(CollisionObjectTest, WrapsToolGeometryForConfiguredLink) {
  auto tool = makeTool();
  auto box = rco_core::domain::GeometryDefinition{};
  box.type = GeometryType::kBox;
  box.size_m = {0.2, 0.1, 0.05};
  tool.geometry = {box};
  const std::vector<std::string> touch_links{"tool0", "wrist_3_link"};

  const auto attached = rco_core::scene::makeAttachedToolCollisionObject(tool, touch_links);

  EXPECT_EQ(attached.link_name, "tool0");
  EXPECT_EQ(attached.object.id, "tool_1");
  EXPECT_EQ(attached.object.header.frame_id, "tool0");
  EXPECT_EQ(attached.object.operation, moveit_msgs::msg::CollisionObject::ADD);
  ASSERT_EQ(attached.object.primitives.size(), 1U);
  EXPECT_EQ(attached.touch_links, touch_links);
}

TEST(CollisionObjectTest, ConvertsCellEntitiesInDefinitionOrder) {
  auto cell = makeCell();
  auto box = makeEntity(GeometryType::kBox);
  box.id = "table_1";
  box.type = CellEntityType::kTable;
  box.geometry.size_m = {1.0, 0.5, 0.1};
  auto sphere = makeEntity(GeometryType::kSphere);
  sphere.id = "obstacle_1";
  sphere.geometry.radius_m = 0.25;
  cell.entities = {box, sphere};

  const auto objects = rco_core::scene::makeCollisionObjects(cell);

  ASSERT_EQ(objects.size(), 2U);
  EXPECT_EQ(objects[0].id, "table_1");
  EXPECT_EQ(objects[0].primitives.front().type, shape_msgs::msg::SolidPrimitive::BOX);
  EXPECT_EQ(objects[1].id, "obstacle_1");
  EXPECT_EQ(objects[1].primitives.front().type, shape_msgs::msg::SolidPrimitive::SPHERE);
}

TEST(CollisionObjectTest, ConvertsEmptyValidCellToEmptyList) {
  const auto objects = rco_core::scene::makeCollisionObjects(makeCell());

  EXPECT_TRUE(objects.empty());
}

TEST(CollisionObjectTest, RejectsInvalidCellBeforeConversion) {
  auto cell = makeCell();
  auto entity = makeEntity(GeometryType::kSphere);
  entity.geometry.radius_m = 0.25;
  cell.entities = {entity, entity};

  EXPECT_THROW(static_cast<void>(rco_core::scene::makeCollisionObjects(cell)),
               std::invalid_argument);
}

} // namespace
