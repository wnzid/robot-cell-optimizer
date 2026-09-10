#include "rco_core/scene/collision_object.hpp"

#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/domain/validation.hpp"

#include <boost/variant/get.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <stdexcept>
#include <vector>

namespace rco_core::scene {
namespace {

geometry_msgs::msg::Pose toRosPose(const domain::Pose& pose) {
  geometry_msgs::msg::Pose result;
  result.position.x = pose.position.x;
  result.position.y = pose.position.y;
  result.position.z = pose.position.z;
  result.orientation.x = pose.orientation.x;
  result.orientation.y = pose.orientation.y;
  result.orientation.z = pose.orientation.z;
  result.orientation.w = pose.orientation.w;
  return result;
}

shape_msgs::msg::SolidPrimitive makePrimitive(const domain::GeometryDefinition& geometry) {
  shape_msgs::msg::SolidPrimitive primitive;

  switch (geometry.type) {
  case domain::GeometryType::kBox:
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(shape_msgs::msg::SolidPrimitive::BOX_Z + 1U);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = geometry.size_m.x;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = geometry.size_m.y;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = geometry.size_m.z;
    break;
  case domain::GeometryType::kCylinder:
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS + 1U);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = geometry.height_m;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = geometry.radius_m;
    break;
  case domain::GeometryType::kSphere:
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions.resize(shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS + 1U);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = geometry.radius_m;
    break;
  case domain::GeometryType::kMesh:
    throw std::logic_error("Mesh geometry cannot be converted to a solid primitive.");
  }

  return primitive;
}

shape_msgs::msg::Mesh makeMesh(const domain::GeometryDefinition& geometry) {
  const std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(geometry.mesh_uri));
  if (!mesh) {
    throw std::invalid_argument("Failed to load mesh collision geometry from '" +
                                geometry.mesh_uri + "'.");
  }

  shapes::ShapeMsg shape_message;
  if (!shapes::constructMsgFromShape(mesh.get(), shape_message)) {
    throw std::invalid_argument("Failed to convert mesh collision geometry from '" +
                                geometry.mesh_uri + "'.");
  }

  const auto* mesh_message = boost::get<shape_msgs::msg::Mesh>(&shape_message);
  if (mesh_message == nullptr) {
    throw std::logic_error("Mesh conversion produced an unexpected shape message type.");
  }
  return *mesh_message;
}

} // namespace

moveit_msgs::msg::CollisionObject makeCollisionObject(const domain::CellEntity& entity) {
  const domain::ValidationErrors errors = domain::validate(entity);
  if (!errors.empty()) {
    throw std::invalid_argument("Invalid cell entity" + errors.front().path + ": " +
                                errors.front().message);
  }

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = entity.pose.frame_id;
  object.id = entity.id;
  if (entity.geometry.type == domain::GeometryType::kMesh) {
    object.meshes.push_back(makeMesh(entity.geometry));
    object.mesh_poses.push_back(toRosPose(entity.pose));
  } else {
    object.primitives.push_back(makePrimitive(entity.geometry));
    object.primitive_poses.push_back(toRosPose(entity.pose));
  }
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

std::vector<moveit_msgs::msg::CollisionObject>
makeCollisionObjects(const domain::CellDefinition& cell) {
  const domain::ValidationErrors errors = domain::validate(cell);
  if (!errors.empty()) {
    throw std::invalid_argument("Invalid cell definition" + errors.front().path + ": " +
                                errors.front().message);
  }

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.reserve(cell.entities.size());
  for (const auto& entity : cell.entities) {
    objects.push_back(makeCollisionObject(entity));
  }
  return objects;
}

} // namespace rco_core::scene
