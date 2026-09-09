#include "rco_core/domain/serialization.hpp"

#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/domain/validation.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <yaml-cpp/emitter.h>
#include <yaml-cpp/emittermanip.h>
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/node/convert.h>     // IWYU pragma: keep
#include <yaml-cpp/node/detail/impl.h> // IWYU pragma: keep
#include <yaml-cpp/node/detail/node.h> // IWYU pragma: keep
#include <yaml-cpp/node/impl.h>        // IWYU pragma: keep
#include <yaml-cpp/node/iterator.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

namespace rco_core::domain {
namespace {

using KeyList = std::span<const std::string_view>;

void addError(ValidationErrors& errors, std::string path, std::string message) {
  errors.push_back({std::move(path), std::move(message)});
}

bool isAllowedKey(std::string_view key, KeyList allowed_keys) {
  return std::ranges::find(allowed_keys, key) != allowed_keys.end();
}

void checkMappingKeys(const YAML::Node& node, std::string_view path, KeyList allowed_keys,
                      ValidationErrors& errors) {
  std::vector<std::string> unknown_keys;

  for (const auto& entry : node) {
    if (!entry.first.IsScalar()) {
      addError(errors, std::string(path), "mapping keys must be scalars");
      continue;
    }

    const std::string key = entry.first.Scalar();
    if (!isAllowedKey(key, allowed_keys)) {
      unknown_keys.push_back(key);
    }
  }

  std::ranges::sort(unknown_keys);
  for (const auto& key : unknown_keys) {
    addError(errors, std::string(path) + "." + key, "unknown key");
  }
}

bool requireMapping(const YAML::Node& node, std::string_view path, ValidationErrors& errors) {
  if (node && node.IsMap()) {
    return true;
  }

  addError(errors, std::string(path), "must be a mapping");
  return false;
}

bool readRequiredString(const YAML::Node& parent, std::string_view key, std::string& output,
                        std::string_view path, ValidationErrors& errors) {
  const YAML::Node node = parent[std::string(key)];
  if (!node) {
    addError(errors, std::string(path), "missing required key");
    return false;
  }

  if (!node.IsScalar()) {
    addError(errors, std::string(path), "must be a string");
    return false;
  }

  output = node.Scalar();
  return true;
}

bool readRequiredDouble(const YAML::Node& parent, std::string_view key, double& output,
                        std::string_view path, ValidationErrors& errors) {
  const YAML::Node node = parent[std::string(key)];
  if (!node) {
    addError(errors, std::string(path), "missing required key");
    return false;
  }

  if (!node.IsScalar()) {
    addError(errors, std::string(path), "must be a number");
    return false;
  }

  try {
    output = node.as<double>();
  } catch (const YAML::Exception&) {
    addError(errors, std::string(path), "must be a number");
    return false;
  }

  return true;
}

void readVector3(const YAML::Node& node, std::string_view path, Vector3& output,
                 ValidationErrors& errors) {
  if (!requireMapping(node, path, errors)) {
    return;
  }

  constexpr std::array<std::string_view, 3> kKeys{"x", "y", "z"};
  checkMappingKeys(node, path, kKeys, errors);

  static_cast<void>(readRequiredDouble(node, "x", output.x, std::string(path) + ".x", errors));
  static_cast<void>(readRequiredDouble(node, "y", output.y, std::string(path) + ".y", errors));
  static_cast<void>(readRequiredDouble(node, "z", output.z, std::string(path) + ".z", errors));
}

void readQuaternion(const YAML::Node& node, std::string_view path, Quaternion& output,
                    ValidationErrors& errors) {
  if (!requireMapping(node, path, errors)) {
    return;
  }

  constexpr std::array<std::string_view, 4> kKeys{"x", "y", "z", "w"};
  checkMappingKeys(node, path, kKeys, errors);

  static_cast<void>(readRequiredDouble(node, "x", output.x, std::string(path) + ".x", errors));
  static_cast<void>(readRequiredDouble(node, "y", output.y, std::string(path) + ".y", errors));
  static_cast<void>(readRequiredDouble(node, "z", output.z, std::string(path) + ".z", errors));
  static_cast<void>(readRequiredDouble(node, "w", output.w, std::string(path) + ".w", errors));
}

void readPose(const YAML::Node& node, std::string_view path, Pose& output,
              ValidationErrors& errors) {
  if (!requireMapping(node, path, errors)) {
    return;
  }

  constexpr std::array<std::string_view, 3> kKeys{"frame_id", "position", "orientation"};
  checkMappingKeys(node, path, kKeys, errors);

  static_cast<void>(readRequiredString(node, "frame_id", output.frame_id,
                                       std::string(path) + ".frame_id", errors));

  const YAML::Node position = node["position"];
  if (!position) {
    addError(errors, std::string(path) + ".position", "missing required key");
  } else {
    readVector3(position, std::string(path) + ".position", output.position, errors);
  }

  const YAML::Node orientation = node["orientation"];
  if (!orientation) {
    addError(errors, std::string(path) + ".orientation", "missing required key");
  } else {
    readQuaternion(orientation, std::string(path) + ".orientation", output.orientation, errors);
  }
}

void emitVector3(YAML::Emitter& emitter, const Vector3& value) {
  emitter << YAML::BeginMap << YAML::Key << "x" << YAML::Value << value.x << YAML::Key << "y"
          << YAML::Value << value.y << YAML::Key << "z" << YAML::Value << value.z << YAML::EndMap;
}

void emitQuaternion(YAML::Emitter& emitter, const Quaternion& value) {
  emitter << YAML::BeginMap << YAML::Key << "x" << YAML::Value << value.x << YAML::Key << "y"
          << YAML::Value << value.y << YAML::Key << "z" << YAML::Value << value.z << YAML::Key
          << "w" << YAML::Value << value.w << YAML::EndMap;
}

void emitPose(YAML::Emitter& emitter, const Pose& pose) {
  emitter << YAML::BeginMap << YAML::Key << "frame_id" << YAML::Value << pose.frame_id << YAML::Key
          << "position" << YAML::Value;
  emitVector3(emitter, pose.position);
  emitter << YAML::Key << "orientation" << YAML::Value;
  emitQuaternion(emitter, pose.orientation);
  emitter << YAML::EndMap;
}

std::string_view geometryTypeYamlName(GeometryType type) {
  switch (type) {
  case GeometryType::kBox:
    return "box";
  case GeometryType::kCylinder:
    return "cylinder";
  case GeometryType::kSphere:
    return "sphere";
  case GeometryType::kMesh:
    return "mesh";
  }

  return "unknown";
}

bool parseGeometryType(std::string_view text, GeometryType& output) {
  if (text == "box") {
    output = GeometryType::kBox;
    return true;
  }
  if (text == "cylinder") {
    output = GeometryType::kCylinder;
    return true;
  }
  if (text == "sphere") {
    output = GeometryType::kSphere;
    return true;
  }
  if (text == "mesh") {
    output = GeometryType::kMesh;
    return true;
  }

  return false;
}

void checkGeometryKeys(const YAML::Node& node, std::string_view path, GeometryType type,
                       ValidationErrors& errors) {
  constexpr std::array<std::string_view, 2> kBoxKeys{"type", "size_m"};
  constexpr std::array<std::string_view, 3> kCylinderKeys{"type", "radius_m", "height_m"};
  constexpr std::array<std::string_view, 2> kSphereKeys{"type", "radius_m"};
  constexpr std::array<std::string_view, 2> kMeshKeys{"type", "mesh_uri"};

  switch (type) {
  case GeometryType::kBox:
    checkMappingKeys(node, path, kBoxKeys, errors);
    break;
  case GeometryType::kCylinder:
    checkMappingKeys(node, path, kCylinderKeys, errors);
    break;
  case GeometryType::kSphere:
    checkMappingKeys(node, path, kSphereKeys, errors);
    break;
  case GeometryType::kMesh:
    checkMappingKeys(node, path, kMeshKeys, errors);
    break;
  }
}

void readGeometry(const YAML::Node& node, std::string_view path, GeometryDefinition& output,
                  ValidationErrors& errors) {
  if (!requireMapping(node, path, errors)) {
    return;
  }

  std::string type_name;
  const bool has_type =
      readRequiredString(node, "type", type_name, std::string(path) + ".type", errors);
  const bool known_type = has_type && parseGeometryType(type_name, output.type);

  if (has_type && !known_type) {
    addError(errors, std::string(path) + ".type", "must be one of: box, cylinder, sphere, mesh");
  }

  if (!known_type) {
    constexpr std::array<std::string_view, 5> kAllGeometryKeys{"type", "size_m", "radius_m",
                                                               "height_m", "mesh_uri"};
    checkMappingKeys(node, path, kAllGeometryKeys, errors);
    return;
  }

  checkGeometryKeys(node, path, output.type, errors);
  switch (output.type) {
  case GeometryType::kBox: {
    const YAML::Node size = node["size_m"];
    if (!size) {
      addError(errors, std::string(path) + ".size_m", "missing required key");
    } else {
      readVector3(size, std::string(path) + ".size_m", output.size_m, errors);
    }
    break;
  }
  case GeometryType::kCylinder:
    static_cast<void>(readRequiredDouble(node, "radius_m", output.radius_m,
                                         std::string(path) + ".radius_m", errors));
    static_cast<void>(readRequiredDouble(node, "height_m", output.height_m,
                                         std::string(path) + ".height_m", errors));
    break;
  case GeometryType::kSphere:
    static_cast<void>(readRequiredDouble(node, "radius_m", output.radius_m,
                                         std::string(path) + ".radius_m", errors));
    break;
  case GeometryType::kMesh:
    static_cast<void>(readRequiredString(node, "mesh_uri", output.mesh_uri,
                                         std::string(path) + ".mesh_uri", errors));
    break;
  }
}

void emitGeometry(YAML::Emitter& emitter, const GeometryDefinition& geometry) {
  emitter << YAML::BeginMap << YAML::Key << "type" << YAML::Value
          << std::string(geometryTypeYamlName(geometry.type));

  switch (geometry.type) {
  case GeometryType::kBox:
    emitter << YAML::Key << "size_m" << YAML::Value;
    emitVector3(emitter, geometry.size_m);
    break;
  case GeometryType::kCylinder:
    emitter << YAML::Key << "radius_m" << YAML::Value << geometry.radius_m << YAML::Key
            << "height_m" << YAML::Value << geometry.height_m;
    break;
  case GeometryType::kSphere:
    emitter << YAML::Key << "radius_m" << YAML::Value << geometry.radius_m;
    break;
  case GeometryType::kMesh:
    emitter << YAML::Key << "mesh_uri" << YAML::Value << geometry.mesh_uri;
    break;
  }

  emitter << YAML::EndMap;
}

std::string_view cellEntityTypeYamlName(CellEntityType type) {
  switch (type) {
  case CellEntityType::kTable:
    return "table";
  case CellEntityType::kWorkpiece:
    return "workpiece";
  case CellEntityType::kFixture:
    return "fixture";
  case CellEntityType::kObstacle:
    return "obstacle";
  }
  return "unknown";
}

bool parseCellEntityType(std::string_view text, CellEntityType& output) {
  if (text == "table") {
    output = CellEntityType::kTable;
    return true;
  }
  if (text == "workpiece") {
    output = CellEntityType::kWorkpiece;
    return true;
  }
  if (text == "fixture") {
    output = CellEntityType::kFixture;
    return true;
  }
  if (text == "obstacle") {
    output = CellEntityType::kObstacle;
    return true;
  }
  return false;
}

void readCellEntity(const YAML::Node& node, std::string_view path, CellEntity& output,
                    ValidationErrors& errors) {
  if (!requireMapping(node, path, errors)) {
    return;
  }

  constexpr std::array<std::string_view, 4> kKeys{"id", "type", "pose", "geometry"};
  checkMappingKeys(node, path, kKeys, errors);
  static_cast<void>(readRequiredString(node, "id", output.id, std::string(path) + ".id", errors));

  std::string type_name;
  const bool has_type =
      readRequiredString(node, "type", type_name, std::string(path) + ".type", errors);
  if (has_type && !parseCellEntityType(type_name, output.type)) {
    addError(errors, std::string(path) + ".type",
             "must be one of: table, workpiece, fixture, obstacle");
  }

  const YAML::Node pose = node["pose"];
  if (!pose) {
    addError(errors, std::string(path) + ".pose", "missing required key");
  } else {
    readPose(pose, std::string(path) + ".pose", output.pose, errors);
  }

  const YAML::Node geometry = node["geometry"];
  if (!geometry) {
    addError(errors, std::string(path) + ".geometry", "missing required key");
  } else {
    readGeometry(geometry, std::string(path) + ".geometry", output.geometry, errors);
  }
}

void emitCellEntity(YAML::Emitter& emitter, const CellEntity& entity) {
  emitter << YAML::BeginMap << YAML::Key << "id" << YAML::Value << entity.id << YAML::Key << "type"
          << YAML::Value << std::string(cellEntityTypeYamlName(entity.type)) << YAML::Key << "pose"
          << YAML::Value;
  emitPose(emitter, entity.pose);
  emitter << YAML::Key << "geometry" << YAML::Value;
  emitGeometry(emitter, entity.geometry);
  emitter << YAML::EndMap;
}

void readJointLimits(const YAML::Node& node, std::string_view path,
                     std::vector<JointLimits>& output, ValidationErrors& errors) {
  if (!node || !node.IsSequence()) {
    addError(errors, std::string(path), "must be a sequence");
    return;
  }

  constexpr std::array<std::string_view, 3> kKeys{"joint_name", "minimum_position_rad",
                                                  "maximum_position_rad"};
  output.reserve(node.size());

  for (std::size_t index = 0; index < node.size(); ++index) {
    const YAML::Node entry = node[index];
    const std::string entry_path = std::string(path) + "[" + std::to_string(index) + "]";

    if (!requireMapping(entry, entry_path, errors)) {
      continue;
    }

    checkMappingKeys(entry, entry_path, kKeys, errors);

    JointLimits limits;
    static_cast<void>(readRequiredString(entry, "joint_name", limits.joint_name,
                                         entry_path + ".joint_name", errors));
    static_cast<void>(readRequiredDouble(entry, "minimum_position_rad", limits.minimum_position_rad,
                                         entry_path + ".minimum_position_rad", errors));
    static_cast<void>(readRequiredDouble(entry, "maximum_position_rad", limits.maximum_position_rad,
                                         entry_path + ".maximum_position_rad", errors));
    output.push_back(std::move(limits));
  }
}

void emitJointLimits(YAML::Emitter& emitter, const std::vector<JointLimits>& joint_limits) {
  emitter << YAML::BeginSeq;
  for (const auto& limits : joint_limits) {
    emitter << YAML::BeginMap << YAML::Key << "joint_name" << YAML::Value << limits.joint_name
            << YAML::Key << "minimum_position_rad" << YAML::Value << limits.minimum_position_rad
            << YAML::Key << "maximum_position_rad" << YAML::Value << limits.maximum_position_rad
            << YAML::EndMap;
  }
  emitter << YAML::EndSeq;
}

} // namespace

PoseYamlParseResult parsePoseYaml(std::string_view yaml_text) {
  YAML::Node root;
  try {
    root = YAML::Load(std::string(yaml_text));
  } catch (const YAML::Exception& error) {
    return {
        std::nullopt,
        {{"$", std::string("invalid YAML: ") + error.what()}},
    };
  }

  ValidationErrors errors;
  if (!requireMapping(root, "$", errors)) {
    return {std::nullopt, std::move(errors)};
  }

  Pose pose;
  readPose(root, "", pose, errors);

  if (errors.empty()) {
    errors = validate(pose);
  }

  if (!errors.empty()) {
    return {std::nullopt, std::move(errors)};
  }

  return {std::move(pose), {}};
}

GeometryYamlParseResult parseGeometryYaml(std::string_view yaml_text) {
  YAML::Node root;
  try {
    root = YAML::Load(std::string(yaml_text));
  } catch (const YAML::Exception& error) {
    return {
        std::nullopt,
        {{"$", std::string("invalid YAML: ") + error.what()}},
    };
  }

  ValidationErrors errors;
  if (!requireMapping(root, "$", errors)) {
    return {std::nullopt, std::move(errors)};
  }

  GeometryDefinition geometry;
  readGeometry(root, "", geometry, errors);

  if (errors.empty()) {
    errors = validate(geometry);
  }

  if (!errors.empty()) {
    return {std::nullopt, std::move(errors)};
  }

  return {std::move(geometry), {}};
}

RobotYamlParseResult parseRobotYaml(std::string_view yaml_text) {
  YAML::Node root;
  try {
    root = YAML::Load(std::string(yaml_text));
  } catch (const YAML::Exception& error) {
    return {
        std::nullopt,
        {{"$", std::string("invalid YAML: ") + error.what()}},
    };
  }

  ValidationErrors errors;
  if (!requireMapping(root, "$", errors)) {
    return {std::nullopt, std::move(errors)};
  }

  constexpr std::array<std::string_view, 7> kKeys{
      "id", "model", "planning_group", "model_frame", "base_frame", "flange_frame", "joint_limits"};
  checkMappingKeys(root, "", kKeys, errors);

  RobotDefinition robot;
  static_cast<void>(readRequiredString(root, "id", robot.id, ".id", errors));
  static_cast<void>(readRequiredString(root, "model", robot.model, ".model", errors));
  static_cast<void>(
      readRequiredString(root, "planning_group", robot.planning_group, ".planning_group", errors));
  static_cast<void>(
      readRequiredString(root, "model_frame", robot.model_frame, ".model_frame", errors));
  static_cast<void>(
      readRequiredString(root, "base_frame", robot.base_frame, ".base_frame", errors));
  static_cast<void>(
      readRequiredString(root, "flange_frame", robot.flange_frame, ".flange_frame", errors));

  const YAML::Node joint_limits = root["joint_limits"];
  if (!joint_limits) {
    addError(errors, ".joint_limits", "missing required key");
  } else {
    readJointLimits(joint_limits, ".joint_limits", robot.joint_limits, errors);
  }

  if (errors.empty()) {
    errors = validate(robot);
  }

  if (!errors.empty()) {
    return {std::nullopt, std::move(errors)};
  }

  return {std::move(robot), {}};
}

ToolYamlParseResult parseToolYaml(std::string_view yaml_text) {
  YAML::Node root;
  try {
    root = YAML::Load(std::string(yaml_text));
  } catch (const YAML::Exception& error) {
    return {
        std::nullopt,
        {{"$", std::string("invalid YAML: ") + error.what()}},
    };
  }

  ValidationErrors errors;
  if (!requireMapping(root, "$", errors)) {
    return {std::nullopt, std::move(errors)};
  }

  constexpr std::array<std::string_view, 5> kKeys{"id", "name", "mass_kg", "tcp", "geometry"};
  checkMappingKeys(root, "", kKeys, errors);

  ToolDefinition tool;
  static_cast<void>(readRequiredString(root, "id", tool.id, ".id", errors));
  static_cast<void>(readRequiredString(root, "name", tool.name, ".name", errors));
  static_cast<void>(readRequiredDouble(root, "mass_kg", tool.mass_kg, ".mass_kg", errors));

  const YAML::Node tcp = root["tcp"];
  if (!tcp) {
    addError(errors, ".tcp", "missing required key");
  } else {
    readPose(tcp, ".tcp", tool.tcp, errors);
  }

  const YAML::Node geometry = root["geometry"];
  if (!geometry) {
    addError(errors, ".geometry", "missing required key");
  } else if (!geometry.IsSequence()) {
    addError(errors, ".geometry", "must be a sequence");
  } else {
    tool.geometry.reserve(geometry.size());
    for (std::size_t index = 0; index < geometry.size(); ++index) {
      GeometryDefinition definition;
      readGeometry(geometry[index], ".geometry[" + std::to_string(index) + "]", definition, errors);
      tool.geometry.push_back(std::move(definition));
    }
  }

  if (errors.empty()) {
    errors = validate(tool);
  }

  if (!errors.empty()) {
    return {std::nullopt, std::move(errors)};
  }

  return {std::move(tool), {}};
}

CellYamlParseResult parseCellYaml(std::string_view yaml_text) {
  YAML::Node root;
  try {
    root = YAML::Load(std::string(yaml_text));
  } catch (const YAML::Exception& error) {
    return {std::nullopt, {{"$", std::string("invalid YAML: ") + error.what()}}};
  }

  ValidationErrors errors;
  if (!requireMapping(root, "$", errors)) {
    return {std::nullopt, std::move(errors)};
  }

  constexpr std::array<std::string_view, 4> kKeys{"id", "robot_base", "workpiece_pose", "entities"};
  checkMappingKeys(root, "", kKeys, errors);

  CellDefinition cell;
  static_cast<void>(readRequiredString(root, "id", cell.id, ".id", errors));

  const YAML::Node robot_base = root["robot_base"];
  if (!robot_base) {
    addError(errors, ".robot_base", "missing required key");
  } else {
    readPose(robot_base, ".robot_base", cell.robot_base, errors);
  }

  const YAML::Node workpiece_pose = root["workpiece_pose"];
  if (!workpiece_pose) {
    addError(errors, ".workpiece_pose", "missing required key");
  } else {
    readPose(workpiece_pose, ".workpiece_pose", cell.workpiece_pose, errors);
  }

  const YAML::Node entities = root["entities"];
  if (!entities) {
    addError(errors, ".entities", "missing required key");
  } else if (!entities.IsSequence()) {
    addError(errors, ".entities", "must be a sequence");
  } else {
    cell.entities.reserve(entities.size());
    for (std::size_t index = 0; index < entities.size(); ++index) {
      CellEntity entity;
      readCellEntity(entities[index], ".entities[" + std::to_string(index) + "]", entity, errors);
      cell.entities.push_back(std::move(entity));
    }
  }

  if (errors.empty()) {
    errors = validate(cell);
  }
  if (!errors.empty()) {
    return {std::nullopt, std::move(errors)};
  }
  return {std::move(cell), {}};
}

std::string serializePoseYaml(const Pose& pose) {
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(static_cast<std::size_t>(std::numeric_limits<double>::max_digits10));

  emitPose(emitter, pose);

  return emitter.c_str();
}

std::string serializeGeometryYaml(const GeometryDefinition& geometry) {
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(static_cast<std::size_t>(std::numeric_limits<double>::max_digits10));

  emitGeometry(emitter, geometry);
  return emitter.c_str();
}

std::string serializeRobotYaml(const RobotDefinition& robot) {
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(static_cast<std::size_t>(std::numeric_limits<double>::max_digits10));

  emitter << YAML::BeginMap << YAML::Key << "id" << YAML::Value << robot.id << YAML::Key << "model"
          << YAML::Value << robot.model << YAML::Key << "planning_group" << YAML::Value
          << robot.planning_group << YAML::Key << "model_frame" << YAML::Value << robot.model_frame
          << YAML::Key << "base_frame" << YAML::Value << robot.base_frame << YAML::Key
          << "flange_frame" << YAML::Value << robot.flange_frame << YAML::Key << "joint_limits"
          << YAML::Value;
  emitJointLimits(emitter, robot.joint_limits);
  emitter << YAML::EndMap;

  return emitter.c_str();
}

std::string serializeToolYaml(const ToolDefinition& tool) {
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(static_cast<std::size_t>(std::numeric_limits<double>::max_digits10));

  emitter << YAML::BeginMap << YAML::Key << "id" << YAML::Value << tool.id << YAML::Key << "name"
          << YAML::Value << tool.name << YAML::Key << "mass_kg" << YAML::Value << tool.mass_kg
          << YAML::Key << "tcp" << YAML::Value;
  emitPose(emitter, tool.tcp);
  emitter << YAML::Key << "geometry" << YAML::Value << YAML::BeginSeq;
  for (const auto& geometry : tool.geometry) {
    emitGeometry(emitter, geometry);
  }
  emitter << YAML::EndSeq << YAML::EndMap;

  return emitter.c_str();
}

std::string serializeCellYaml(const CellDefinition& cell) {
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(static_cast<std::size_t>(std::numeric_limits<double>::max_digits10));

  emitter << YAML::BeginMap << YAML::Key << "id" << YAML::Value << cell.id << YAML::Key
          << "robot_base" << YAML::Value;
  emitPose(emitter, cell.robot_base);
  emitter << YAML::Key << "workpiece_pose" << YAML::Value;
  emitPose(emitter, cell.workpiece_pose);
  emitter << YAML::Key << "entities" << YAML::Value << YAML::BeginSeq;
  for (const auto& entity : cell.entities) {
    emitCellEntity(emitter, entity);
  }
  emitter << YAML::EndSeq << YAML::EndMap;
  return emitter.c_str();
}

} // namespace rco_core::domain
