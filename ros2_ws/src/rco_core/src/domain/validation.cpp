#include "rco_core/domain/validation.hpp"

#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"

#include <cmath>
#include <cstddef>
#include <set>
#include <string>
#include <utility>

namespace rco_core::domain {
namespace {

constexpr double kQuaternionNormTolerance = 1e-6;

bool isFinite(double value) {
  return std::isfinite(value);
}

bool isFinite(const Vector3& value) {
  return isFinite(value.x) && isFinite(value.y) && isFinite(value.z);
}

bool isFinite(const Quaternion& value) {
  return isFinite(value.x) && isFinite(value.y) && isFinite(value.z) && isFinite(value.w);
}

bool isStrictlyPositive(const Vector3& value) {
  return value.x > 0.0 && value.y > 0.0 && value.z > 0.0;
}

double quaternionNorm(const Quaternion& quaternion) {
  return std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y +
                   quaternion.z * quaternion.z + quaternion.w * quaternion.w);
}

void appendErrors(ValidationErrors& destination, ValidationErrors source,
                  const std::string& prefix) {
  for (auto& error : source) {
    error.path = prefix + error.path;
    destination.push_back(std::move(error));
  }
}

} // namespace

ValidationErrors validate(const Pose& pose) {
  ValidationErrors errors;

  if (pose.frame_id.empty()) {
    errors.push_back({
        ".frame_id",
        "must not be empty",
    });
  }

  if (!isFinite(pose.position)) {
    errors.push_back({
        ".position",
        "must contain only finite SI values",
    });
  }

  if (!isFinite(pose.orientation)) {
    errors.push_back({
        ".orientation",
        "must contain only finite values",
    });

    return errors;
  }

  const double norm = quaternionNorm(pose.orientation);

  if (std::abs(norm - 1.0) > kQuaternionNormTolerance) {
    errors.push_back({
        ".orientation",
        "quaternion must be normalized",
    });
  }

  return errors;
}

ValidationErrors validate(const GeometryDefinition& geometry) {
  ValidationErrors errors;

  switch (geometry.type) {
  case GeometryType::kBox:
    if (!isFinite(geometry.size_m)) {
      errors.push_back({".size_m", "box dimensions must contain only finite SI values"});
    } else if (!isStrictlyPositive(geometry.size_m)) {
      errors.push_back({".size_m", "box dimensions must be greater than zero"});
    }
    break;

  case GeometryType::kCylinder:
    if (!isFinite(geometry.radius_m) || geometry.radius_m <= 0.0) {
      errors.push_back({".radius_m", "cylinder radius must be finite and greater than zero"});
    }
    if (!isFinite(geometry.height_m) || geometry.height_m <= 0.0) {
      errors.push_back({".height_m", "cylinder height must be finite and greater than zero"});
    }
    break;

  case GeometryType::kSphere:
    if (!isFinite(geometry.radius_m) || geometry.radius_m <= 0.0) {
      errors.push_back({".radius_m", "sphere radius must be finite and greater than zero"});
    }
    break;

  case GeometryType::kMesh:
    if (geometry.mesh_uri.empty()) {
      errors.push_back({".mesh_uri", "mesh URI must not be empty"});
    }
    break;

  default:
    errors.push_back({".type", "unsupported geometry type"});
    break;
  }

  return errors;
}

ValidationErrors validate(const RobotDefinition& robot) {
  ValidationErrors errors;

  if (robot.id.empty()) {
    errors.push_back({".id", "must not be empty"});
  }

  if (robot.model.empty()) {
    errors.push_back({".model", "must not be empty"});
  }

  if (robot.planning_group.empty()) {
    errors.push_back({
        ".planning_group",
        "must not be empty",
    });
  }

  if (robot.model_frame.empty()) {
    errors.push_back({
        ".model_frame",
        "must not be empty",
    });
  }

  if (robot.base_frame.empty()) {
    errors.push_back({
        ".base_frame",
        "must not be empty",
    });
  }

  if (robot.flange_frame.empty()) {
    errors.push_back({
        ".flange_frame",
        "must not be empty",
    });
  }

  if (robot.joint_limits.empty()) {
    errors.push_back({
        ".joint_limits",
        "must contain at least one joint",
    });
  }

  std::set<std::string> joint_names;
  for (std::size_t index = 0; index < robot.joint_limits.size(); ++index) {
    const auto& limits = robot.joint_limits[index];
    const std::string path = ".joint_limits[" + std::to_string(index) + "]";

    if (limits.joint_name.empty()) {
      errors.push_back({
          path + ".joint_name",
          "must not be empty",
      });
    } else if (!joint_names.insert(limits.joint_name).second) {
      errors.push_back({
          path + ".joint_name",
          "must be unique",
      });
    }

    if (!isFinite(limits.minimum_position_rad)) {
      errors.push_back({
          path + ".minimum_position_rad",
          "must be a finite radian value",
      });
    }

    if (!isFinite(limits.maximum_position_rad)) {
      errors.push_back({
          path + ".maximum_position_rad",
          "must be a finite radian value",
      });
    }

    if (isFinite(limits.minimum_position_rad) && isFinite(limits.maximum_position_rad) &&
        limits.minimum_position_rad >= limits.maximum_position_rad) {
      errors.push_back({
          path,
          "minimum position must be less than maximum position",
      });
    }
  }

  return errors;
}

ValidationErrors validate(const ToolDefinition& tool) {
  ValidationErrors errors;

  if (tool.id.empty()) {
    errors.push_back({".id", "must not be empty"});
  }

  if (tool.name.empty()) {
    errors.push_back({".name", "must not be empty"});
  }

  if (!isFinite(tool.mass_kg) || tool.mass_kg < 0.0) {
    errors.push_back({
        ".mass_kg",
        "must be a finite non-negative SI value",
    });
  }

  appendErrors(errors, validate(tool.tcp), ".tcp");

  for (std::size_t index = 0; index < tool.geometry.size(); ++index) {
    appendErrors(errors, validate(tool.geometry[index]),
                 ".geometry[" + std::to_string(index) + "]");
  }

  return errors;
}

ValidationErrors validate(const CellEntity& entity) {
  ValidationErrors errors;

  if (entity.id.empty()) {
    errors.push_back({".id", "must not be empty"});
  }

  switch (entity.type) {
  case CellEntityType::kTable:
  case CellEntityType::kWorkpiece:
  case CellEntityType::kFixture:
  case CellEntityType::kObstacle:
    break;
  default:
    errors.push_back({".type", "unsupported cell entity type"});
    break;
  }

  appendErrors(errors, validate(entity.pose), ".pose");
  appendErrors(errors, validate(entity.geometry), ".geometry");
  return errors;
}

ValidationErrors validate(const CellDefinition& cell) {
  ValidationErrors errors;

  if (cell.id.empty()) {
    errors.push_back({".id", "must not be empty"});
  }

  appendErrors(errors, validate(cell.robot_base), ".robot_base");

  appendErrors(errors, validate(cell.workpiece_pose), ".workpiece_pose");

  std::set<std::string> entity_ids;
  for (std::size_t index = 0; index < cell.entities.size(); ++index) {
    const auto& entity = cell.entities[index];
    const std::string path = ".entities[" + std::to_string(index) + "]";
    appendErrors(errors, validate(entity), path);

    if (!entity.id.empty() && !entity_ids.insert(entity.id).second) {
      errors.push_back({path + ".id", "must be unique"});
    }
  }

  return errors;
}

ValidationErrors validate(const TaskDefinition& task) {
  ValidationErrors errors;

  if (task.id.empty()) {
    errors.push_back({".id", "must not be empty"});
  }

  for (std::size_t segment_index = 0; segment_index < task.segments.size(); ++segment_index) {
    const auto& segment = task.segments[segment_index];

    for (std::size_t target_index = 0; target_index < segment.targets.size(); ++target_index) {
      appendErrors(errors, validate(segment.targets[target_index].pose),
                   ".segments[" + std::to_string(segment_index) + "].targets[" +
                       std::to_string(target_index) + "].pose");
    }
  }

  return errors;
}

ValidationErrors validate(const StudyDefinition& study) {
  ValidationErrors errors;

  if (study.schema_version != kCurrentStudySchemaVersion) {
    errors.push_back({
        ".schema_version",
        "unsupported study schema version",
    });
  }

  if (study.id.empty()) {
    errors.push_back({".id", "must not be empty"});
  }

  appendErrors(errors, validate(study.robot), ".robot");

  appendErrors(errors, validate(study.tool), ".tool");

  appendErrors(errors, validate(study.cell), ".cell");

  appendErrors(errors, validate(study.task), ".task");

  return errors;
}

} // namespace rco_core::domain
