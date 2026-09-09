#pragma once

#include "rco_core/domain/types.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rco_core::domain {

// Domain schema objects are intentionally plain data structures for
// validation, serialization, and deterministic comparison.
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)

inline constexpr std::uint32_t kCurrentStudySchemaVersion = 1U;

enum class GeometryType : std::uint8_t {
  kBox,
  kCylinder,
  kSphere,
  kMesh,
};

enum class CellEntityType : std::uint8_t {
  kTable,
  kWorkpiece,
  kFixture,
  kObstacle,
};

enum class ProcessSegmentType : std::uint8_t {
  kProcess,
  kApproach,
  kRetract,
  kTransition,
};

struct GeometryDefinition {
  GeometryType type{GeometryType::kBox};

  // SI units only.
  Vector3 size_m;
  double radius_m{0.0};
  double height_m{0.0};

  std::string mesh_uri;

  bool operator==(const GeometryDefinition&) const = default;
};

struct RobotDefinition {
  std::string id;
  std::string model;
  std::string planning_group;
  std::string model_frame;
  std::string base_frame;
  std::string flange_frame;
  std::vector<JointLimits> joint_limits;

  bool operator==(const RobotDefinition&) const = default;
};

struct ToolDefinition {
  std::string id;
  std::string name;

  // kg
  double mass_kg{0.0};

  // Pose of the TCP relative to the configured parent frame.
  Pose tcp;

  std::vector<GeometryDefinition> geometry;

  bool operator==(const ToolDefinition&) const = default;
};

struct CellEntity {
  std::string id;
  CellEntityType type{CellEntityType::kObstacle};
  Pose pose;
  GeometryDefinition geometry;

  bool operator==(const CellEntity&) const = default;
};

struct CellDefinition {
  std::string id;

  Pose robot_base;
  Pose workpiece_pose;

  std::vector<CellEntity> entities;

  bool operator==(const CellDefinition&) const = default;
};

struct TargetPose {
  Pose pose;

  bool operator==(const TargetPose&) const = default;
};

struct ProcessSegment {
  ProcessSegmentType type{ProcessSegmentType::kProcess};
  std::vector<TargetPose> targets;

  // m/s for translational process motion.
  double tcp_speed_mps{0.0};

  bool operator==(const ProcessSegment&) const = default;
};

struct TaskDefinition {
  std::string id;
  std::vector<ProcessSegment> segments;

  bool operator==(const TaskDefinition&) const = default;
};

struct CandidateConfiguration {
  std::string robot_id;
  Pose robot_base;
  Pose workpiece_pose;

  bool operator==(const CandidateConfiguration&) const = default;
};

struct CandidateMetrics {
  bool feasible{false};

  double reachability_fraction{0.0};
  double minimum_clearance_m{0.0};
  double minimum_joint_margin{0.0};
  double minimum_singular_value{0.0};
  double worst_condition_number{0.0};
  double cycle_time_s{0.0};

  bool operator==(const CandidateMetrics&) const = default;
};

struct EvaluationResult {
  CandidateConfiguration candidate;
  CandidateMetrics metrics;
  std::vector<std::string> failure_reasons;

  bool operator==(const EvaluationResult&) const = default;
};

struct OptimizationProblem {
  std::size_t evaluation_budget{0U};
  std::uint64_t random_seed{0U};

  bool operator==(const OptimizationProblem&) const = default;
};

struct StudyDefinition {
  std::uint32_t schema_version{kCurrentStudySchemaVersion};

  std::string id;

  RobotDefinition robot;
  ToolDefinition tool;
  CellDefinition cell;
  TaskDefinition task;
  OptimizationProblem optimization;

  bool operator==(const StudyDefinition&) const = default;
};

// NOLINTEND(misc-non-private-member-variables-in-classes)

} // namespace rco_core::domain
