#include "rco_core/domain/model.hpp"
#include "rco_core/domain/serialization.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/domain/validation.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <gtest/gtest.h>
#include <optional>
#include <string>

namespace {

using rco_core::domain::CellDefinition;
using rco_core::domain::CellEntity;
using rco_core::domain::CellEntityType;
using rco_core::domain::CellYamlParseResult;
using rco_core::domain::GeometryDefinition;
using rco_core::domain::GeometryType;
using rco_core::domain::GeometryYamlParseResult;
using rco_core::domain::JointLimits;
using rco_core::domain::OptimizationProblem;
using rco_core::domain::OptimizationYamlParseResult;
using rco_core::domain::Pose;
using rco_core::domain::PoseYamlParseResult;
using rco_core::domain::ProcessSegment;
using rco_core::domain::ProcessSegmentType;
using rco_core::domain::RobotDefinition;
using rco_core::domain::RobotYamlParseResult;
using rco_core::domain::StudyDefinition;
using rco_core::domain::StudyYamlParseResult;
using rco_core::domain::TargetPose;
using rco_core::domain::TaskDefinition;
using rco_core::domain::TaskYamlParseResult;
using rco_core::domain::ToolDefinition;
using rco_core::domain::ToolYamlParseResult;
using rco_core::domain::ValidationErrors;

Pose makePose() {
  Pose pose;
  pose.frame_id = "world";
  pose.position = {1.25, -0.5, 0.125};
  pose.orientation = {0.5, -0.5, 0.5, -0.5};
  return pose;
}

RobotDefinition makeRobot() {
  RobotDefinition robot;
  robot.id = "robot_1";
  robot.model = "ur5e";
  robot.planning_group = "ur_manipulator";
  robot.model_frame = "world";
  robot.base_frame = "base_link";
  robot.flange_frame = "tool0";
  robot.joint_limits = {
      JointLimits{"shoulder_pan_joint", -6.25, 6.25},
      JointLimits{"shoulder_lift_joint", -3.0, 3.0},
  };
  return robot;
}

ToolDefinition makeTool() {
  ToolDefinition tool;
  tool.id = "tool_1";
  tool.name = "welding_torch";
  tool.mass_kg = 2.5;
  tool.tcp = makePose();

  GeometryDefinition cylinder;
  cylinder.type = GeometryType::kCylinder;
  cylinder.radius_m = 0.04;
  cylinder.height_m = 0.3;
  tool.geometry.push_back(cylinder);
  return tool;
}

CellDefinition makeCell() {
  CellDefinition cell;
  cell.id = "cell_1";
  cell.robot_base = makePose();
  cell.workpiece_pose = makePose();

  CellEntity fixture;
  fixture.id = "fixture_1";
  fixture.type = CellEntityType::kFixture;
  fixture.pose = makePose();
  fixture.geometry.type = GeometryType::kBox;
  fixture.geometry.size_m = {0.5, 0.25, 0.1};
  cell.entities.push_back(fixture);
  return cell;
}

TaskDefinition makeTask() {
  TaskDefinition task;
  task.id = "task_1";

  const std::array types{ProcessSegmentType::kProcess, ProcessSegmentType::kApproach,
                         ProcessSegmentType::kRetract, ProcessSegmentType::kTransition};
  for (const auto type : types) {
    ProcessSegment segment;
    segment.type = type;
    segment.targets.push_back(TargetPose{makePose()});
    segment.tcp_speed_mps = 0.25;
    task.segments.push_back(segment);
  }
  return task;
}

OptimizationProblem makeOptimization() {
  OptimizationProblem optimization;
  optimization.evaluation_budget = 2500U;
  optimization.random_seed = 42U;
  return optimization;
}

StudyDefinition makeStudy() {
  StudyDefinition study;
  study.id = "study_1";
  study.robot = makeRobot();
  study.tool = makeTool();
  study.cell = makeCell();
  study.task = makeTask();
  study.optimization = makeOptimization();
  return study;
}

bool containsError(const ValidationErrors& errors, const std::string& path,
                   const std::string& message) {
  return std::ranges::any_of(errors, [&path, &message](const auto& error) {
    return error.path == path && error.message == message;
  });
}

TEST(DomainSerializationTest, RoundTripsPoseExactly) {
  const Pose pose = makePose();

  const std::string yaml = rco_core::domain::serializePoseYaml(pose);
  const PoseYamlParseResult result = rco_core::domain::parsePoseYaml(yaml);

  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value, std::optional<Pose>{pose});
  EXPECT_TRUE(result.errors.empty());
}

TEST(DomainSerializationTest, EmitsKeysInDeterministicSchemaOrder) {
  const std::string yaml = rco_core::domain::serializePoseYaml(makePose());

  EXPECT_EQ(yaml, R"(frame_id: world
position:
  x: 1.25
  y: -0.5
  z: 0.125
orientation:
  x: 0.5
  y: -0.5
  z: 0.5
  w: -0.5)");
}

TEST(DomainSerializationTest, RejectsMalformedYaml) {
  const PoseYamlParseResult result = rco_core::domain::parsePoseYaml("frame_id: [world");

  EXPECT_FALSE(result.ok());
  ASSERT_FALSE(result.errors.empty());
  EXPECT_EQ(result.errors.front().path, "$");
}

TEST(DomainSerializationTest, ReportsMissingRequiredFieldPath) {
  const PoseYamlParseResult result = rco_core::domain::parsePoseYaml(R"(frame_id: world
position:
  x: 0
  y: 0
  z: 0
orientation:
  x: 0
  y: 0
  z: 0)");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".orientation.w", "missing required key"));
}

TEST(DomainSerializationTest, ReportsWrongScalarTypePath) {
  const PoseYamlParseResult result = rco_core::domain::parsePoseYaml(R"(frame_id: world
position:
  x: zero
  y: 0
  z: 0
orientation:
  x: 0
  y: 0
  z: 0
  w: 1)");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".position.x", "must be a number"));
}

TEST(DomainSerializationTest, RejectsUnknownFieldWithExactPath) {
  const PoseYamlParseResult result = rco_core::domain::parsePoseYaml(R"(frame_id: world
position:
  x: 0
  y: 0
  z: 0
  metres: true
orientation:
  x: 0
  y: 0
  z: 0
  w: 1)");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".position.metres", "unknown key"));
}

TEST(DomainSerializationTest, AppliesPoseValidationAfterParsing) {
  const PoseYamlParseResult result = rco_core::domain::parsePoseYaml(R"(frame_id: world
position:
  x: 0
  y: 0
  z: 0
orientation:
  x: 0
  y: 0
  z: 0
  w: 2)");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".orientation", "quaternion must be normalized"));
}

TEST(DomainSerializationTest, RoundTripsEveryGeometryVariantExactly) {
  GeometryDefinition box;
  box.type = GeometryType::kBox;
  box.size_m = {1.25, 0.5, 0.125};

  GeometryDefinition cylinder;
  cylinder.type = GeometryType::kCylinder;
  cylinder.radius_m = 0.25;
  cylinder.height_m = 1.5;

  GeometryDefinition sphere;
  sphere.type = GeometryType::kSphere;
  sphere.radius_m = 0.75;

  GeometryDefinition mesh;
  mesh.type = GeometryType::kMesh;
  mesh.mesh_uri = "package://rco_description/meshes/fixture.stl";

  const std::array geometries{box, cylinder, sphere, mesh};
  for (const auto& geometry : geometries) {
    const std::string yaml = rco_core::domain::serializeGeometryYaml(geometry);
    const GeometryYamlParseResult result = rco_core::domain::parseGeometryYaml(yaml);

    EXPECT_TRUE(result.ok());
    EXPECT_EQ(result.value, std::optional<GeometryDefinition>{geometry});
    EXPECT_TRUE(result.errors.empty());
  }
}

TEST(DomainSerializationTest, EmitsGeometryKeysInDeterministicSchemaOrder) {
  GeometryDefinition geometry;
  geometry.type = GeometryType::kCylinder;
  geometry.radius_m = 0.125;
  geometry.height_m = 0.5;

  const std::string yaml = rco_core::domain::serializeGeometryYaml(geometry);

  EXPECT_EQ(yaml, R"(type: cylinder
radius_m: 0.125
height_m: 0.5)");
}

TEST(DomainSerializationTest, RejectsUnknownGeometryType) {
  const GeometryYamlParseResult result =
      rco_core::domain::parseGeometryYaml("type: capsule\nradius_m: 0.5");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".type", "must be one of: box, cylinder, sphere, mesh"));
}

TEST(DomainSerializationTest, ReportsMissingGeometryFieldPath) {
  const GeometryYamlParseResult result =
      rco_core::domain::parseGeometryYaml("type: cylinder\nradius_m: 0.5");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".height_m", "missing required key"));
}

TEST(DomainSerializationTest, RejectsFieldFromAnotherGeometryVariant) {
  const GeometryYamlParseResult result =
      rco_core::domain::parseGeometryYaml("type: sphere\nradius_m: 0.5\nheight_m: 1.0");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".height_m", "unknown key"));
}

TEST(DomainSerializationTest, AppliesGeometryValidationAfterParsing) {
  const GeometryYamlParseResult result =
      rco_core::domain::parseGeometryYaml("type: sphere\nradius_m: -0.5");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".radius_m",
                            "sphere radius must be finite and greater than zero"));
}

TEST(DomainSerializationTest, RoundTripsRobotAndJointLimitsExactly) {
  const RobotDefinition robot = makeRobot();

  const std::string yaml = rco_core::domain::serializeRobotYaml(robot);
  const RobotYamlParseResult result = rco_core::domain::parseRobotYaml(yaml);

  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.value, std::optional<RobotDefinition>{robot});
  EXPECT_TRUE(result.errors.empty());
}

TEST(DomainSerializationTest, EmitsRobotKeysInDeterministicSchemaOrder) {
  RobotDefinition robot = makeRobot();
  robot.joint_limits.resize(1);

  const std::string yaml = rco_core::domain::serializeRobotYaml(robot);

  EXPECT_EQ(yaml, R"(id: robot_1
model: ur5e
planning_group: ur_manipulator
model_frame: world
base_frame: base_link
flange_frame: tool0
joint_limits:
  - joint_name: shoulder_pan_joint
    minimum_position_rad: -6.25
    maximum_position_rad: 6.25)");
}

TEST(DomainSerializationTest, ReportsIndexedUnknownJointLimitField) {
  const RobotYamlParseResult result = rco_core::domain::parseRobotYaml(R"(id: robot_1
model: ur5e
planning_group: ur_manipulator
model_frame: world
base_frame: base_link
flange_frame: tool0
joint_limits:
  - joint_name: shoulder_pan_joint
    minimum_position_rad: -6.25
    maximum_position_rad: 6.25
    velocity_rad_s: 1.0)");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".joint_limits[0].velocity_rad_s", "unknown key"));
}

TEST(DomainSerializationTest, RejectsNonSequenceJointLimits) {
  const RobotYamlParseResult result = rco_core::domain::parseRobotYaml(R"(id: robot_1
model: ur5e
planning_group: ur_manipulator
model_frame: world
base_frame: base_link
flange_frame: tool0
joint_limits: {})");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".joint_limits", "must be a sequence"));
}

TEST(DomainSerializationTest, AppliesRobotValidationAfterParsing) {
  const RobotYamlParseResult result = rco_core::domain::parseRobotYaml(R"(id: robot_1
model: ur5e
planning_group: ur_manipulator
model_frame: world
base_frame: base_link
flange_frame: tool0
joint_limits:
  - joint_name: shoulder_pan_joint
    minimum_position_rad: -6.25
    maximum_position_rad: 6.25
  - joint_name: shoulder_pan_joint
    minimum_position_rad: -3.0
    maximum_position_rad: 3.0)");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".joint_limits[1].joint_name", "must be unique"));
}

TEST(DomainSerializationTest, RoundTripsToolWithNestedPoseAndGeometry) {
  const ToolDefinition tool = makeTool();

  const std::string yaml = rco_core::domain::serializeToolYaml(tool);
  const ToolYamlParseResult result = rco_core::domain::parseToolYaml(yaml);

  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.value, std::optional<ToolDefinition>{tool});
  EXPECT_TRUE(result.errors.empty());
}

TEST(DomainSerializationTest, ReportsNestedToolPoseErrorPath) {
  std::string yaml = rco_core::domain::serializeToolYaml(makeTool());
  const std::string needle = "    w: -0.5\n";
  yaml.erase(yaml.find(needle), needle.size());

  const ToolYamlParseResult result = rco_core::domain::parseToolYaml(yaml);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".tcp.orientation.w", "missing required key"));
}

TEST(DomainSerializationTest, ReportsNestedToolGeometryValidationPath) {
  auto tool = makeTool();
  tool.geometry.front().radius_m = -0.04;

  const std::string yaml = rco_core::domain::serializeToolYaml(tool);
  const ToolYamlParseResult result = rco_core::domain::parseToolYaml(yaml);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".geometry[0].radius_m",
                            "cylinder radius must be finite and greater than zero"));
}

TEST(DomainSerializationTest, RejectsNonSequenceToolGeometry) {
  std::string yaml = rco_core::domain::serializeToolYaml(makeTool());
  const std::size_t geometry_position = yaml.find("geometry:");
  yaml.replace(geometry_position, yaml.size() - geometry_position, "geometry: {}");

  const ToolYamlParseResult result = rco_core::domain::parseToolYaml(yaml);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".geometry", "must be a sequence"));
}

TEST(DomainSerializationTest, RoundTripsCellWithNestedEntities) {
  const CellDefinition cell = makeCell();

  const std::string yaml = rco_core::domain::serializeCellYaml(cell);
  const CellYamlParseResult result = rco_core::domain::parseCellYaml(yaml);

  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.value, std::optional<CellDefinition>{cell});
  EXPECT_TRUE(result.errors.empty());
  EXPECT_NE(yaml.find("type: fixture"), std::string::npos);
}

TEST(DomainSerializationTest, RejectsUnknownCellEntityTypeWithIndexedPath) {
  std::string yaml = rco_core::domain::serializeCellYaml(makeCell());
  const std::size_t type_position = yaml.find("type: fixture");
  yaml.replace(type_position, std::string("type: fixture").size(), "type: wall");

  const CellYamlParseResult result = rco_core::domain::parseCellYaml(yaml);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".entities[0].type",
                            "must be one of: table, workpiece, fixture, obstacle"));
}

TEST(DomainSerializationTest, ReportsNestedCellEntityGeometryPath) {
  auto cell = makeCell();
  cell.entities.front().geometry.size_m.z = 0.0;

  const CellYamlParseResult result =
      rco_core::domain::parseCellYaml(rco_core::domain::serializeCellYaml(cell));

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".entities[0].geometry.size_m",
                            "box dimensions must be greater than zero"));
}

TEST(DomainSerializationTest, RejectsNonSequenceCellEntities) {
  std::string yaml = rco_core::domain::serializeCellYaml(makeCell());
  const std::size_t entities_position = yaml.find("entities:");
  yaml.replace(entities_position, yaml.size() - entities_position, "entities: {}");

  const CellYamlParseResult result = rco_core::domain::parseCellYaml(yaml);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".entities", "must be a sequence"));
}

TEST(DomainSerializationTest, RoundTripsTaskWithEverySegmentType) {
  const TaskDefinition task = makeTask();

  const std::string yaml = rco_core::domain::serializeTaskYaml(task);
  const TaskYamlParseResult result = rco_core::domain::parseTaskYaml(yaml);

  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.value, std::optional<TaskDefinition>{task});
  EXPECT_TRUE(result.errors.empty());
  EXPECT_NE(yaml.find("type: process"), std::string::npos);
  EXPECT_NE(yaml.find("type: approach"), std::string::npos);
  EXPECT_NE(yaml.find("type: retract"), std::string::npos);
  EXPECT_NE(yaml.find("type: transition"), std::string::npos);
}

TEST(DomainSerializationTest, EmitsTaskKeysInDeterministicSchemaOrder) {
  auto task = makeTask();
  task.segments.resize(1);

  const std::string yaml = rco_core::domain::serializeTaskYaml(task);

  EXPECT_EQ(yaml, R"(id: task_1
segments:
  - type: process
    targets:
      - pose:
          frame_id: world
          position:
            x: 1.25
            y: -0.5
            z: 0.125
          orientation:
            x: 0.5
            y: -0.5
            z: 0.5
            w: -0.5
    tcp_speed_mps: 0.25)");
}

TEST(DomainSerializationTest, RejectsUnknownProcessSegmentTypeWithIndexedPath) {
  std::string yaml = rco_core::domain::serializeTaskYaml(makeTask());
  const std::size_t type_position = yaml.find("type: process");
  yaml.replace(type_position, std::string("type: process").size(), "type: dwell");

  const TaskYamlParseResult result = rco_core::domain::parseTaskYaml(yaml);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".segments[0].type",
                            "must be one of: process, approach, retract, transition"));
}

TEST(DomainSerializationTest, RejectsNonSequenceTaskSegments) {
  const TaskYamlParseResult result = rco_core::domain::parseTaskYaml("id: task_1\nsegments: {}");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".segments", "must be a sequence"));
}

TEST(DomainSerializationTest, RejectsNonSequenceSegmentTargets) {
  std::string yaml = rco_core::domain::serializeTaskYaml(makeTask());
  const std::size_t targets_position = yaml.find("targets:");
  const std::size_t speed_position = yaml.find("    tcp_speed_mps:", targets_position);
  yaml.replace(targets_position, speed_position - targets_position, "targets: {}\n");

  const TaskYamlParseResult result = rco_core::domain::parseTaskYaml(yaml);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".segments[0].targets", "must be a sequence"));
}

TEST(DomainSerializationTest, AppliesSegmentValidationAfterParsing) {
  auto task = makeTask();
  task.segments.front().tcp_speed_mps = 0.0;

  const TaskYamlParseResult result =
      rco_core::domain::parseTaskYaml(rco_core::domain::serializeTaskYaml(task));

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".segments[0].tcp_speed_mps",
                            "must be a finite SI value greater than zero"));
}

TEST(DomainSerializationTest, ReportsNestedTaskTargetPosePath) {
  auto task = makeTask();
  task.segments.front().targets.front().pose.frame_id.clear();

  const TaskYamlParseResult result =
      rco_core::domain::parseTaskYaml(rco_core::domain::serializeTaskYaml(task));

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(
      containsError(result.errors, ".segments[0].targets[0].pose.frame_id", "must not be empty"));
}

TEST(DomainSerializationTest, RoundTripsOptimizationExactly) {
  const OptimizationProblem optimization = makeOptimization();

  const std::string yaml = rco_core::domain::serializeOptimizationYaml(optimization);
  const OptimizationYamlParseResult result = rco_core::domain::parseOptimizationYaml(yaml);

  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.value, std::optional<OptimizationProblem>{optimization});
  EXPECT_TRUE(result.errors.empty());
}

TEST(DomainSerializationTest, EmitsOptimizationKeysInDeterministicSchemaOrder) {
  const std::string yaml = rco_core::domain::serializeOptimizationYaml(makeOptimization());

  EXPECT_EQ(yaml, R"(evaluation_budget: 2500
random_seed: 42)");
}

TEST(DomainSerializationTest, RejectsNonIntegerOptimizationValue) {
  const OptimizationYamlParseResult result =
      rco_core::domain::parseOptimizationYaml("evaluation_budget: many\nrandom_seed: 42");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".evaluation_budget", "must be a non-negative integer"));
}

TEST(DomainSerializationTest, RejectsNegativeRandomSeed) {
  const OptimizationYamlParseResult result =
      rco_core::domain::parseOptimizationYaml("evaluation_budget: 2500\nrandom_seed: -1");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".random_seed", "must be a non-negative integer"));
}

TEST(DomainSerializationTest, AppliesOptimizationValidationAfterParsing) {
  const OptimizationYamlParseResult result =
      rco_core::domain::parseOptimizationYaml("evaluation_budget: 0\nrandom_seed: 42");

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".evaluation_budget", "must be greater than zero"));
}

TEST(DomainSerializationTest, RoundTripsCompleteStudyExactly) {
  const StudyDefinition study = makeStudy();

  const std::string yaml = rco_core::domain::serializeStudyYaml(study);
  const StudyYamlParseResult result = rco_core::domain::parseStudyYaml(yaml);

  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.value, std::optional<StudyDefinition>{study});
  EXPECT_TRUE(result.errors.empty());
}

TEST(DomainSerializationTest, EmitsStudySectionsInDeterministicSchemaOrder) {
  const std::string yaml = rco_core::domain::serializeStudyYaml(makeStudy());

  const std::size_t schema_position = yaml.find("schema_version:");
  const std::size_t id_position = yaml.find("id:");
  const std::size_t robot_position = yaml.find("robot:");
  const std::size_t tool_position = yaml.find("tool:");
  const std::size_t cell_position = yaml.find("cell:");
  const std::size_t task_position = yaml.find("task:");
  const std::size_t optimization_position = yaml.find("optimization:");

  EXPECT_LT(schema_position, id_position);
  EXPECT_LT(id_position, robot_position);
  EXPECT_LT(robot_position, tool_position);
  EXPECT_LT(tool_position, cell_position);
  EXPECT_LT(cell_position, task_position);
  EXPECT_LT(task_position, optimization_position);
}

TEST(DomainSerializationTest, PrefixesNestedStudyErrors) {
  std::string yaml = rco_core::domain::serializeStudyYaml(makeStudy());
  const std::string needle = "  model: ur5e\n";
  yaml.insert(yaml.find(needle) + needle.size(), "  legacy_name: old\n");

  const StudyYamlParseResult result = rco_core::domain::parseStudyYaml(yaml);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".robot.legacy_name", "unknown key"));
}

TEST(DomainSerializationTest, RejectsNonMappingStudySectionWithExactPath) {
  std::string yaml = rco_core::domain::serializeStudyYaml(makeStudy());
  const std::size_t robot_position = yaml.find("robot:");
  const std::size_t tool_position = yaml.find("tool:", robot_position);
  yaml.replace(robot_position, tool_position - robot_position, "robot: invalid\n");

  const StudyYamlParseResult result = rco_core::domain::parseStudyYaml(yaml);

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".robot", "must be a mapping"));
}

TEST(DomainSerializationTest, AppliesStudySchemaVersionValidation) {
  auto study = makeStudy();
  study.schema_version = 999U;

  const StudyYamlParseResult result =
      rco_core::domain::parseStudyYaml(rco_core::domain::serializeStudyYaml(study));

  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(containsError(result.errors, ".schema_version", "unsupported study schema version"));
}

} // namespace
