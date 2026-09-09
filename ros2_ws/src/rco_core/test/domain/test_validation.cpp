#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/domain/validation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <string>

namespace {

using rco_core::domain::CellDefinition;
using rco_core::domain::CellEntity;
using rco_core::domain::CellEntityType;
using rco_core::domain::GeometryDefinition;
using rco_core::domain::GeometryType;
using rco_core::domain::JointLimits;
using rco_core::domain::OptimizationProblem;
using rco_core::domain::Pose;
using rco_core::domain::ProcessSegment;
using rco_core::domain::ProcessSegmentType;
using rco_core::domain::RobotDefinition;
using rco_core::domain::StudyDefinition;
using rco_core::domain::TargetPose;
using rco_core::domain::TaskDefinition;
using rco_core::domain::ToolDefinition;
using rco_core::domain::ValidationErrors;

Pose makeValidPose(const std::string& frame_id) {
  Pose pose;
  pose.frame_id = frame_id;
  pose.orientation.w = 1.0;
  return pose;
}

RobotDefinition makeValidRobot() {
  RobotDefinition robot;
  robot.id = "robot_1";
  robot.model = "ur5e";
  robot.planning_group = "ur_manipulator";
  robot.model_frame = "world";
  robot.base_frame = "base_link";
  robot.flange_frame = "tool0";

  robot.joint_limits.push_back({
      "shoulder_pan_joint",
      -6.283185307179586,
      6.283185307179586,
  });

  return robot;
}

ToolDefinition makeValidTool() {
  ToolDefinition tool;
  tool.id = "tool_1";
  tool.name = "reference_tool";
  tool.mass_kg = 1.0;
  tool.tcp = makeValidPose("tool0");
  return tool;
}

CellDefinition makeValidCell() {
  CellDefinition cell;
  cell.id = "cell_1";
  cell.robot_base = makeValidPose("world");
  cell.workpiece_pose = makeValidPose("world");
  return cell;
}

TaskDefinition makeValidTask() {
  TaskDefinition task;
  task.id = "task_1";
  ProcessSegment segment;
  segment.type = ProcessSegmentType::kProcess;
  segment.targets.push_back(TargetPose{makeValidPose("world")});
  segment.tcp_speed_mps = 0.25;
  task.segments.push_back(segment);
  return task;
}

StudyDefinition makeValidStudy() {
  StudyDefinition study;
  study.id = "study_1";
  study.robot = makeValidRobot();
  study.tool = makeValidTool();
  study.cell = makeValidCell();
  study.task = makeValidTask();
  study.optimization.evaluation_budget = 100U;
  study.optimization.random_seed = 42U;
  return study;
}

bool containsPath(const ValidationErrors& errors, const std::string& expected_path) {
  return std::ranges::any_of(
      errors, [&expected_path](const auto& error) { return error.path == expected_path; });
}

TEST(DomainValidationTest, AcceptsValidStudy) {
  const auto study = makeValidStudy();

  const auto errors = rco_core::domain::validate(study);

  EXPECT_TRUE(errors.empty());
}

TEST(DomainValidationTest, RejectsUnsupportedSchemaVersion) {
  auto study = makeValidStudy();
  study.schema_version = 999U;

  const auto errors = rco_core::domain::validate(study);

  EXPECT_TRUE(containsPath(errors, ".schema_version"));
}

TEST(DomainValidationTest, RejectsEmptyStudyIdentifier) {
  auto study = makeValidStudy();
  study.id.clear();

  const auto errors = rco_core::domain::validate(study);

  EXPECT_TRUE(containsPath(errors, ".id"));
}

TEST(DomainValidationTest, RejectsNonNormalizedQuaternion) {
  auto pose = makeValidPose("world");
  pose.orientation.w = 2.0;

  const auto errors = rco_core::domain::validate(pose);

  EXPECT_TRUE(containsPath(errors, ".orientation"));
}

TEST(DomainValidationTest, RejectsNonFinitePosition) {
  auto pose = makeValidPose("world");
  pose.position.x = std::numeric_limits<double>::infinity();

  const auto errors = rco_core::domain::validate(pose);

  EXPECT_TRUE(containsPath(errors, ".position"));
}

TEST(DomainValidationTest, RejectsEmptyPoseFrame) {
  auto pose = makeValidPose("");

  const auto errors = rco_core::domain::validate(pose);

  EXPECT_TRUE(containsPath(errors, ".frame_id"));
}

TEST(DomainValidationTest, RejectsInvertedJointLimits) {
  auto robot = makeValidRobot();

  robot.joint_limits.front() = JointLimits{
      "shoulder_pan_joint",
      1.0,
      -1.0,
  };

  const auto errors = rco_core::domain::validate(robot);

  EXPECT_TRUE(containsPath(errors, ".joint_limits[0]"));
}

TEST(DomainValidationTest, ReportsNestedRobotErrorPath) {
  auto study = makeValidStudy();
  study.robot.planning_group.clear();

  const auto errors = rco_core::domain::validate(study);

  EXPECT_TRUE(containsPath(errors, ".robot.planning_group"));
}

TEST(DomainValidationTest, ReportsNestedToolTcpErrorPath) {
  auto study = makeValidStudy();
  study.tool.tcp.frame_id.clear();

  const auto errors = rco_core::domain::validate(study);

  EXPECT_TRUE(containsPath(errors, ".tool.tcp.frame_id"));
}

TEST(DomainValidationTest, RejectsNonPositiveBoxDimension) {
  GeometryDefinition geometry;
  geometry.type = GeometryType::kBox;
  geometry.size_m = {1.0, 0.0, 1.0};

  const auto errors = rco_core::domain::validate(geometry);

  EXPECT_TRUE(containsPath(errors, ".size_m"));
}

TEST(DomainValidationTest, RejectsNonFiniteCylinderDimension) {
  GeometryDefinition geometry;
  geometry.type = GeometryType::kCylinder;
  geometry.radius_m = 0.5;
  geometry.height_m = std::numeric_limits<double>::infinity();

  const auto errors = rco_core::domain::validate(geometry);

  EXPECT_TRUE(containsPath(errors, ".height_m"));
}

TEST(DomainValidationTest, RejectsEmptyMeshUri) {
  GeometryDefinition geometry;
  geometry.type = GeometryType::kMesh;

  const auto errors = rco_core::domain::validate(geometry);

  EXPECT_TRUE(containsPath(errors, ".mesh_uri"));
}

TEST(DomainValidationTest, RejectsEmptyRobotModelFrame) {
  auto robot = makeValidRobot();
  robot.model_frame.clear();

  const auto errors = rco_core::domain::validate(robot);

  EXPECT_TRUE(containsPath(errors, ".model_frame"));
}

TEST(DomainValidationTest, RejectsRobotWithoutJointLimits) {
  auto robot = makeValidRobot();
  robot.joint_limits.clear();

  const auto errors = rco_core::domain::validate(robot);

  EXPECT_TRUE(containsPath(errors, ".joint_limits"));
}

TEST(DomainValidationTest, ReportsIndexedDuplicateJointName) {
  auto robot = makeValidRobot();
  robot.joint_limits.push_back(robot.joint_limits.front());

  const auto errors = rco_core::domain::validate(robot);

  EXPECT_TRUE(containsPath(errors, ".joint_limits[1].joint_name"));
}

TEST(DomainValidationTest, RejectsEmptyToolName) {
  auto tool = makeValidTool();
  tool.name.clear();

  const auto errors = rco_core::domain::validate(tool);

  EXPECT_TRUE(containsPath(errors, ".name"));
}

TEST(DomainValidationTest, ReportsNestedToolGeometryPath) {
  auto tool = makeValidTool();
  GeometryDefinition geometry;
  geometry.type = GeometryType::kSphere;
  geometry.radius_m = 0.0;
  tool.geometry.push_back(geometry);

  const auto errors = rco_core::domain::validate(tool);

  EXPECT_TRUE(containsPath(errors, ".geometry[0].radius_m"));
}

TEST(DomainValidationTest, ReportsNestedCellEntityPosePath) {
  CellEntity entity;
  entity.id = "fixture_1";
  entity.type = CellEntityType::kFixture;
  entity.pose = makeValidPose("");
  entity.geometry.type = GeometryType::kBox;
  entity.geometry.size_m = {1.0, 1.0, 1.0};

  const auto errors = rco_core::domain::validate(entity);

  EXPECT_TRUE(containsPath(errors, ".pose.frame_id"));
}

TEST(DomainValidationTest, RejectsDuplicateCellEntityIds) {
  auto cell = makeValidCell();
  CellEntity entity;
  entity.id = "fixture_1";
  entity.type = CellEntityType::kFixture;
  entity.pose = makeValidPose("world");
  entity.geometry.type = GeometryType::kBox;
  entity.geometry.size_m = {1.0, 1.0, 1.0};
  cell.entities = {entity, entity};

  const auto errors = rco_core::domain::validate(cell);

  EXPECT_TRUE(containsPath(errors, ".entities[1].id"));
}

TEST(DomainValidationTest, AcceptsEveryProcessSegmentType) {
  const std::array types{ProcessSegmentType::kProcess, ProcessSegmentType::kApproach,
                         ProcessSegmentType::kRetract, ProcessSegmentType::kTransition};

  for (const auto type : types) {
    ProcessSegment segment;
    segment.type = type;
    segment.targets.push_back(TargetPose{makeValidPose("world")});
    segment.tcp_speed_mps = 0.25;

    EXPECT_TRUE(rco_core::domain::validate(segment).empty());
  }
}

TEST(DomainValidationTest, RejectsTaskWithoutSegments) {
  auto task = makeValidTask();
  task.segments.clear();

  const auto errors = rco_core::domain::validate(task);

  EXPECT_TRUE(containsPath(errors, ".segments"));
}

TEST(DomainValidationTest, RejectsSegmentWithoutTargets) {
  auto task = makeValidTask();
  task.segments.front().targets.clear();

  const auto errors = rco_core::domain::validate(task);

  EXPECT_TRUE(containsPath(errors, ".segments[0].targets"));
}

TEST(DomainValidationTest, RejectsNonPositiveSegmentSpeed) {
  auto task = makeValidTask();
  task.segments.front().tcp_speed_mps = 0.0;

  const auto errors = rco_core::domain::validate(task);

  EXPECT_TRUE(containsPath(errors, ".segments[0].tcp_speed_mps"));
}

TEST(DomainValidationTest, ReportsNestedTargetPosePath) {
  auto task = makeValidTask();
  task.segments.front().targets.front().pose.frame_id.clear();

  const auto errors = rco_core::domain::validate(task);

  EXPECT_TRUE(containsPath(errors, ".segments[0].targets[0].pose.frame_id"));
}

TEST(DomainValidationTest, RejectsZeroEvaluationBudget) {
  OptimizationProblem optimization;
  optimization.evaluation_budget = 0U;

  const auto errors = rco_core::domain::validate(optimization);

  EXPECT_TRUE(containsPath(errors, ".evaluation_budget"));
}

TEST(DomainValidationTest, ReportsNestedStudyOptimizationPath) {
  auto study = makeValidStudy();
  study.optimization.evaluation_budget = 0U;

  const auto errors = rco_core::domain::validate(study);

  EXPECT_TRUE(containsPath(errors, ".optimization.evaluation_budget"));
}

} // namespace
