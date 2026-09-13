#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/scene/planning_scene.hpp"

#include <gtest/gtest.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/utils/robot_model_test_utils.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <rclcpp/utilities.hpp>
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

class PlanningSceneTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }
};

CellDefinition makeCell() {
  CellDefinition cell;
  cell.id = "cell_1";
  cell.robot_base.frame_id = "world";
  cell.robot_base.orientation.w = 1.0;
  cell.workpiece_pose.frame_id = "world";
  cell.workpiece_pose.orientation.w = 1.0;
  return cell;
}

CellEntity makeBox(const std::string& id) {
  CellEntity entity;
  entity.id = id;
  entity.type = CellEntityType::kObstacle;
  entity.pose.frame_id = "world";
  entity.pose.orientation.w = 1.0;
  entity.geometry.type = GeometryType::kBox;
  entity.geometry.size_m = {1.0, 0.5, 0.25};
  return entity;
}

CellEntity makeSphere(const std::string& id) {
  CellEntity entity;
  entity.id = id;
  entity.type = CellEntityType::kObstacle;
  entity.pose.frame_id = "world";
  entity.pose.orientation.w = 1.0;
  entity.geometry.type = GeometryType::kSphere;
  entity.geometry.radius_m = 0.2;
  return entity;
}

ToolDefinition makeTool() {
  ToolDefinition tool;
  tool.id = "tool_1";
  tool.name = "reference_tool";
  tool.mass_kg = 1.0;
  tool.tcp.frame_id = "tool0";
  tool.tcp.orientation.w = 1.0;
  auto geometry = rco_core::domain::GeometryDefinition{};
  geometry.type = GeometryType::kBox;
  geometry.size_m = {0.2, 0.1, 0.05};
  tool.geometry = {geometry};
  return tool;
}

moveit::core::RobotModelPtr makeToolRobotModel() {
  moveit::core::RobotModelBuilder builder("test_robot", "world");
  builder.addChain("world->tool0", "fixed");
  return builder.build();
}

TEST_F(PlanningSceneTest, BuildsNamedWorldDiffInDefinitionOrder) {
  auto cell = makeCell();
  cell.entities = {makeBox("table_1"), makeSphere("obstacle_1")};

  const auto scene = rco_core::scene::makePlanningSceneWorldDiff(cell);

  EXPECT_EQ(scene.name, "cell_1");
  EXPECT_TRUE(scene.is_diff);
  EXPECT_TRUE(scene.robot_state.is_diff);
  ASSERT_EQ(scene.world.collision_objects.size(), 2U);
  EXPECT_EQ(scene.world.collision_objects.front().id, "table_1");
  EXPECT_EQ(scene.world.collision_objects.front().primitives.front().type,
            shape_msgs::msg::SolidPrimitive::BOX);
  EXPECT_EQ(scene.world.collision_objects.back().id, "obstacle_1");
  EXPECT_EQ(scene.world.collision_objects.back().primitives.front().type,
            shape_msgs::msg::SolidPrimitive::SPHERE);
}

TEST_F(PlanningSceneTest, BuildsEmptyWorldDiffForEmptyValidCell) {
  const auto scene = rco_core::scene::makePlanningSceneWorldDiff(makeCell());

  EXPECT_EQ(scene.name, "cell_1");
  EXPECT_TRUE(scene.is_diff);
  EXPECT_TRUE(scene.robot_state.is_diff);
  EXPECT_TRUE(scene.world.collision_objects.empty());
}

TEST_F(PlanningSceneTest, RejectsInvalidCellBeforeBuildingDiff) {
  auto cell = makeCell();
  const auto duplicate = makeSphere("obstacle_1");
  cell.entities = {duplicate, duplicate};

  EXPECT_THROW(static_cast<void>(rco_core::scene::makePlanningSceneWorldDiff(cell)),
               std::invalid_argument);
}

TEST_F(PlanningSceneTest, AppliesCellWorldDiffToMoveItScene) {
  moveit::core::RobotModelBuilder builder("test_robot", "world");
  const auto robot_model = builder.build();
  ASSERT_NE(robot_model, nullptr);
  planning_scene::PlanningScene scene(robot_model);
  auto cell = makeCell();
  cell.entities = {makeBox("table_1"), makeSphere("obstacle_1")};

  ASSERT_TRUE(rco_core::scene::applyCellWorldDiff(scene, cell));

  EXPECT_EQ(scene.getName(), "cell_1");
  EXPECT_EQ(scene.getWorld()->size(), 2U);
  EXPECT_TRUE(scene.getWorld()->hasObject("table_1"));
  EXPECT_TRUE(scene.getWorld()->hasObject("obstacle_1"));
}

TEST_F(PlanningSceneTest, RejectsInvalidCellBeforeMutatingMoveItScene) {
  moveit::core::RobotModelBuilder builder("test_robot", "world");
  const auto robot_model = builder.build();
  ASSERT_NE(robot_model, nullptr);
  planning_scene::PlanningScene scene(robot_model);
  auto initial_cell = makeCell();
  initial_cell.entities = {makeBox("table_1")};
  ASSERT_TRUE(rco_core::scene::applyCellWorldDiff(scene, initial_cell));
  auto invalid_cell = makeCell();
  const auto duplicate = makeSphere("obstacle_1");
  invalid_cell.entities = {duplicate, duplicate};

  EXPECT_THROW(static_cast<void>(rco_core::scene::applyCellWorldDiff(scene, invalid_cell)),
               std::invalid_argument);
  EXPECT_EQ(scene.getName(), "cell_1");
  EXPECT_EQ(scene.getWorld()->size(), 1U);
  EXPECT_TRUE(scene.getWorld()->hasObject("table_1"));
}

TEST_F(PlanningSceneTest, ReplacesExistingWorldWithoutLeavingStaleObjects) {
  moveit::core::RobotModelBuilder builder("test_robot", "world");
  const auto robot_model = builder.build();
  ASSERT_NE(robot_model, nullptr);
  planning_scene::PlanningScene scene(robot_model);
  auto initial_cell = makeCell();
  initial_cell.entities = {makeBox("table_1"), makeSphere("obstacle_1")};
  ASSERT_TRUE(rco_core::scene::applyCellWorldDiff(scene, initial_cell));
  auto replacement_cell = makeCell();
  replacement_cell.id = "cell_2";
  replacement_cell.entities = {makeSphere("fixture_1")};

  ASSERT_TRUE(rco_core::scene::replaceCellWorld(scene, replacement_cell));

  EXPECT_EQ(scene.getName(), "cell_2");
  EXPECT_EQ(scene.getWorld()->size(), 1U);
  EXPECT_FALSE(scene.getWorld()->hasObject("table_1"));
  EXPECT_FALSE(scene.getWorld()->hasObject("obstacle_1"));
  EXPECT_TRUE(scene.getWorld()->hasObject("fixture_1"));
}

TEST_F(PlanningSceneTest, ReplacesExistingWorldWithEmptyCell) {
  moveit::core::RobotModelBuilder builder("test_robot", "world");
  const auto robot_model = builder.build();
  ASSERT_NE(robot_model, nullptr);
  planning_scene::PlanningScene scene(robot_model);
  auto initial_cell = makeCell();
  initial_cell.entities = {makeBox("table_1")};
  ASSERT_TRUE(rco_core::scene::applyCellWorldDiff(scene, initial_cell));
  auto empty_cell = makeCell();
  empty_cell.id = "empty_cell";

  ASSERT_TRUE(rco_core::scene::replaceCellWorld(scene, empty_cell));

  EXPECT_EQ(scene.getName(), "empty_cell");
  EXPECT_TRUE(scene.getWorld()->getObjectIds().empty());
}

TEST_F(PlanningSceneTest, RejectsInvalidReplacementBeforeRemovingExistingWorld) {
  moveit::core::RobotModelBuilder builder("test_robot", "world");
  const auto robot_model = builder.build();
  ASSERT_NE(robot_model, nullptr);
  planning_scene::PlanningScene scene(robot_model);
  auto initial_cell = makeCell();
  initial_cell.entities = {makeBox("table_1")};
  ASSERT_TRUE(rco_core::scene::applyCellWorldDiff(scene, initial_cell));
  auto invalid_cell = makeCell();
  const auto duplicate = makeSphere("obstacle_1");
  invalid_cell.entities = {duplicate, duplicate};

  EXPECT_THROW(static_cast<void>(rco_core::scene::replaceCellWorld(scene, invalid_cell)),
               std::invalid_argument);
  EXPECT_EQ(scene.getWorld()->size(), 1U);
  EXPECT_TRUE(scene.getWorld()->hasObject("table_1"));
}

TEST_F(PlanningSceneTest, AppliesAttachedToolToMoveItScene) {
  const auto robot_model = makeToolRobotModel();
  ASSERT_NE(robot_model, nullptr);
  planning_scene::PlanningScene scene(robot_model);
  const std::vector<std::string> touch_links{"tool0"};

  ASSERT_TRUE(rco_core::scene::applyAttachedTool(scene, makeTool(), touch_links));

  EXPECT_TRUE(scene.getCurrentState().hasAttachedBody("tool_1"));
  moveit_msgs::msg::AttachedCollisionObject attached;
  ASSERT_TRUE(scene.getAttachedCollisionObjectMsg(attached, "tool_1"));
  EXPECT_EQ(attached.link_name, "tool0");
  EXPECT_EQ(attached.touch_links, touch_links);
}

TEST_F(PlanningSceneTest, RejectsInvalidToolBeforeMutatingMoveItScene) {
  const auto robot_model = makeToolRobotModel();
  ASSERT_NE(robot_model, nullptr);
  planning_scene::PlanningScene scene(robot_model);
  const std::vector<std::string> touch_links{"tool0"};
  ASSERT_TRUE(rco_core::scene::applyAttachedTool(scene, makeTool(), touch_links));
  auto invalid_tool = makeTool();
  invalid_tool.geometry.front().size_m.y = 0.0;

  EXPECT_THROW(
      static_cast<void>(rco_core::scene::applyAttachedTool(scene, invalid_tool, touch_links)),
      std::invalid_argument);
  EXPECT_TRUE(scene.getCurrentState().hasAttachedBody("tool_1"));
}

TEST_F(PlanningSceneTest, ReplacesAttachedToolWithoutLeavingStaleBody) {
  const auto robot_model = makeToolRobotModel();
  ASSERT_NE(robot_model, nullptr);
  planning_scene::PlanningScene scene(robot_model);
  const std::vector<std::string> touch_links{"tool0"};
  ASSERT_TRUE(rco_core::scene::applyAttachedTool(scene, makeTool(), touch_links));
  auto replacement = makeTool();
  replacement.id = "tool_2";

  ASSERT_TRUE(rco_core::scene::replaceAttachedTool(scene, "tool_1", replacement, touch_links));

  EXPECT_FALSE(scene.getCurrentState().hasAttachedBody("tool_1"));
  EXPECT_TRUE(scene.getCurrentState().hasAttachedBody("tool_2"));
}

TEST_F(PlanningSceneTest, RejectsUnknownReplacementLinkBeforeRemovingTool) {
  const auto robot_model = makeToolRobotModel();
  ASSERT_NE(robot_model, nullptr);
  planning_scene::PlanningScene scene(robot_model);
  const std::vector<std::string> touch_links{"tool0"};
  ASSERT_TRUE(rco_core::scene::applyAttachedTool(scene, makeTool(), touch_links));
  auto replacement = makeTool();
  replacement.id = "tool_2";
  replacement.tcp.frame_id = "missing_link";

  EXPECT_FALSE(rco_core::scene::replaceAttachedTool(scene, "tool_1", replacement, touch_links));
  EXPECT_TRUE(scene.getCurrentState().hasAttachedBody("tool_1"));
  EXPECT_FALSE(scene.getCurrentState().hasAttachedBody("tool_2"));
}

} // namespace
