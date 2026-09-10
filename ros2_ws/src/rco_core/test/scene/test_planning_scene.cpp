#include "rco_core/domain/model.hpp"
#include "rco_core/domain/types.hpp"
#include "rco_core/scene/planning_scene.hpp"

#include <gtest/gtest.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <stdexcept>
#include <string>

namespace {

using rco_core::domain::CellDefinition;
using rco_core::domain::CellEntity;
using rco_core::domain::CellEntityType;
using rco_core::domain::GeometryType;

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

TEST(PlanningSceneTest, BuildsNamedWorldDiffInDefinitionOrder) {
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

TEST(PlanningSceneTest, BuildsEmptyWorldDiffForEmptyValidCell) {
  const auto scene = rco_core::scene::makePlanningSceneWorldDiff(makeCell());

  EXPECT_EQ(scene.name, "cell_1");
  EXPECT_TRUE(scene.is_diff);
  EXPECT_TRUE(scene.robot_state.is_diff);
  EXPECT_TRUE(scene.world.collision_objects.empty());
}

TEST(PlanningSceneTest, RejectsInvalidCellBeforeBuildingDiff) {
  auto cell = makeCell();
  const auto duplicate = makeSphere("obstacle_1");
  cell.entities = {duplicate, duplicate};

  EXPECT_THROW(static_cast<void>(rco_core::scene::makePlanningSceneWorldDiff(cell)),
               std::invalid_argument);
}

} // namespace
