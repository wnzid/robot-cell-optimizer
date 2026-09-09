#include "rco_core/robot_model_context.hpp"

#include <gtest/gtest.h>
#include <memory>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <stdexcept>
#include <vector>

namespace {

rclcpp::Node::SharedPtr makeNode(const char* name) {
  return std::make_shared<rclcpp::Node>(
      name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
}

// GoogleTest macros expand into internal control flow.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
TEST(RobotModelContextTest, RejectsInvalidPlanningGroup) {
  const auto node = makeNode("rco_invalid_group_test");

  EXPECT_THROW(rco_core::RobotModelContext(node, "does_not_exist", "tool0"), std::invalid_argument);
}

// GoogleTest macros expand into internal control flow.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
TEST(RobotModelContextTest, RejectsInvalidTcpLink) {
  const auto node = makeNode("rco_invalid_tcp_test");

  EXPECT_THROW(rco_core::RobotModelContext(node, "ur_manipulator", "does_not_exist"),
               std::invalid_argument);
}

// GoogleTest macros expand into internal control flow.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
TEST(RobotModelContextTest, Ur5eJointLimitsAreUsable) {
  const auto node = makeNode("rco_joint_limits_test");

  const rco_core::RobotModelContext robot{
      node,
      "ur_manipulator",
      "tool0",
  };

  const auto* group = robot.jointModelGroup();

  ASSERT_TRUE(group != nullptr && group->getVariableCount() == 6U);

  const auto state = robot.makeDefaultState();

  EXPECT_TRUE(state.satisfiesBounds(group));

  for (const auto& variable_name : group->getVariableNames()) {
    const auto& bounds = robot.robotModel()->getVariableBounds(variable_name);

    EXPECT_TRUE(bounds.position_bounded_);
    EXPECT_LT(bounds.min_position_, bounds.max_position_);
  }
}

// GoogleTest macros expand into internal control flow.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
TEST(RobotModelContextTest, DetectsJointValueOutsideBounds) {
  const auto node = makeNode("rco_joint_out_of_bounds_test");

  const rco_core::RobotModelContext robot{
      node,
      "ur_manipulator",
      "tool0",
  };

  const auto* group = robot.jointModelGroup();
  ASSERT_NE(group, nullptr);

  auto state = robot.makeDefaultState();

  std::vector<double> values;
  state.copyJointGroupPositions(group, values);

  ASSERT_FALSE(values.empty());

  const auto& first_variable = group->getVariableNames().front();

  const auto& bounds = robot.robotModel()->getVariableBounds(first_variable);

  values.front() = bounds.max_position_ + 1.0;

  state.setJointGroupPositions(group, values);
  state.update();

  EXPECT_FALSE(state.satisfiesBounds(group));

  state.enforceBounds(group);
  state.update();

  EXPECT_TRUE(state.satisfiesBounds(group));
}

} // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  const int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
