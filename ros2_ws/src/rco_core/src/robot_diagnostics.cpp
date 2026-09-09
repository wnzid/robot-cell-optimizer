#include "rco_core/robot_model_context.hpp"

#include <exception>
#include <iostream>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <string>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<rclcpp::Node>(
        "rco_robot_diagnostics",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    const std::string planning_group = "ur_manipulator";
    const std::string tcp_link = "tool0";

    const rco_core::RobotModelContext robot{
        node,
        planning_group,
        tcp_link,
    };

    const auto& model = robot.robotModel();
    const auto* group = robot.jointModelGroup();

    std::cout << "Robot Cell Optimizer - Robot Diagnostics\n";
    std::cout << "========================================\n";
    std::cout << "Model name:      " << model->getName() << '\n';
    std::cout << "Model frame:     " << robot.modelFrame() << '\n';
    std::cout << "Planning group:  " << robot.planningGroup() << '\n';
    std::cout << "TCP link:        " << robot.tcpLink() << '\n';
    std::cout << "DOF:             " << group->getVariableCount() << '\n';

    std::cout << "\nActive joints:\n";
    for (const auto* joint : group->getActiveJointModels()) {
      std::cout << "  - " << joint->getName() << '\n';
    }

    std::cout << "\nJoint variables and bounds:\n";
    for (const auto& variable_name : group->getVariableNames()) {
      const auto& bounds = model->getVariableBounds(variable_name);

      std::cout << "  - " << variable_name << ": [" << bounds.min_position_ << ", "
                << bounds.max_position_ << "]\n";
    }

    auto state = robot.makeDefaultState();

    const auto& tcp_transform = state.getGlobalLinkTransform(robot.tcpLink());

    const auto& translation = tcp_transform.translation();

    std::cout << "\nDefault TCP position:\n";
    std::cout << "  x: " << translation.x() << '\n';
    std::cout << "  y: " << translation.y() << '\n';
    std::cout << "  z: " << translation.z() << '\n';

    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << "Robot diagnostics failed: " << exception.what() << '\n';

    rclcpp::shutdown();
    return 1;
  }
}
