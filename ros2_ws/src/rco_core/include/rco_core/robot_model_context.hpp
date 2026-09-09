#pragma once

#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/node.hpp>
#include <string>

namespace rco_core {

class RobotModelContext {
public:
  RobotModelContext(const rclcpp::Node::SharedPtr& node, std::string planning_group,
                    std::string tcp_link);

  [[nodiscard]] const moveit::core::RobotModelConstPtr& robotModel() const noexcept;

  [[nodiscard]] const moveit::core::JointModelGroup* jointModelGroup() const noexcept;

  [[nodiscard]] const std::string& planningGroup() const noexcept;

  [[nodiscard]] const std::string& tcpLink() const noexcept;

  [[nodiscard]] const std::string& modelFrame() const noexcept;

  [[nodiscard]] moveit::core::RobotState makeDefaultState() const;

private:
  robot_model_loader::RobotModelLoader robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_{nullptr};

  std::string planning_group_;
  std::string tcp_link_;
  std::string model_frame_;
};

} // namespace rco_core
