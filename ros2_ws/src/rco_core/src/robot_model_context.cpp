#include "rco_core/robot_model_context.hpp"

#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/node.hpp>
#include <stdexcept>
#include <string>
#include <utility>

namespace rco_core {

RobotModelContext::RobotModelContext(const rclcpp::Node::SharedPtr& node,
                                     std::string planning_group, std::string tcp_link)
    : robot_model_loader_(node, "robot_description"), robot_model_(robot_model_loader_.getModel()),
      planning_group_(std::move(planning_group)), tcp_link_(std::move(tcp_link)) {
  if (!robot_model_) {
    throw std::runtime_error("Failed to load MoveIt RobotModel from 'robot_description'.");
  }

  joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);

  if (joint_model_group_ == nullptr) {
    throw std::invalid_argument("Planning group '" + planning_group_ +
                                "' does not exist in RobotModel.");
  }

  if (!robot_model_->hasLinkModel(tcp_link_)) {
    throw std::invalid_argument("TCP link '" + tcp_link_ + "' does not exist in RobotModel.");
  }

  model_frame_ = robot_model_->getModelFrame();
}

const moveit::core::RobotModelConstPtr& RobotModelContext::robotModel() const noexcept {
  return robot_model_;
}

const moveit::core::JointModelGroup* RobotModelContext::jointModelGroup() const noexcept {
  return joint_model_group_;
}

const std::string& RobotModelContext::planningGroup() const noexcept {
  return planning_group_;
}

const std::string& RobotModelContext::tcpLink() const noexcept {
  return tcp_link_;
}

const std::string& RobotModelContext::modelFrame() const noexcept {
  return model_frame_;
}

moveit::core::RobotState RobotModelContext::makeDefaultState() const {
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.update();
  return state;
}

} // namespace rco_core
