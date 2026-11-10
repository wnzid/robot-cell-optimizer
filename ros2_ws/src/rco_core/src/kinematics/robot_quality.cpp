#include "rco_core/kinematics/robot_quality.hpp"

#include "rco_core/kinematics/quality.hpp"
#include "rco_core/robot_model_context.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <stdexcept>
#include <vector>

namespace rco_core::kinematics {

Eigen::MatrixXd tcpJacobian(const RobotModelContext& robot,
                            const moveit::core::RobotState& input_state) {
  if (input_state.getRobotModel() != robot.robotModel()) {
    throw std::invalid_argument("Quality state must share the configured robot model.");
  }
  const auto* group = robot.jointModelGroup();
  const auto& links = group->getLinkModelNames();
  if (!group->isChain() || group->getVariableCount() != 6U ||
      std::ranges::find(links, robot.tcpLink()) == links.end()) {
    throw std::invalid_argument("Quality analysis requires a six-axis chain containing the TCP.");
  }
  for (const auto& name : input_state.getVariableNames()) {
    if (!std::isfinite(input_state.getVariablePosition(name))) {
      throw std::invalid_argument("Quality state variables must be finite.");
    }
  }
  if (!input_state.satisfiesBounds()) {
    throw std::invalid_argument("Quality state must satisfy all joint limits.");
  }
  auto state = input_state;
  state.update();
  Eigen::MatrixXd jacobian;
  if (!state.getJacobian(group, robot.robotModel()->getLinkModel(robot.tcpLink()),
                         Eigen::Vector3d::Zero(), jacobian)) {
    throw std::runtime_error("MoveIt failed to calculate the TCP Jacobian.");
  }
  return jacobian;
}

JacobianQuality analyzeRobotState(const RobotModelContext& robot,
                                  const moveit::core::RobotState& state,
                                  const QualityOptions& options) {
  return analyzeJacobian(tcpJacobian(robot, state), options);
}

PathQuality analyzeJointPath(const RobotModelContext& robot,
                             const std::vector<std::vector<double>>& positions_rad,
                             const QualityOptions& options) {
  std::vector<Eigen::MatrixXd> jacobians;
  jacobians.reserve(positions_rad.size());
  for (const auto& positions : positions_rad) {
    if (positions.size() != robot.jointModelGroup()->getVariableCount()) {
      throw std::invalid_argument("Quality path needs one position per group variable.");
    }
    auto state = robot.makeDefaultState();
    state.setJointGroupPositions(robot.jointModelGroup(), positions);
    jacobians.push_back(tcpJacobian(robot, state));
  }
  return analyzeJacobianPath(jacobians, options);
}

} // namespace rco_core::kinematics
