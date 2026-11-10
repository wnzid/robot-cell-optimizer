#pragma once

#include "rco_core/kinematics/quality.hpp"
#include "rco_core/robot_model_context.hpp"

#include <Eigen/Core>
#include <moveit/robot_state/robot_state.hpp>
#include <vector>

namespace rco_core::kinematics {

// Geometric Jacobian at the configured TCP link origin, expressed in the
// planning group's root reference frame (MoveIt convention). Copies the state;
// requires the same robot model, finite variables and all joint bounds satisfied.
// The group must be a chain with six variables and contain the TCP link.
[[nodiscard]] Eigen::MatrixXd tcpJacobian(const RobotModelContext& robot,
                                          const moveit::core::RobotState& state);

[[nodiscard]] JacobianQuality analyzeRobotState(const RobotModelContext& robot,
                                                const moveit::core::RobotState& state,
                                                const QualityOptions& options = {});

// Every vector must hold six finite, in-bounds group positions in group order.
// Other joints use the robot's default state. No skipping of failed IK results:
// callers must choose solved samples explicitly and report omitted samples.
[[nodiscard]] PathQuality analyzeJointPath(const RobotModelContext& robot,
                                           const std::vector<std::vector<double>>& positions_rad,
                                           const QualityOptions& options = {});

} // namespace rco_core::kinematics
