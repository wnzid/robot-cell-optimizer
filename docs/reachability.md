# Reachability evaluation

## Seeded single-target IK

`rco_core::reachability::solveSeededIk` evaluates one TCP target with the
configured MoveIt solver and a caller-supplied joint seed. It calls
`getPositionIK` once. MoveIt's contract for this method forbids random
reseeding; the configured solver must honor that contract.

Inputs:

- A loaded `RobotModelContext` with a single-tip solver whose tip matches the
  configured TCP link.
- A validated `TargetPose` expressed in `robot.modelFrame()`. Positions are
  meters and orientations are normalized quaternions.
- Finite joint positions in radians, ordered by the planning group's variable
  names, with exactly one value per variable and within robot joint limits.
- The caller's sample index, preserved in the result.

The target is transformed from the model frame into the solver base frame.
MoveIt's joint-variable mapping converts between group order and solver order.
Other joints retain the model's default state. Custom tool offsets and external
workpiece frames must be resolved by the caller before this API is used.

A successful result contains group-ordered joint positions. The returned state
must satisfy joint bounds and reproduce the target through forward kinematics
within **0.0001 m** position error and **0.0001 rad** orientation error.

Failure behavior:

- `INVALID_TARGET`: invalid pose values or a frame other than the model frame.
- `IK_FAILED`: solver failure, invalid solver output, or failed FK verification.
- `JOINT_LIMIT`: a returned finite solution violates joint bounds.
- Invalid seeds and unsupported robot/solver configurations throw
  `std::invalid_argument`; they are caller configuration errors.

A failed IK attempt is not proof that every other seed would fail. This API
reports kinematic reachability only. It does not yet check collisions or joint
discontinuity between path samples, so a successful result is not a motion-safety
or full manufacturing-task feasibility result.

`ContinuitySeed` can retain successful solutions between calls. It updates only
from reachable results and keeps the last accepted seed across failures. Its
reset method restores the caller's initial seed.

## Verification

Real UR5e/KDL fixtures cover nearby reachable targets, a far-away unreachable
target, repeated identical evaluations, FK agreement, invalid frames and poses,
invalid seeds, and incompatible TCP configuration. They run through the existing
robot-model launch test, which supplies the actual URDF, SRDF, and IK plugin.

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
cd ros2_ws
colcon test --packages-select rco_core --ctest-args -R test_test_robot_model_context_launch.py
colcon test-result --verbose
```
