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
The vector-seed overload keeps other joints at the model's default state. The
`RobotState` overload preserves the supplied non-group joints and checks that
the state shares the configured robot model, is finite, and satisfies all joint
limits. Custom tool offsets and external workpiece frames must be resolved by
the caller before this API is used.

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

## Ordered paths

`solveSeededPath` evaluates a nonempty vector of targets in order, returning one
indexed result per target. The initial seed is used for the first sample; only
accepted solutions become seeds for later samples. Failed samples remain in the
output, and evaluation continues from the last accepted seed.

The caller supplies a finite positive maximum joint step in radians. Any joint
whose absolute displacement from the seed exceeds that limit causes a
`DISCONTINUITY` result. The candidate joint solution is retained for diagnostics,
but does not become the next seed. This rule also applies to the first sample's
displacement from the initial seed. Joint coordinates are compared without
modulo-2-pi wrapping so a change of winding is not silently accepted.

This is a kinematic path check, not a continuous collision check or trajectory
planner. Later successes do not erase earlier failures; downstream feasibility
evaluation must account for every mandatory sample.

## Collision-aware paths

The `solveSeededPath` overload accepting a `PlanningScene` checks each IK
solution for whole-robot self collisions, world collisions, and attached-body
collisions before accepting it as the next seed. It uses the scene's allowed
collision matrix and padding. The scene must share the exact robot model
instance with the robot context. Evaluation copies the current scene state,
preserving attached bodies and joints outside the planning group; neither the
scene nor its state is changed.

A colliding solution becomes `COLLISION` and retains its joint positions for
diagnostics. It never advances the seed. The precedence is invalid target / IK
failure / joint-limit failure, then collision, then joint discontinuity. Every
sample still has a result, and evaluation continues after failures.

These checks cover the supplied discrete samples only. They do not prove that
the motion between samples is collision-free. Swept or sampled trajectory
checking belongs to the later collision/trajectory stages.

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
