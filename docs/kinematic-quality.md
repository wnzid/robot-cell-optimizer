# Kinematic quality analysis

`analyzeRobotState` computes the geometric Jacobian at the configured TCP link
origin using MoveIt's planning-group reference frame, then analyzes its singular
values. The state is copied and must share the robot model, have finite variables,
and satisfy joint bounds. This implementation supports six-variable chain groups.

## Units and scaling

The raw Jacobian has linear rows in meters per radian and angular rows in radians
per radian. `QualityOptions::characteristic_length_m` divides the three linear rows
before SVD. Set it explicitly for a study and use the same value for candidate
comparisons. The default is 1 meter. Metrics are properties of the scaled Jacobian;
condition numbers from different scaling conventions should not be compared.

Reported metrics:

- Minimum and maximum singular values.
- Yoshikawa manipulability, computed as the product of six singular values rather
  than a determinant. This is equivalent to sqrt(det(J J^T)) for the scaled matrix.
- Numerical rank and a near-singular flag. A singular value counts toward rank
  only when it exceeds `relative_singular_threshold * maximum_singular_value`.
  The configurable relative threshold defaults to 1e-6.
- Condition number, sigma_max / sigma_min, absent when the matrix is classified
  as rank deficient. Absence avoids infinity/NaN in downstream serialization.
  Small nonzero singular values and manipulability remain visible even when the
  threshold classifies a state as near-singular.

This classification is a configurable numerical test, not a robot safety
certification or a guarantee of dynamic feasibility. Joint-limit margins remain
available separately through `normalizedJointMargins`.

## Path summaries

`analyzeJointPath` evaluates every supplied group joint vector. Other joints use
the default robot state. For custom attached bodies or non-group context, use
`analyzeRobotState` with an explicit state instead. Invalid samples throw; there
is no silent omission of failed samples. Callers converting reachability results
must explicitly account for failed IK samples before submitting solved positions.

The path result reports sample and near-singular counts and min/max/mean/5th/95th
percentiles for minimum singular value, manipulability and defined condition
numbers. Percentiles use linear interpolation at (N - 1) * probability. Condition
statistics include only defined condition numbers, with their own count. Always
inspect `near_singular_count` alongside condition statistics; a finite mean does
not mean the full path is well-conditioned. Empty paths have absent summaries.

## Verification

`test_kinematic_quality` covers analytical identity/diagonal/rank-deficient
matrices, scaling, frame rotation, thresholds, invalid values, overflow and path
aggregation. Four UR5e fixtures run via the existing robot-model launch test:
normal rank, wrist singularity, bad inputs, and robot-model mismatch.

From the repository root, after building:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
cd ros2_ws
colcon test --packages-select rco_core --ctest-args -R 'test_kinematic_quality|test_test_robot_model_context_launch.py' --event-handlers console_direct+
colcon test-result --verbose
```
