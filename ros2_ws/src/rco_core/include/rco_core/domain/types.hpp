#pragma once

#include <string>

namespace rco_core::domain {

// Domain schema objects are intentionally plain data structures for
// validation, serialization, and deterministic comparison.
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)

struct Vector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  bool operator==(const Vector3&) const = default;
};

struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};

  bool operator==(const Quaternion&) const = default;
};

struct Pose {
  std::string frame_id;
  Vector3 position;
  Quaternion orientation;

  bool operator==(const Pose&) const = default;
};

struct JointLimits {
  std::string joint_name;
  double minimum_position_rad{0.0};
  double maximum_position_rad{0.0};

  bool operator==(const JointLimits&) const = default;
};

// NOLINTEND(misc-non-private-member-variables-in-classes)

} // namespace rco_core::domain
