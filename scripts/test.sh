#!/usr/bin/env bash

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS environment setup is not guaranteed to be nounset-safe.
source /opt/ros/jazzy/setup.bash

if [[ -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  source "$ROOT/ros2_ws/install/setup.bash"
fi

set -u

cd "$ROOT/ros2_ws"

colcon test \
  --event-handlers console_direct+

colcon test-result --verbose
