#!/usr/bin/env bash

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS setup scripts may reference optional unset variables,
# so enable nounset only after sourcing the environment.
source /opt/ros/jazzy/setup.bash

set -u

cd "$ROOT/ros2_ws"

colcon build \
  --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
