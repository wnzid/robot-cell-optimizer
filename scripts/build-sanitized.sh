#!/usr/bin/env bash

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy was not found at /opt/ros/jazzy."
  exit 1
fi

# Source ROS before enabling nounset because ROS setup scripts may reference
# variables that are not defined in a fresh shell.
source /opt/ros/jazzy/setup.bash
set -u

cd "$WS"

echo "=== Sanitized ROS/C++ build ==="
echo "Sanitizers: AddressSanitizer + UndefinedBehaviorSanitizer"
echo

colcon \
  --log-base log-sanitized \
  build \
  --build-base build-sanitized \
  --install-base install-sanitized \
  --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    "-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer" \
    "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined" \
    "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=address,undefined"

echo
echo "Sanitized build passed."
