#!/usr/bin/env bash

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

rm -rf \
  "$ROOT/ros2_ws/build" \
  "$ROOT/ros2_ws/install" \
  "$ROOT/ros2_ws/log"

echo "ROS workspace build artifacts removed."
