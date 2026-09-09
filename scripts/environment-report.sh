#!/usr/bin/env bash

set -eo pipefail

echo "Robot Cell Optimizer — Environment Report"
echo "========================================="
echo

echo "[Operating system]"
if [[ -f /etc/os-release ]]; then
  . /etc/os-release
  echo "${PRETTY_NAME:-unknown}"
else
  uname -a
fi

echo
echo "[Kernel]"
uname -r

echo
echo "[ROS]"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
  echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
  echo "ROS_VERSION=${ROS_VERSION:-unknown}"
else
  echo "ROS 2 Jazzy not found"
fi

echo
echo "[Compilers]"
gcc --version | head -n 1
g++ --version | head -n 1
cmake --version | head -n 1

echo
echo "[Python]"
python3 --version

echo
echo "[Node]"
if command -v node >/dev/null 2>&1; then
  node --version
else
  echo "node: not found"
fi

if command -v npm >/dev/null 2>&1; then
  printf "npm "
  npm --version
else
  echo "npm: not found"
fi

echo
echo "[ROS build tools]"
colcon version-check 2>/dev/null || colcon --help 2>&1 | head -n 1

echo
echo "[MoveIt]"
if command -v dpkg-query >/dev/null 2>&1; then
  dpkg-query -W -f='${Package} ${Version}\n' \
    'ros-jazzy-moveit-core' \
    'ros-jazzy-moveit-ros-planning' \
    2>/dev/null || true
fi

echo
echo "[Universal Robots]"
if command -v dpkg-query >/dev/null 2>&1; then
  dpkg-query -W -f='${Package} ${Version}\n' \
    'ros-jazzy-ur-description' \
    'ros-jazzy-ur-robot-driver' \
    2>/dev/null || true
fi

echo
echo "[Gazebo]"
if command -v gz >/dev/null 2>&1; then
  gz --versions 2>/dev/null || gz --version 2>/dev/null || true
else
  echo "gz: not found"
fi

echo
echo "[Git]"
git --version
