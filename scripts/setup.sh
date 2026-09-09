#!/usr/bin/env bash

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "========================================"
echo " Robot Cell Optimizer — Project Setup"
echo "========================================"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: ROS 2 Jazzy was not found at /opt/ros/jazzy."
  exit 1
fi

echo
echo "=== ROS 2 Jazzy ==="

# Source ROS before enabling nounset because ROS setup scripts may reference
# variables that are undefined in a completely fresh shell.
source /opt/ros/jazzy/setup.bash
set -u

echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"

if ! command -v rosdep >/dev/null 2>&1; then
  echo "ERROR: rosdep is not installed."
  exit 1
fi

echo
echo "=== ROS dependencies ==="

rosdep update

rosdep install \
  --from-paths ros2_ws/src \
  --ignore-src \
  --rosdistro jazzy \
  -r \
  -y

echo
echo "=== Python environment ==="

if [[ ! -x .venv/bin/python ]]; then
  echo "Creating .venv..."
  python3 -m venv .venv
fi

env -u PYTHONPATH \
  .venv/bin/python -m pip install \
  -r app/backend/requirements-dev.lock

env -u PYTHONPATH \
  .venv/bin/python -m pip install \
  --no-deps \
  -e app/backend

env -u PYTHONPATH \
  .venv/bin/python -m pip check

echo
echo "=== Node.js ==="

if ! command -v node >/dev/null 2>&1; then
  echo "ERROR: Node.js is not available in this shell."
  echo "If using NVM, run:"
  echo "  nvm use"
  exit 1
fi

if ! command -v npm >/dev/null 2>&1; then
  echo "ERROR: npm is not available."
  exit 1
fi

EXPECTED_NODE="$(tr -d '[:space:]' < .nvmrc)"
ACTUAL_NODE="$(node --version | sed 's/^v//')"

echo "Expected Node: $EXPECTED_NODE"
echo "Actual Node:   $ACTUAL_NODE"

if [[ "$ACTUAL_NODE" != "$EXPECTED_NODE" ]]; then
  echo
  echo "ERROR: Node.js version does not match .nvmrc."
  echo "Run:"
  echo "  nvm use"
  exit 1
fi

echo
echo "=== Frontend dependencies ==="

(
  cd app/frontend
  npm ci
)

echo
echo "=== Pre-commit ==="

.venv/bin/pre-commit install

echo
echo "========================================"
echo " Project setup complete"
echo "========================================"
echo
echo "Next:"
echo "  ./scripts/quality.sh"
