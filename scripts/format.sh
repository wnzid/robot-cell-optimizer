#!/usr/bin/env bash

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

mapfile -t files < <(
  find "$ROOT/ros2_ws/src" \
    -type f \
    \( \
      -name '*.cpp' -o \
      -name '*.hpp' -o \
      -name '*.cc' -o \
      -name '*.cxx' -o \
      -name '*.h' \
    \) \
    | sort
)

if [[ ${#files[@]} -eq 0 ]]; then
  echo "No C/C++ files found."
  exit 0
fi

clang-format -i "${files[@]}"

echo "Formatted ${#files[@]} file(s)."
