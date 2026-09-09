#!/usr/bin/env bash

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash

set -u

if ! command -v clang-tidy >/dev/null 2>&1; then
  echo "clang-tidy is not installed."
  exit 1
fi

found=0

while IFS= read -r file; do
  found=1

  relative="${file#"$ROOT/ros2_ws/src/"}"
  package="${relative%%/*}"
  build_dir="$ROOT/ros2_ws/build/$package"

  if [[ ! -f "$build_dir/compile_commands.json" ]]; then
    echo "Missing compile_commands.json for $package."
    echo "Run ./scripts/build.sh first."
    exit 1
  fi

  echo
  echo "clang-tidy: ${file#"$ROOT/"}"

  clang-tidy \
    --quiet \
    "$file" \
    -p "$build_dir" \
    --warnings-as-errors='*'
done < <(
  find "$ROOT/ros2_ws/src" \
    -type f \
    \( \
      -name '*.cpp' -o \
      -name '*.cc' -o \
      -name '*.cxx' \
    \) \
    | sort
)

if [[ "$found" -eq 0 ]]; then
  echo "No C++ translation units found."
  exit 0
fi

echo
echo "clang-tidy passed."
