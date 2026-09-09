#!/usr/bin/env bash

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "========================================"
echo " Robot Cell Optimizer — Quality Gate"
echo "========================================"

echo
echo "=== 1/7 C++ formatting ==="
./scripts/check-format.sh

echo
echo "=== 2/7 ROS/C++ build ==="
./scripts/build.sh

echo
echo "=== 3/7 ROS/C++ tests ==="
./scripts/test.sh

echo
echo "=== 4/7 clang-tidy ==="
./scripts/check-clang-tidy.sh

echo
echo "=== 5/7 Python ==="
./scripts/check-python.sh

echo
echo "=== 6/7 Frontend ==="
./scripts/check-frontend.sh

echo
echo "=== 7/7 Git whitespace ==="
git diff --check

echo
echo "========================================"
echo " ALL QUALITY GATES PASSED"
echo "========================================"
