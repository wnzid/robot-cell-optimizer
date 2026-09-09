#!/usr/bin/env bash

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$ROOT/.venv"

if [[ ! -x "$VENV/bin/python" ]]; then
  echo "Python virtual environment not found at $VENV."
  exit 1
fi

cd "$ROOT/app/backend"

run_clean() {
  env -u PYTHONPATH "$@"
}

echo "=== Ruff lint ==="
run_clean "$VENV/bin/ruff" check src tests

echo
echo "=== Ruff format ==="
run_clean "$VENV/bin/ruff" format --check src tests

echo
echo "=== mypy ==="
run_clean "$VENV/bin/mypy" src tests

echo
echo "=== pytest ==="
env -u PYTHONPATH \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  "$VENV/bin/pytest"

echo
echo "Python quality checks passed."
