#!/usr/bin/env bash

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$ROOT/.venv"

if [[ ! -x "$VENV/bin/python" ]]; then
  echo "Python virtual environment not found at $VENV."
  exit 1
fi

cd "$ROOT/app/backend"

echo "=== Ruff lint ==="
"$VENV/bin/ruff" check src tests

echo
echo "=== Ruff format ==="
"$VENV/bin/ruff" format --check src tests

echo
echo "=== mypy ==="
"$VENV/bin/mypy" src tests

echo
echo "=== pytest ==="
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$VENV/bin/pytest"

echo
echo "Python quality checks passed."
