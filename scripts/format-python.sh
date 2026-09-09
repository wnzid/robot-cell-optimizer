#!/usr/bin/env bash

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$ROOT/.venv"

if [[ ! -x "$VENV/bin/ruff" ]]; then
  echo "Ruff not found in $VENV."
  exit 1
fi

cd "$ROOT/app/backend"

env -u PYTHONPATH "$VENV/bin/ruff" check --fix src tests
env -u PYTHONPATH "$VENV/bin/ruff" format src tests

echo "Python formatting complete."
