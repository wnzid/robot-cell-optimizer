#!/usr/bin/env bash

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$ROOT/.venv"

if [[ ! -x "$VENV/bin/ruff" ]]; then
  echo "Ruff not found in $VENV."
  echo "Run: .venv/bin/python -m pip install -e 'app/backend[dev]'"
  exit 1
fi

cd "$ROOT/app/backend"

"$VENV/bin/ruff" check --fix src tests
"$VENV/bin/ruff" format src tests

echo "Python formatting complete."
