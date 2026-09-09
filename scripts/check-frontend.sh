#!/usr/bin/env bash

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FRONTEND="$ROOT/app/frontend"

if ! command -v node >/dev/null 2>&1; then
  echo "Node.js not found."
  echo "Run: nvm use"
  exit 1
fi

if ! command -v npm >/dev/null 2>&1; then
  echo "npm not found."
  exit 1
fi

if [[ ! -f "$FRONTEND/package.json" ]]; then
  echo "Frontend package.json not found."
  exit 1
fi

if [[ ! -d "$FRONTEND/node_modules" ]]; then
  echo "Frontend dependencies are not installed."
  echo "Run:"
  echo "  cd app/frontend"
  echo "  npm ci"
  exit 1
fi

cd "$FRONTEND"

echo "=== TypeScript ==="
npm run typecheck

echo
echo "=== ESLint ==="
npm run lint

echo
echo "=== Vitest ==="
npm run test

echo
echo "=== Production build ==="
npm run build

echo
echo "Frontend quality checks passed."
