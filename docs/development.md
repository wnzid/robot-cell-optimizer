# Development Guide

This document defines the validated development environment and the canonical
setup, build, test, quality, and debugging workflow for Robot Cell Optimizer.

## Validated environment

The project is developed and validated with:

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- MoveIt 2 for ROS 2 Jazzy
- Universal Robots ROS 2 packages
- Gazebo Harmonic
- GCC/G++ 13
- CMake 3.28+
- Python 3.12
- Node.js pinned by `.nvmrc`
- npm dependencies pinned by `package-lock.json`

For exact installed versions, run:

    ./scripts/environment-report.sh

## Repository layout

    robot-cell-optimizer/
    ├── ros2_ws/src/       ROS 2 and C++ packages
    ├── app/backend/       Python application backend
    ├── app/frontend/      React/TypeScript frontend
    ├── assets/            Workcell assets
    ├── benchmarks/        Reproducible benchmark scenarios
    ├── config/            Shared configuration
    ├── docs/              Public documentation
    ├── scripts/           Canonical development commands
    └── tests/             Repository-level tests

## Initial setup

Clone the repository and enter it:

    git clone <repository-url>
    cd robot-cell-optimizer

Ensure ROS 2 Jazzy is installed at:

    /opt/ros/jazzy

If Node is managed with NVM, load the pinned version:

    nvm install
    nvm use

Then run:

    ./scripts/setup.sh

The setup script:

- sources ROS 2 Jazzy;
- installs ROS dependencies through rosdep;
- creates `.venv` when required;
- installs locked Python development dependencies;
- installs the editable backend package;
- validates the Node.js version;
- installs frontend packages using `npm ci`;
- installs the pre-commit hook.

## ROS build

Build the complete ROS workspace with:

    ./scripts/build.sh

The normal development build uses `RelWithDebInfo`.

Generated files live under:

    ros2_ws/build/
    ros2_ws/install/
    ros2_ws/log/

These directories are not committed.

## ROS tests

Run all ROS/C++ tests:

    ./scripts/test.sh

The test command also reports the complete colcon test result summary.

## C++ formatting

Apply the project formatter:

    ./scripts/format.sh

Check formatting without modifying source files:

    ./scripts/check-format.sh

## C++ static analysis

Run clang-tidy:

    ./scripts/check-clang-tidy.sh

The standard ROS build must exist first because clang-tidy consumes the generated
compile database.

## Sanitizers

Build the C++ workspace with AddressSanitizer and UndefinedBehaviorSanitizer:

    ./scripts/build-sanitized.sh

Sanitized artifacts use independent directories:

    ros2_ws/build-sanitized/
    ros2_ws/install-sanitized/
    ros2_ws/log-sanitized/

This keeps sanitizer instrumentation separate from the normal development build.

## Python environment

The backend development environment lives in:

    .venv/

Python development dependencies are pinned in:

    app/backend/requirements-dev.lock

ROS modifies `PYTHONPATH`, so backend tooling deliberately removes `PYTHONPATH`
before execution. This prevents ROS Python packages from leaking into the isolated
application environment.

Run all backend checks:

    ./scripts/check-python.sh

This includes:

- Ruff linting
- Ruff formatting verification
- mypy strict type checking
- pytest

Apply Python formatting with:

    ./scripts/format-python.sh

## Frontend

The frontend is located in:

    app/frontend/

The expected Node.js version is stored in:

    .nvmrc

Install exactly the locked dependency tree with:

    cd app/frontend
    npm ci
    cd ../..

Run all frontend quality gates:

    ./scripts/check-frontend.sh

This includes:

- strict TypeScript checking
- ESLint
- Vitest
- Vite production build

## Full project quality gate

The canonical local verification command is:

    ./scripts/quality.sh

It checks:

1. C++ formatting
2. ROS/C++ build
3. ROS/C++ tests
4. clang-tidy
5. Python quality
6. frontend quality
7. Git whitespace

Successful execution ends with:

    ALL QUALITY GATES PASSED

## Pre-commit

Install the Git hook with:

    .venv/bin/pre-commit install

Run all configured hooks manually:

    .venv/bin/pre-commit run --all-files

## Environment report

Generate a development environment report with:

    ./scripts/environment-report.sh

This reports the important local versions for:

- operating system
- Linux kernel
- ROS
- GCC/G++
- CMake
- Python
- Node/npm
- MoveIt
- Universal Robots packages
- Gazebo
- Git

## Continuous integration

GitHub Actions is defined in:

    .github/workflows/ci.yml

CI validates three independent environments:

- ROS 2 / C++
- Python
- Frontend

Fresh CI environments are important because they expose undeclared dependencies
that may otherwise be hidden by an existing developer workstation.

## Fresh-shell verification

Before declaring a foundation or release milestone complete, open a fresh terminal
and run:

    cd ~/projects/robot-cell-optimizer
    source /opt/ros/jazzy/setup.bash
    nvm use
    ./scripts/quality.sh

Then verify repository integrity:

    git diff --check
    git status --short --untracked-files=all

For the Phase 0 foundation, also verify the sanitizer build:

    ./scripts/build-sanitized.sh

## Private development context

Local coding-agent context is intentionally excluded from Git.

The following files/directories must remain private:

    AGENTS.md
    .agent/

They are covered by `.gitignore` and are not part of the public repository.
