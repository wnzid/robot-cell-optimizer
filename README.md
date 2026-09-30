<div align="center">

# Robot Cell Optimizer

**Reproducible reachability and motion-analysis tooling for simulated industrial workcells.**

`ROS 2 Jazzy` · `MoveIt 2` · `C++` · `Gazebo Harmonic` · `React`

</div>

Robot Cell Optimizer is an engineering platform for analysing target reachability and, over time, optimizing the placement and motion of a simulated industrial robot. The repository combines ROS 2 packages, C++ analysis nodes, a Python application boundary, a React interface, configuration, benchmarks, and repeatable quality scripts.

## Current milestone

Given a set of Cartesian target poses, determine which targets are reachable by the configured robot model and report the result through reproducible scenarios.

## Planned analysis surface

- Cartesian target reachability and inverse kinematics
- Joint-limit and singularity analysis
- Manipulability scoring
- Collision-aware motion planning
- Cycle-time estimation
- Robot-base placement optimization

Planned items are intentionally separated from the implemented milestone.

## Validated environment

- Ubuntu 24.04
- ROS 2 Jazzy and MoveIt 2
- Universal Robots ROS 2 packages
- Gazebo Harmonic
- GCC/G++ 13 and CMake 3.28+
- Python 3.12
- Node.js version pinned by `.nvmrc`

## Setup

```bash
nvm install
nvm use
./scripts/setup.sh
./scripts/build.sh
./scripts/test.sh
```

Useful quality commands:

```bash
./scripts/check-format.sh
./scripts/check-clang-tidy.sh
./scripts/build-sanitized.sh
./scripts/environment-report.sh
```

## Repository layout

```text
ros2_ws/src/    ROS 2 and C++ packages
app/backend/    Python application backend
app/frontend/   React/TypeScript interface
config/         Shared robot and analysis configuration
docs/           Development and architecture documentation
scripts/        Canonical setup, build, test, and analysis commands
tests/          Repository-level checks
```

See [docs/development.md](docs/development.md) for the complete development workflow.

## License

The `LICENSE` file is currently empty, so no redistribution terms are declared.
