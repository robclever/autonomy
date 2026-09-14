# slam_sim

A learning sandbox for **autonomy applied to aircraft**, written in C++.

The goal is to stage and understand the pieces of an autonomous aircraft in a
simulated world — from how an aircraft senses its environment, through sensor
fusion and SLAM (Simultaneous Localization and Mapping), to higher-level
autonomy that plans and acts on the resulting understanding.

## Intent

This project is a *learning vehicle*. It is intentionally modular so each
part can be studied, modified, and tested in isolation. 

The core idea is a clean separation between **absolute truth** and **aircraft belief**:

- The **environment** models the world and holds the *ground truth*. Each tick
  it emits the structured observations a real aircraft would perceive from its
  current state (nearby landmarks, distances, bearings).
- The **aircraft platform** owns its pose and motion, and advances through the
  world based on the commands it receives from autonomy.
- **Sensors** receive those true observations and degrade them with realistic
  noise, bias, and dropouts — producing what the aircraft *actually measures*
  (infrared, radar, and eventually IMU, GPS, camera).
- **Fusion** combines the noisy sensors into a coherent state estimate.
- **Autonomy** consumes the sensor/fusion output to run SLAM — building a map
  and localizing within it — and then plans both the immediate trajectory and
  higher-level decisions.

Because we keep ground truth around for comparison, we can measure how well
SLAM and autonomy actually perform.

## Simulation flow (one tick)

```
environment ──(true observations)──▶ sensors ──(noisy measurements)──▶ fusion
                                                                         │
                                   platform ◀──(commands)── autonomy ◀───┘
                                                                        │
                                     │              SLAM: map + localize
                                     └──(recompute observations next tick)──┘
```

## Layout

```
environment/   ground-truth world model (Landmark + World) and observation
               generation: structured body-frame observations from a pose
platform/      aircraft pose, kinematics, and dynamics
systems/
  sensors/     per-sensor models (infrared, radar, imu, gps, camera, ...)
  fusion/      sensor fusion / state estimation (e.g. EKF/UKF)
  autonomy/    SLAM (mapping + localization), path planning, guidance, control
core/
  include/
    math/        minimal Vec3 + Quaternion (dependency-free for now)
    kinematics/  NED/body frames, Pose + Transform
    time/        deterministic step-based clock
    measurement/ angle types
    (later:      message types, noise models)
sim/           simulation loop / orchestrator
evaluation/    metrics comparing estimates to ground truth
visualization/ map & trajectory viewers
config/        scenario and parameter definitions
tests/         unit tests (assert-based, wired into CTest and `make tests`)
```

## Guiding principles

- **Deterministic, step-based simulation** — reproducible and unit-testable.
- **Modular systems with clean interfaces** — swap and test each in isolation.
- **Ground truth always kept for comparison** — so we can measure learning.
- **Coordinate frames explicit** (NED / body) and handled centrally in `core/`.

## Tooling

- Build: canonical `CMakeLists.txt` (C++17, CTest); a plain `Makefile` builds
  the same targets when cmake isn't installed. Try `make`, then `make tests`.
- Formatting: `.clang-format` (`make format` / `make check-format`).
- CI: `.github/workflows/ci.yml` — builds, runs tests, checks formatting, and
  verifies dependency layering on every push / pull request.

### Running CI locally

The CI workflow steps can be run manually:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
clang-format --dry-run --Werror $(git ls-files '*.cpp' '*.h')
python3 scripts/check_deps.py
```

To run the actual GitHub Actions workflow locally, install
[act](https://github.com/nektos/act) (`brew install act`) and run:

```bash
act
```

## Status

Bootstrapped. Build (CMake + plain-make fallback), formatting, CI, a
deterministic clock, `core/` math + NED/body frames, and a first environment
model (Landmark + World emitting structured observations) are in place.

`systems/sensors/*`, `systems/fusion`, `systems/autonomy`, and `platform/` are
still stubs. Next steps: first sensor models (adding noise/bias to World
observations), platform kinematics/attitude dynamics, then the simulation loop
and a first fusion/SLAM stage.
