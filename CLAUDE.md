# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

QtRocket is an open-source model rocket simulator: a Qt6 Widgets GUI (`qtrocket`) and a Qt-free headless REPL (`qtrocket-cli`) built on a shared C++23 simulation core. Default branch is `development`.

## Build

```bash
cmake --preset debug-clang             # configure (Ninja + Debug + clang, build/ dir)
cmake --build --preset debug-clang     # build everything
```

- Generator/compiler/build-type are pinned by `CMakePresets.json` (Ninja everywhere — CLI, VS Code CMake Tools, and the .vscode tasks all use the presets). Use the `debug` preset for the default system compiler. **Don't configure with ad-hoc `cmake -B build` invocations** — mixing generators in an existing `build/` breaks the FetchContent sub-builds.

- All dependencies except Qt6 (system-installed) are pulled via FetchContent (GoogleTest, jsoncpp, curl, Eigen, Boost.property_tree), so the **first configure/build downloads and compiles them — it is slow**. Don't delete `build/` casually.
- Executables: `build/qtrocket` (GUI), `build/qtrocket-cli` (headless REPL — useful for exercising the sim core without a display).
- `compile_commands.json` is exported in `build/`.

## Tests

GoogleTest, five aggregate suites, all registered with ctest under names matching `qtrocket_*`:

```bash
ctest --test-dir build -R 'qtrocket_*'                 # all suites (what CI runs)
ctest --test-dir build -R PartTests                    # tests are also discovered individually via gtest_discover_tests
./build/model/tests/model_tests                        # Part composition / inertia tests
./build/sim/tests/sim_tests                            # RK45 solver, US Standard Atmosphere tests
./build/tests/integration_tests                        # end-to-end physics + motor DB persistence
./build/tests/propagator_tests                         # focused Propagator behavior
./build/tests/cli_tests                                # CLI/REPL command behavior
./build/model/tests/model_tests --gtest_filter='PartTests.Clone*'   # single test
```

Test sources live next to what they test: `model/tests/`, `sim/tests/`, and top-level `tests/` for integration.

## Coverage

Coverage uses Clang/LLVM source-based coverage (`llvm-cov` + `llvm-profdata`) and a separate `build-coverage/` tree, so it does not disturb the normal `build/` directory:

```bash
cmake --preset coverage-clang
cmake --build --preset coverage-clang --target coverage
```

The `coverage` target builds the instrumented test binaries, runs `ctest -R '^qtrocket_.*tests$'`, merges the raw profiles, prints the summary, and writes:

- Text summary: `build-coverage/coverage/summary.txt`
- HTML report: `build-coverage/coverage/html/index.html`

`QTROCKET_ENABLE_COVERAGE=ON` intentionally requires Clang/AppleClang. GCC remains supported through the normal `debug-gcc` and `release-gcc` presets, but not for this `llvm-cov` coverage target.

## Architecture

Layering (each layer only depends on those below it):

```
gui/ (Qt6 Widgets — the ONLY Qt-dependent code; GuiRunner owns QApplication)
cli/ (readline-style REPL, no Qt)
QtRocket.h/.cpp (controller singleton, Qt-free)
model/  +  sim/
utils/
```

- **QtRocket** is the master controller singleton: owns the RocketModel/Propagator pairs, Environment, and MotorModelDatabase; entry points are `launchRocket()`, `setInitialState()`, `getStates()`.
- **`qtrocket_core`**: `QtRocket.cpp` is built once into the `qtrocket_core` static lib (`PUBLIC model sim utils`), linked by all three executables (`qtrocket`, `qtrocket-cli`, `integration_tests`) — **new executables link `qtrocket_core`**. (Until June 2026 it was compiled directly into every executable to dodge a `utils → QtRocket::getInstance()` back-call cycle; those back-calls are gone — `Environment` is injected via the Propagator, and `MotorModelDatabase` logs via `Logger::getInstance()`.)
- **model/** — physical description of the rocket:
  - `RocketModel` implements `Propagatable` (the model↔sim bridge interface: `getForces()`, `getTorques()`, `getMass(t)`, `terminateCondition()`...).
  - `Part` (`model/Part.h`) is the base of a composite tree of rocket components (concrete types in `model/parts/`, e.g. `HollowSphere`). Composite mass/CM/inertia are recomputed lazily via a dirty-flag that propagates up the tree; child inertia tensors are shifted to the composite CM with the parallel-axis theorem. Parts store *per-unit-mass* geometric tensors (m²); composites are full mass-weighted tensors (kg·m²). Children are added by move; `clone()` deep-copies.
  - `MotorModel` + `ThrustCurve`: time-aware thrust and burning mass (ignition at t=0, linear interpolation between thrust samples).
  - `MotorModelDatabase` (motor storage/search, XML save/load) with two ingest paths: local RockSim `.rse` files (`RSEDatabaseLoader`, Boost property_tree XML) and the thrustcurve.org REST API (`ThrustCurveAPI`, jsoncpp + `utils::CurlConnection`). Clients (GUI/CLI) go through the database; they never touch the loader or API directly. Bundled motor data lives in `data/` (tests locate it via the `QTROCKET_DATA_DIR` compile definition).
- **sim/** — the numerics:
  - `Propagator` drives the ODE loop (`runUntilTerminate()` until altitude z < 0) and records the state history.
  - `Integrator` selects the `DESolver` backend at runtime: `RK4Solver` (fixed step) or `RK45Solver` (adaptive Runge-Kutta-Fehlberg).
  - `Environment` holds pluggable physics models: `GravityModel` (Constant / Spherical), `AtmosphericModel` (Constant / USStandardAtmosphere / Vacuum), geoid model.
  - `StateData` carries position/velocity plus quaternion orientation, but currently only the 3 linear DOF are integrated (6-DOF is planned).
- **utils/** — `Logger` singleton (stdout + `log.txt`), math typedefs over Eigen (`Vector3`, `Matrix3`, `Quaternion`), and `CurlConnection` (libcurl HTTP GET wrapper: TLS verification, timeouts, status checks). The bottom layer: utils must never include model/ or sim/ headers — that's what keeps the dependency graph acyclic.

## Conventions

- SI units throughout: meters, kilograms, seconds, Newtons; altitude is the z component, ground at z = 0.
- Quaternions are stored (x, y, z, w).
- GUI forms are `.ui` files compiled via AUTOUIC; resources via `qtrocket.qrc` (AUTORCC); `Q_OBJECT` classes need AUTOMOC — all already enabled globally.
