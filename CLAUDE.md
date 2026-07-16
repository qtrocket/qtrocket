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
- Executables: `build/gui/qtrocket` (GUI), `build/cli/qtrocket-cli` (headless REPL — useful for exercising the sim core without a display; takes a script-file arg or `-c "<command>"`, and exits 1 if any command reported ERR, so piped scripts are CI-consumable).
- `compile_commands.json` is exported in `build/`.

## Tests

GoogleTest, six test binaries, all registered with ctest under names matching `qtrocket_*`:

```bash
ctest --test-dir build -R 'qtrocket_*'                 # all suites (what CI runs)
ctest --test-dir build -R 'qtrocket_*' -LE heavy       # fast loop: skip the full-ladder flight sweeps
ctest --test-dir build -R 'qtrocket_*' -L heavy        # ONLY the heavy full-ladder flight sweeps
ctest --test-dir build -R PartTests                    # tests are also discovered individually via gtest_discover_tests
./build/core/model/tests/model_tests                   # Part composition / inertia tests
./build/core/sim/tests/sim_tests                       # RK45 solver, US Standard Atmosphere tests
./build/tests/integration_tests                        # end-to-end physics + motor DB persistence
./build/tests/propagator_tests                         # focused Propagator behavior
./build/tests/cli_tests                                # CLI/REPL command behavior
./build/tests/design_matrix_tests                      # CLI design/persistence/part-type + 1/4A->M flight matrix
./build/core/model/tests/model_tests --gtest_filter='PartTests.Clone*'   # single test
```

The `design_matrix` binary is registered with ctest twice: `qtrocket_design_matrix_tests` flies one motor per class for the flight sweeps (fast smoke), and `qtrocket_design_matrix_heavy_tests` (label `heavy`, sets `QTROCKET_FULL_LADDER=1`) re-runs the `FlightMatrix`/`Atmosphere` sweeps over each class's complete 1/4A→M motor ladder. So `-LE heavy` is the fast loop and `-L heavy` is the full sweep; both are covered by a plain `-R 'qtrocket_*'`.

Test sources live next to what they test: `core/model/tests/`, `core/sim/tests/`, and top-level `tests/` for integration.

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

## Sanitizers and clang-tidy

Both run in CI (`linux-clang-asan.yml` and `clang-tidy.yml` — CI is one workflow file per configuration under `.github/workflows/`, sharing steps via `.github/actions/build-and-test`); keep them green locally before pushing:

```bash
cmake --preset asan-clang                       # separate build-asan/ tree
cmake --build --preset asan-clang
ctest --preset asan-clang -R 'qtrocket_*'       # full suite under ASan+UBSan
scripts/run-tidy.sh                             # clang-tidy over all hand-written TUs (needs a built build/)
```

- `QTROCKET_ENABLE_SANITIZERS=ON` (what the `asan-clang` preset sets) instruments the FetchContent deps too, and makes UBSan findings fatal (`-fno-sanitize-recover`), so a passing ctest means genuinely clean.
- clang-tidy config is layered: root `.clang-tidy` (curated bugprone/performance/analyzer set, findings are errors) plus per-test-dir overrides (`tests/.clang-tidy` etc.) that disable `bugprone-unchecked-optional-access` — gtest `ASSERT_*` guards defeat its flow analysis. `ExcludeHeaderFilterRegex` needs clang-tidy >= 19.
- run `run-clang-tidy -fix` single-threaded only: parallel fixit application double-applies edits in headers seen from multiple TUs.

## Architecture

Layering (each layer only depends on those below it):

```
gui/ (Qt6 Widgets — the ONLY Qt-dependent code)
cli/ (readline-style REPL, no Qt)
core/QtRocket.h/.cpp (controller singleton, Qt-free)
core/model/  +  core/sim/
core/utils/
```

Includes for core code are rooted at `core/` (`#include "model/..."`, `"sim/..."`, `"utils/..."`); each core lib exports that root as a `PUBLIC` include dir.

- **QtRocket** is the master controller singleton: owns the RocketModel/Propagator pairs, Environment, and MotorModelDatabase; entry points are `launchRocket()`, `setInitialState()`, `getStates()`.
- **`qtrocket_core`**: `QtRocket.cpp` is built once into the `qtrocket_core` static lib (`PUBLIC model sim utils`), linked by all three executables (`qtrocket`, `qtrocket-cli`, `integration_tests`) — **new executables link `qtrocket_core`**. (Until June 2026 it was compiled directly into every executable to dodge a `utils → QtRocket::getInstance()` back-call cycle; those back-calls are gone — `Environment` is injected via the Propagator, and `MotorModelDatabase` logs via `Logger::getInstance()`.)
- **model/** — physical description of the rocket:
  - `RocketModel` implements `Propagatable` (the model↔sim bridge interface: `getForces()`, `getTorques()`, `getMass(t)`, `terminateCondition()`...).
  - `Part` (`model/Part.h`) is the base of a composite tree of rocket components (concrete types in `model/parts/`, e.g. `HollowSphere`). Composite mass/CM/inertia are recomputed lazily via a dirty-flag that propagates up the tree; child inertia tensors are shifted to the composite CM with the parallel-axis theorem. Parts store *per-unit-mass* geometric tensors (m²); composites are full mass-weighted tensors (kg·m²). Children are added by move; `clone()` deep-copies.
  - `MotorModel` + `ThrustCurve`: time-aware thrust and burning mass (ignition at t=0, linear interpolation between thrust samples).
  - `MotorModelDatabase` (motor storage/search, XML save/load) with two ingest paths: local RockSim `.rse` files (`RSEDatabaseLoader`, Boost property_tree XML) and the thrustcurve.org REST client (`ThrustCurveClient`, behind the internal `ThrustCurveAPI` interface, jsoncpp + `utils::CurlConnection`). Clients (GUI/CLI) go through the database; they never touch the loader or remote client directly. Bundled motor data lives in `data/` (tests locate it via the `QTROCKET_DATA_DIR` compile definition).
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

## Comments

Comment the *why*, not the *what*. Prefer self-documenting names and types over prose. Keep comments
terse, plain-spoken, and lowercase — no marketing tone, no emphasis-by-capitalization. The goal is code
that reads like a human wrote it: brief where brief suffices, a couple of dense lines where real
rationale is needed. Terse exemplars already in the tree: `sim/GravityModel.h`, `utils/math/MathTypes.h`.

Do:
- Put one short `@brief` (≤2 lines) on non-trivial public API: purpose, units, and any non-obvious
  contract (ownership, nullability, clamping, who calls it).
- Keep genuinely non-obvious domain/math notes — the physics the code can't show (e.g. a silhouette
  formula, Barrowman rationale).
- Compress architectural rationale to 2-3 dense lines; state the invariant in one plain clause. If it
  needs more than that, it belongs in `docs/`, not a header.
- Preserve structural/legal comments: `/// \cond` / `/// \endcond`, license headers, and an
  include-purpose note only when the reason is non-obvious.

Don't:
- No Doxygen block on a trivial getter/setter/field whose name and type already say it (`double
  getLength() const;` needs nothing; at most a trailing `///< units`).
- No comment that restates the code (`return inertiaTensor; ///< returns the inertia tensor`).
- No multi-paragraph essays or design-doc prose in headers.
- No `whitepaper X.Y` / `Step N` / `P5`/`P6` phase-tag pointers as the reason — if a rationale matters,
  state it in one plain clause instead of pointing elsewhere.
- No process or past-tense narration ("previously x_c was reported from the end", "lands at Step 6",
  "this used to…"). Comment the code as it is, not its history, and don't narrate changes.
- No ALL-CAPS emphasis words; state hard invariants plainly.
