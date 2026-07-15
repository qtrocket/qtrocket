# QtRocket

[![CI](https://github.com/cthunter01/qtrocket/actions/workflows/cmake-multi-platform.yml/badge.svg?branch=development)](https://github.com/cthunter01/qtrocket/actions/workflows/cmake-multi-platform.yml)

An open-source model-rocket flight simulator: a C++23 simulation core shared by a Qt6 Widgets
GUI, a scriptable headless REPL, and a standalone OpenGL design viewer.

![qtrocket-visualizer rendering a 75 mm multi-part design](docs/images/qtrocket-visualizer.png)

## What it does today

QtRocket simulates 3-DOF point-mass flights of single-stage hobby rockets, built honestly on
real numerics:

- **Integration** — fixed-step RK4 or adaptive Runge–Kutta–Fehlberg 4(5) with embedded error
  control, selectable at runtime behind one `DESolver` interface.
- **Atmosphere** — the full seven-layer US Standard Atmosphere 1976 (density, pressure,
  temperature, speed of sound, viscosity; the NOAA source document is in `docs/`), plus
  constant and vacuum models.
- **Gravity** — constant g₀ or Newtonian inverse-square over a spherical geoid.
- **Mass properties** — a composite part tree (nose cones, body tubes, fin sets) with
  closed-form inertia tensors, parallel-axis composition, and time-varying mass/CG/inertia
  during the motor burn, recomputed from the thrust curve each step.
- **Motors** — three ingest paths behind one database: RockSim `.rse` files, RASP `.eng`
  files, and a live [thrustcurve.org](https://www.thrustcurve.org) REST client.
- **Designs** — versioned `.qrd` XML persistence with placement expressed as physical intent
  (seat/station links), resolved by a geometry solver that hard-fails self-intersecting
  designs.

The current scope is deliberate: **3-DOF today, with the 6-DOF seams already in place** —
quaternion state carried in `StateData`, a `DESolver<Quaternion>` slot in the propagator, a
torque interface on the model, and full composite inertia tensors recorded every step.
Barrowman stability math (per-part CN<sub>α</sub> and center-of-pressure composition) is
implemented and test-verified; surfacing it in the CLI and GUI is the next milestone. The
[roadmap](TODO.md) tracks what's real versus planned.

## Sixty seconds in the REPL

`qtrocket-cli` drives the whole engine headlessly — interactively, from a pipe, from a script
file, or via `-c "<command>"`. Exit status is honest: 0 only if every command succeeded.

```console
$ qtrocket-cli demo.cli
# QtRocket CLI - type 'help' for commands, 'quit' to exit.
OK loadmotors: 249 motors from data/Aerotech.rse
OK newdesign: root NoseCone id=2
OK addpart: BodyTube id=3 under 2
OK addpart: FinSet id=4 under 3
OK checkdesign: 3 parts, contiguous, no overlaps
OK setmotor: G80T
OK launch: 2703 steps, t_final=27.020s
  apogee    = 770.498 m @ t=9.060s
  max_speed = 286.154 m/s @ t=1.230s
  downrange = 0.000 m
  landing   = t=27.020s, pos=(0.000, 0.000, -0.233)
CSV <working-dir>/qtrocket_run.csv
OK bye
```

The GUI covers the same engine — open/save `.qrd` designs with a live part tree, import or
search motors online, run flights, and inspect altitude/velocity plots — and
`qtrocket-visualizer` renders any `.qrd` in 3D using the same placement solver the simulator
flies, highlighting overlap offenders in red.

## Architecture

```mermaid
graph TD
    GUI["gui/ — Qt6 Widgets (the only Qt-dependent code)"] --> CORE
    CLI["cli/ — headless REPL"] --> CORE
    VIZ["visualizer/ — OpenGL .qrd viewer"] --> MODEL
    CORE["core/QtRocket — controller"] --> MODEL["core/model — rocket description"]
    CORE --> SIM["core/sim — propagation & environment"]
    MODEL --> UTILS["core/utils — logging, math types, HTTP"]
    SIM --> UTILS
```

Each layer depends only on those below it, and the seams are interfaces on purpose:
`Propagatable` bridges model and sim, `DESolver<T>` makes integrators interchangeable,
`AtmosphericModel`/`GravityModel` are pluggable physics, and the thrustcurve.org client sits
behind an interface so motor search is testable without a network. A 58-page architecture
review of exactly what is implemented, what is dormant, and why lives in
[docs/ArchitectureReviewLatex](docs/ArchitectureReviewLatex/ArchitectureReview.pdf).

## Verification

- **248 automated tests** (GoogleTest, six suites) at a >1:1 test-to-source ratio for the
  core — including physics oracles: apogee must rise monotonically with total impulse across
  a ¼A→M motor ladder flown through the real REPL, drag must reduce apogee versus vacuum for
  both integrators, and RKF45 must track RK4.
- **Six CI configurations run the full suite on every push**: Linux GCC, Linux Clang, Linux
  ASan+UBSan (dependencies instrumented, UBSan findings fatal), macOS, Windows MSVC, and
  FreeBSD 15 booted in a QEMU VM — plus a clang-tidy job with findings-as-errors over every
  hand-written translation unit.
- **Warnings are errors** on all four compilers (`-Wall -Wextra -Wpedantic -Werror`, `/W4 /WX`).
- **Coverage** via LLVM source-based instrumentation (`cmake --build --preset coverage-clang
  --target coverage`): 90% function / 74% line on the Qt-free core at last report.
- A committed numeric baseline pins composite mass/CG/inertia/CP for all 24 design fixtures
  to ULP-level tolerance; the fixtures regenerate from committed CLI scripts.

## Building

Qt6 is the only system dependency — everything else (GoogleTest, Eigen, Boost.property_tree,
jsoncpp, curl) is fetched and pinned by CMake. First configure compiles them, so it is slow
once.

```bash
cmake --preset release            # or debug, debug-clang, release-gcc, ...
cmake --build --preset release
ctest --preset release -R 'qtrocket_*'
```

Executables land in `build/gui/qtrocket`, `build/cli/qtrocket-cli`, and
`build/visualizer/qtrocket-visualizer`. Sanitizer builds use `--preset asan-clang`;
`scripts/run-tidy.sh` runs the same clang-tidy sweep CI does.

## Documentation

Three LaTeX whitepapers (source and PDF committed) document the design at depth:

- [Architecture Review](docs/ArchitectureReviewLatex/ArchitectureReview.pdf) — a whole-system
  capability and gap review, including a 16-area feature comparison against OpenRocket and a
  ranked deficiency inventory.
- [Design Whitepaper](docs/DesignWhitepaperLatex/DesignWhitepaper.pdf) — the system design
  reference, with a physics primer and the rationale behind the major seams.
- [Part Placement Design](docs/PartPlacementDesignLatex/PartPlacement.pdf) — the station-pair
  placement model: how designs store physical intent rather than baked coordinates, and the
  resolver that turns it into geometry.

## Development practices

Since June 2026, QtRocket has been developed with AI assistance. Prior to that, 
QtRocket began in February 2023 and was designed and built entirely by hand for its first
three years: the layered architecture and its interface seams, the simulation core, the Qt
GUI, the motor database and ingest pipeline, and the multi-platform CI all predate any AI
involvement. Since mid-2026 I have used AI
assistance to accelerate work on top of that foundation, mostly in the headless REPL,
the OpenGL visualizer, and automated the generation of LaTeX design documentation. All AI work
is spec-first (the
build-ready specs live in `docs/`) and held to the same gates as everything else in the
tree. All builds must be warning-clean `-Werror`, the full test suite, sanitizers, and clang-tidy,
and the architecture documents are verified against the source citation-by-citation rather
than taken on faith. The design decisions are still human.

## License

[GPL-3.0](LICENSE). Copyright © 2022–2026 Travis Hunter.
