# QtRocket — Architectural Re-Analysis (Post-P0)

**Date:** 2026-06-07 · **Branch:** `development` · **Repo:** `/home/travis/Development/qtrocket`

**Purpose:** Re-scan the codebase after the P0 milestone (and three of the five P1 items) were completed, focusing on (a) features added since the prior scan — chiefly a **headless CLI** — (b) **GUI ↔ CLI feature parity and inconsistencies**, and (c) any **regressions or new issues** introduced by the P0/P1 work.

**Relationship to prior docs:** This is a *delta* document. The baseline is [ANALYSIS_RESULTS.md](docs/ANALYSIS_RESULTS.md) (2026-06-05), left unchanged as a record. The roadmap is [TODO.md](TODO.md). Every claim below was re-verified against the working tree on 2026-06-07, and the findings were exercised against a **clean build + all 18 CTest cases passing** and a **live `qtrocket-cli` run** (see [§10](#10-verification-appendix)).

**One-line status:** The simulator went from *"a vacuum, from-rest, straight-up point mass with three dead GUI inputs"* to *"a drag-aware, atmosphere-coupled, input-respecting point-mass trajectory with a scriptable headless driver and real automated physics tests."* That is a large, genuine step. The new surface area also introduced a **parity gap between the CLI and the GUI** — several capabilities now exist only in the CLI, and a couple of GUI inputs are dead or dangerous.

---

## 0. Executive summary (what's different)

**Completed since the prior scan** (all P0, plus P1 drag/atmosphere/tests — confirmed against source and by running):

- **Initial state is now honored.** `RocketModel::launch()` seeds `currentState` from `initialState` ([model/RocketModel.cpp:86-90](model/RocketModel.cpp#L86-L90)). A 45° / 30 m/s launch produces **251 m of downrange** in the CLI (was identically 0 before) — the single highest-value fix, verified end-to-end.
- **Mass / drag / reference area are real.** `setMass`, `setDragCoefficient`, `setReferenceArea` have backing members and are consumed by the force model ([model/RocketModel.h:109-145](model/RocketModel.h#L109-L145)).
- **Aerodynamic drag + atmosphere coupling.** `getForces` now computes `−½·ρ(alt)·|v|·v·Cd·A`, pulling ρ from the active atmosphere ([model/RocketModel.cpp:42-73](model/RocketModel.cpp#L42-L73)). Drag roughly **halves apogee** vs. vacuum (1940 m → 847 m, verified). A `VacuumAtmosphere` baseline was added.
- **RK4 trial-state fix.** `getForces(t, position, velocity)` is evaluated at each RK4 stage state ([sim/Propagator.cpp:29-40](sim/Propagator.cpp#L29-L40)), so velocity/altitude-dependent drag integrates at full RK4 accuracy.
- **Timestep actually propagates** to the integrator ([sim/Propagator.h:54-65](sim/Propagator.h#L54-L65)).
- **A headless CLI/REPL** (`qtrocket-cli`) drives the same engine without Qt — the biggest new feature ([cli/Repl.cpp](cli/Repl.cpp)).
- **Motor sourcing unified** behind `utils::MotorModelDatabase` (RSE import, thrustcurve.org search, list/get, save **and load**) with source-agnostic DTOs ([utils/MotorModelDatabase.h](utils/MotorModelDatabase.h)).
- **RK45Solver finished** as an adaptive RKF45 behind a redesigned `DESolver` interface, **unit-tested** ([sim/RK45Solver.h](sim/RK45Solver.h), [sim/tests/RK45SolverTests.cpp](sim/tests/RK45SolverTests.cpp)).
- **Real automated coverage:** drag/terminal-velocity/timestep physics tests and a motor-DB save→load round-trip ([tests/](tests/)).
- **Dead code removed:** `model/MotorModelDatabase.{h,cpp}`, `QtRocket::states`, `launchSitePosition`, `runSim()`. Klima/Quest enum swap fixed; ThrustCurveAPI debug spam removed.

**The headline caveat:** the GUI did **not** keep pace with the CLI. The launch-angle input is **disabled** (so the GUI still always flies straight up despite the fix that made angle work), there is **no reference-area input**, the MainWindow **timestep field is dead**, an **integrator dropdown is dead**, and an unvalidated **timestep of 0 hangs the run loop**. Details in [§3](#3-gui--cli-feature-parity-the-core-of-this-review) and [§4](#4-issues-found-regressions-traps-dead-code).

**Net maturity:** the physics is no longer "a stub" — it is a *believable single-stage point-mass ascent under drag*. The remaining big rocks are unchanged from the prior scan: **component geometry**, **Barrowman CP/stability**, **6-DOF**, and **recovery**.

---

## 1. What changed since the prior scan (commit-mapped)

`git diff --stat 59efd88..HEAD` = **43 files, +2794 / −316**. The substantive commits:

| Commit | Theme | TODO item |
|---|---|---|
| `9aab4d2` | `minFlightTime = 4 s` guard; don't terminate at step 0 below ground | (pre-P0 groundwork) |
| `52666f7` | Seed `currentState` from `initialState` | P0 |
| `b0dcece` | **Add `qtrocket-cli`**; fix `setTimeStep` propagation | P0 + new feature |
| `3204aa8` | Drag + atmosphere selection, `VacuumAtmosphere`, reference area, RK4 trial-state fix, env-list bugfix, physics tests | P1 (×3) + P0 |
| `be1364e` | Unify motor selection behind `utils::MotorModelDatabase` | P0 (dead-code) |
| `ee0f480` | Fix swapped Klima/Quest manufacturer enum | P0 |
| `ecc0d4d` | Implement motor-DB **load**; wire save/load into CLI + GUI | P5 |
| `9dc211a` | Put thrustcurve.org behind `MotorModelDatabase` | P0/refactor |
| `fda3d40` | Enable "Calculate Trajectory" from the thrustcurve.org path | P0 |
| `12619a8` | Finish `RK45Solver` as an adaptive `DESolver` | P0 (decide RK45) |
| `d745cef` | Remove dead `QtRocket::states` / `launchSitePosition` | P0 |

The work also moved the GUI launcher out of the controller (`QtRocket::run`/`guiWorker` → [gui/GuiRunner.cpp](gui/GuiRunner.cpp)), making `QtRocket` **Qt-free** — the refactor that made the CLI possible.

---

## 2. Updated inventory (new / changed / deleted)

### New files
- **`cli/CliMain.cpp`**, **`cli/Repl.{h,cpp}`** — the headless REPL (≈590 LOC). Line-oriented; works interactively, from a pipe, or a redirected script.
- **`gui/GuiRunner.{h,cpp}`** — the Qt thread launcher extracted from `QtRocket`.
- **`sim/VacuumAtmosphere.h`** — zero-density atmosphere (drag-free baseline).
- **`sim/RK45Solver.h`** — now a complete adaptive RKF45 (was the untracked, non-compiling "where I left off" fragment).
- **`tests/PhysicsIntegrationTests.cpp`**, **`tests/MotorDatabasePersistenceTests.cpp`**, **`tests/CMakeLists.txt`**, **`sim/tests/RK45SolverTests.cpp`** — new suites (`qtrocket_integration_tests` + RK45 cases).

### Materially changed
- **`sim/DESolver.h`** — `step()` now returns `StepResult{state, rate, stepSize}`; fixed- and adaptive-step solvers are interchangeable, and the caller advances its clock by `stepSize` ([sim/DESolver.h:25-61](sim/DESolver.h#L25-L61)).
- **`model/RocketModel.{h,cpp}`** — drag, `dryMass`/`dragCoefficient`/`referenceArea`, `isMotorSet()`, new `getForces` signature.
- **`sim/Propagator.{h,cpp}`** — `StepResult`-based loop, timestep push-down, `minFlightTime`.
- **`sim/Environment.h`** — `getAvailable*` blank-entry bug fixed; Vacuum registered.
- **`utils/MotorModelDatabase.{h,cpp}`** — the unified surface (see [§3.2](#32-the-unifying-abstraction-utilsmotormodeldatabase)); `loadMotorDatabase` implemented.
- **`gui/MainWindow.{cpp,h,ui}`**, **`gui/ThrustCurveMotorSelector.cpp`** — single "motor is set" gating, DB-backed selectors, "Load Motor Database" button.
- **`utils/ThrustCurveAPI.cpp`**, **`model/MotorModel.h`** — debug spam removed; manufacturer mapping via `toEnum` for all makers.

### Deleted
- **`model/MotorModelDatabase.{h,cpp}`** (the dead duplicate class) and its CMake entry.

---

## 3. GUI ↔ CLI feature parity (the core of this review)

Two front-ends now drive one engine: the Qt **GUI** (`qtrocket`) and the headless **CLI** (`qtrocket-cli`). Both call into the same `QtRocket` singleton, `RocketModel`, `Propagator`, `Environment`, and `MotorModelDatabase`, so the *physics* is identical. The divergence is entirely in **what each front-end exposes** and **how it validates input**.

### 3.1 Capability matrix

| Capability | GUI | CLI | Notes |
|---|:--:|:--:|---|
| Load RockSim `.rse` | ✅ `Load RSE` | ✅ `loadmotors` | both via `importRSEFile` |
| Load `.qmd` database | ✅ `Load Motor Database` | ✅ `loaddb` | both via `loadMotorDatabase` |
| Save `.qmd` database | ⚠️ Tools menu, **fixed name** `qtrocket_motors.qmd`, no dialog | ✅ `savedb <path>` | GUI can't choose path ([gui/MainWindow.cpp:116-119](gui/MainWindow.cpp#L116-L119)) |
| List motors | ✅ combo box | ✅ `listmotors [substr]` | CLI also shows avg-thrust/Itot |
| Select motor | ✅ `Set Motor` | ✅ `setmotor <name>` | both gate on `isMotorSet()` |
| thrustcurve.org facets | ✅ `Get TC Motor Data` | ✅ `tcfacets` | |
| thrustcurve.org search | ✅ (in selector) | ✅ `tcsearch k=v` | |
| Set mass | ✅ `mass` (default 1.0) | ✅ `setmass` (**validates >0**) | GUI silently ignores 0/empty |
| Set drag Cd | ✅ `dragCoeff` (default 0.5) | ✅ `setdrag` | |
| **Set reference area** | ❌ **not exposed** | ✅ `setarea` | **GUI drag stuck at default 1.134e-3 m²** |
| Set initial velocity | ✅ `initialVelocity` (default 5.0) | ✅ `setvelocity` | |
| **Set launch angle** | ❌ **field disabled / read-only** (90°) | ✅ `setangle` | **GUI always launches vertical** |
| Set timestep | ⚠️ Sim Options only (MainWindow field **dead**; **no validation → hang**) | ✅ `settimestep` (**validates >0**) | see [§4 H2](#4-issues-found-regressions-traps-dead-code) |
| Select atmosphere | ✅ Sim Options combo | ✅ `setatmosphere` | both include Vacuum now |
| Select gravity | ✅ Sim Options combo | ❌ **no command** | minor (only Constant is frame-consistent) |
| Select integrator | ❌ combo present but **dead** | ❌ (always RK4) | RK45 exists but unwired |
| Launch | ✅ `Calculate Trajectory` | ✅ `launch` | |
| View results | ✅ plots: altitude, Z-velocity, thrust | ✅ summary (apogee/max-speed/downrange/landing) | different projections |
| Export CSV | ❌ | ✅ auto `qtrocket_run.csv` + `save`/`states` | |
| Inspect config | ~ (values shown in fields) | ✅ `status` (**omits timestep**) | |

### 3.2 The unifying abstraction: `utils::MotorModelDatabase`

The strongest architectural improvement of this cycle. Both front-ends now talk to **one** source-agnostic surface ([utils/MotorModelDatabase.h](utils/MotorModelDatabase.h)):

- `importRSEFile(path)` → net-new count; `loadMotorDatabase`/`saveMotorDatabase`; `size()`.
- `listMotors(MotorQuery)` / `getMotorModel(name)` — list cheaply (returns `MotorSummary`), fetch the full model on selection.
- `getOnlineSearchFacets()` / `searchOnline(MotorQuery)` — thrustcurve.org behind the DB; results are **merged into the map** so `getMotorModel` sees them.

`MotorQuery` / `MotorSummary` / `MotorSearchFacets` are clean DTOs that decouple clients from both `ThrustCurveAPI` and the XML format; `ThrustCurveAPI` is now owned privately ([utils/MotorModelDatabase.h:154-162](utils/MotorModelDatabase.h#L154-L162)). The GUI's `ThrustCurveMotorSelector` was rewritten onto this surface ([gui/ThrustCurveMotorSelector.cpp:48-95](gui/ThrustCurveMotorSelector.cpp#L48-L95)). This is exactly the right shape and is the main reason the CLI was cheap to add.

### 3.3 Parity verdict

- **CLI is now the more complete instrument**: it exposes reference area, launch angle, atmosphere, timestep, CSV export, and `status`, all validated.
- **The GUI lags in three user-visible ways** (angle disabled, no reference area, dead/unsafe timestep) and one cosmetic way (fixed save path). Because the P0 work made angle and timestep *functional*, the GUI's disabled angle and dead timestep field are now **actively misleading** — the classic "stop lying to the user" problem the P0 round was meant to end, resurfacing on the GUI side.

---

## 4. Issues found (regressions, traps, dead code)

Severity: **[H]** correctness/usability trap · **[M]** parity gap / dead UI · **[L]** latent/cosmetic. None block the build (18/18 tests pass); these are about honesty, safety, and polish.

### [H1] GUI launch-angle input is disabled — the marquee P0 fix is invisible in the GUI
[gui/MainWindow.ui:94-104](gui/MainWindow.ui#L94-L104): the `initialAngle` line edit is `enabled=false`, `readOnly=true`, fixed text `90.0`. `onButton_calculateTrajectory_clicked` reads it ([gui/MainWindow.cpp:131-132](gui/MainWindow.cpp#L131-L132)), but the user can never change it, so `cos(90°)≈0`, `sin(90°)=1` ⇒ every GUI launch is purely vertical. The fix that made angle matter (verified to yield 251 m downrange in the CLI) **cannot be exercised from the GUI at all.** Fix: enable the field (and ideally validate 0–90).

### [H2] Unvalidated timestep of 0 hangs the simulation loop (GUI path)
`SimOptionsWindow` sends `ui->timeStep->text().toDouble()` straight into `setTimeStep` with no validation ([gui/SimOptionsWindow.cpp:65](gui/SimOptionsWindow.cpp#L65)). The field has only **placeholder** text `0.01`, no actual text ([gui/SimOptionsWindow.ui:41-43](gui/SimOptionsWindow.ui#L41-L43)), so opening Simulation Options and clicking **OK without typing a value** calls `setTimeStep(0.0)`. Then in `runUntilTerminate` the RK4 step is 0, `result.stepSize == 0`, `currentTime += 0` never advances past `minFlightTime`, and the loop spins forever appending states → **infinite loop + unbounded memory** ([sim/Propagator.cpp:65-90](sim/Propagator.cpp#L65-L90)). The greyed placeholder *looks* like a value, inviting exactly this. The CLI guards against it (`settimestep` rejects ≤0, [cli/Repl.cpp:411-415](cli/Repl.cpp#L411-L415)); the GUI does not. *Latent before this cycle (the old loop also incremented by a 0 member), but the timestep-propagation fix kept it live and the CLI guard makes the asymmetry glaring.* **Best fix: validate once in `Propagator::setTimeStep`/`QtRocket::setTimeStep` so both front-ends are safe** ([sim/Propagator.h:54-65](sim/Propagator.h#L54-L65)).

### [M1] GUI cannot set reference area (drag only half-tunable)
The drag model needs `Cd` **and** area `A`, but the GUI exposes only `Cd`; `referenceArea` is stuck at the default `1.134e-3 m²` ([model/RocketModel.h:145](model/RocketModel.h#L145)). The CLI has `setarea`. Add a reference-area field beside `Cd`.

### [M2] MainWindow "Time Step" field is dead
`gui/MainWindow.ui` has a `timeStep` line edit (default `0.01`, [gui/MainWindow.ui:156-160](gui/MainWindow.ui#L156-L160)) that **nothing reads** — `onButton_calculateTrajectory_clicked` ignores it, and only `SimOptionsWindow` has a wired timestep. Two timestep inputs in the GUI, one inert. Remove it or wire it (and retire the Sim Options one to avoid two sources of truth).

### [M3] "Integrator" dropdown is dead; RK45 is implemented but unwired
`SimOptionsWindow.ui` has an `integratorCombo` ([gui/SimOptionsWindow.ui:75](gui/SimOptionsWindow.ui#L75)) that is **never populated or read**, and `Propagator` hard-codes `RK4Solver` ([sim/Propagator.h:70](sim/Propagator.h#L70)). So the finished, tested `RK45Solver` is not selectable anywhere. This is the UI half of P4's "make RK45 selectable, RK4 default" — wire the combo to a solver factory behind `DESolver`.

### [M4] CLI has no gravity-model selector
The GUI's Sim Options can pick gravity; the CLI cannot. Minor — only "Constant Gravity" is consistent with the launch frame today — but it's an asymmetry. Add `setgravity` (and gate "Spherical Gravity" until the frame work in P4).

### [M5] Logging spam + GUI/CLI log-level asymmetry
`MotorModel` logs thrust and mass at **INFO on essentially every evaluation** ([model/MotorModel.cpp:63](model/MotorModel.cpp#L63), [model/MotorModel.cpp:86](model/MotorModel.cpp#L86)) — and `getForces`/`getMass`/`getThrust` are called ~4×/step. The CLI mutes this to `ERROR_` ([cli/CliMain.cpp:19](cli/CliMain.cpp#L19)), but the GUI still runs at the most verbose `PERF_` ([main.cpp:16](main.cpp#L16)), so a multi-thousand-step GUI sim floods stdout + `log.txt`. This is P5's "audit logging," still open, and now a concrete GUI/CLI inconsistency. Demote per-step logs to a `PERF_`-only or trace level and lower the GUI default.

### [M6] GUI motor-DB save is a fixed filename with no dialog
[gui/MainWindow.cpp:116-119](gui/MainWindow.cpp#L116-L119) always writes `qtrocket_motors.qmd` in the cwd. There is a file dialog for *loading* a DB but not for *saving* one — inconsistent with both the CLI (`savedb <path>`) and the load path. Add a save dialog.

### [M7] GUI input is unvalidated
`mass`/`dragCoeff` are read with `toDouble()` and passed straight in ([gui/MainWindow.cpp:122-148](gui/MainWindow.cpp#L122-L148)). An empty/zero mass becomes `setMass(0)`, which `RocketModel` silently ignores (keeps the prior/default 1 kg, [model/RocketModel.h:124](model/RocketModel.h#L124)) — the GUI shows a value the sim isn't using. The CLI validates. Add field validators / feedback.

### [L1] Stale comment: RSE loader "side effect" no longer exists
`MotorModelDatabase::importRSEFile` claims `RSEDatabaseLoader` "also still pushes into the QtRocket-global database as a constructor side effect" ([utils/MotorModelDatabase.cpp:57-62](utils/MotorModelDatabase.cpp#L57-L62)). It doesn't — the loader only appends to its own vector ([utils/RSEDatabaseLoader.cpp:49-102](utils/RSEDatabaseLoader.cpp#L49-L102)). The comment describes a hazard that's already gone; delete it so no one "fixes" a non-issue.

### [L2] RK4 NaN guard is still a no-op
[sim/RK4Solver.h:52](sim/RK4Solver.h#L52): `if(dt == std::numeric_limits<double>::quiet_NaN())` can never be true (NaN compares unequal to everything), so a missing `setTimeStep` would silently integrate with `dt = NaN`. Use `std::isnan(dt)`. (Harmless today; `setTimeStep` is always called.)

### [L3] Sample time-labeling off-by-one persists; t=0 state never recorded
`runUntilTerminate` appends the **post-step** state labeled with the **pre-increment** time, and never records the true initial state ([sim/Propagator.cpp:65-89](sim/Propagator.cpp#L65-L89)). So each recorded sample's value is one `dt` ahead of its label, and the first row is the post-first-step state at `t=0`. Reported apogee/landing times are ~one `dt` early. Carried over from the prior scan; cheap to fix while building the event system (P4).

### [L4] Dead methods / members
- `ThrustCurveAPI::getMotorData` — never called (a near-duplicate of `getThrustCurve`, which `searchMotors` uses) ([utils/ThrustCurveAPI.cpp:82](utils/ThrustCurveAPI.cpp#L82)).
- `Propagatable::nextState` member — unused; the Propagator uses a local ([model/Propagatable.h:51](model/Propagatable.h#L51)).
- `Propagator::retainStates` and `setSaveStats` — neither is called, and both set the same `saveStates` flag ([sim/Propagator.h:48-66](sim/Propagator.h#L48-L66)).

### [L5] `DEG_PER_RAD = 57.2958` duplicated and low-precision
Hard-coded in [gui/MainWindow.cpp:137](gui/MainWindow.cpp#L137), [cli/Repl.cpp:30](cli/Repl.cpp#L30), and [tests/PhysicsIntegrationTests.cpp:39](tests/PhysicsIntegrationTests.cpp#L39). Centralize in `utils/math/Constants.h` at full precision.

### [L6] `getMetadata` likely mis-keys motor "type"
[utils/ThrustCurveAPI.cpp:192](utils/ThrustCurveAPI.cpp#L192) reads `(*iter)["types"]` inside the per-element `types` loop where it almost certainly means `["type"]`. The metadata `types` list isn't consumed by the facets path, so impact is nil today — but it's a latent parsing bug. Pre-existing.

---

## 5. Physics & math model update

| Aspect | Prior scan (2026-06-05) | **Now (2026-06-07)** |
|---|---|---|
| Initial conditions | discarded (always from rest at origin) | **honored** — `launch()` seeds `currentState` from `initialState` |
| Forces | thrust(+Z) + gravity only (vacuum) | thrust(+Z) + gravity + **aerodynamic drag** `−½ρ\|v\|v·Cd·A` |
| Atmosphere in the loop | never consulted | **consulted every force eval** (ρ from active model; altitude clamped ≥0) |
| RK4 force evaluation | at fixed `currentState` (would degrade to Euler with drag) | at **each trial (position, velocity)** — full RK4 for drag/gravity |
| Timestep → integrator | not propagated (stuck 0.01) | **propagated** + re-asserted each run |
| Integrator | fixed RK4; RK45 broken/untracked | RK4 (default) **+ working adaptive RKF45** (tested, *not yet wired*) |
| Mass | `motorMass + 1 kg sphere` (GUI ignored) | `motorMass + dryMass` (GUI/CLI-set; composite-part term deferred to P2) |
| Termination | `z < 0` | `z < 0` **and** `t > 4 s` (`minFlightTime` guard) |

**Correctness checks (this review):**
- **Drag sign/magnitude** is right: the terminal-velocity force balance is exact to 1e-6 in [tests/PhysicsIntegrationTests.cpp:152-181](tests/PhysicsIntegrationTests.cpp#L152-L181), and the CLI shows max speed shifting from impact (vacuum) to just after burnout (atmosphere) — the expected qualitative signature.
- **RKF45 coefficients** in [sim/RK45Solver.h:154-165](sim/RK45Solver.h#L154-L165) match the standard Fehlberg tableau; the solver reproduces constant-acceleration exactly and tracks SHM/exponential decay within tolerance, and the step size demonstrably adapts ([sim/tests/RK45SolverTests.cpp](sim/tests/RK45SolverTests.cpp)).

**Residual fidelity gaps (acknowledged, on the roadmap):**
- **Thrust direction is still world +Z** ([model/RocketModel.cpp:46](model/RocketModel.cpp#L46)) — no gravity-turn (P1, still open).
- **The ODE callback carries no stage time** ([sim/DESolver.h:52-58](sim/DESolver.h#L52-L58)), so thrust `f(t)` is evaluated at the step-start time for all four RK4 stages. Spatial forces (drag, gravity) are now full-order; the *thrust term's* time dependence remains first-order. Small at `dt = 0.01 s`, but real — thread time through the ODE interface when convenient.
- **`minFlightTime = 4 s`** is a heuristic ([sim/Propagator.h:28](sim/Propagator.h#L28)): a flight that truly ends before 4 s keeps integrating *underground* (altitude clamped to 0 for density) until the guard releases, so very short flights misreport landing. The real fix is the P4 event system (apogee/burnout/impact detection).

---

## 6. Architecture observations

- **Controller ↔ library dependency cycle.** `model` (`RocketModel.cpp`) and `utils` (`MotorModelDatabase.cpp`) include `QtRocket.h` and call `QtRocket::getInstance()`, while `QtRocket` depends on `model`/`sim`/`utils`. The build breaks the cycle by **compiling `QtRocket.cpp` directly into each executable** (`qtrocket`, `qtrocket-cli`, `integration_tests`) instead of a shared lib — documented in [CMakeLists.txt:166-179](CMakeLists.txt#L166-L179) and [tests/CMakeLists.txt:1-9](tests/CMakeLists.txt#L1-L9). `RocketModel::getForces` reaching into the singleton for the `Environment` ([model/RocketModel.cpp:51-62](model/RocketModel.cpp#L51-L62)) is the tightest knot: it couples the physics to the global controller and forces every physics test to spin up the singleton. **Dependency-injecting `Environment` into `RocketModel`/`Propagator`** would cut the cycle and let the model be unit-tested in isolation.
- **Positive: `QtRocket` is now Qt-free.** Extracting the GUI thread to `gui/GuiRunner` is what made the headless CLI (and the headless integration tests) possible — a clean, high-leverage refactor worth preserving.
- **`Environment` mutation vs. replacement.** The CLI *mutates* the existing `Environment` (`setAtmosphereModel`), while `SimOptionsWindow` *replaces* it with a freshly constructed one on every accept ([gui/SimOptionsWindow.cpp:59-71](gui/SimOptionsWindow.cpp#L59-L71)). This works only because `getForces` re-fetches `QtRocket::getInstance()->getEnvironment()` every call, so there's no stale-pointer bug — but the two front-ends manage environment lifetime differently, and the GUI's rebuild silently resets any non-default selection each time the dialog is accepted.

---

## 7. Updated State Assessment

### ✅ Works today (tested or verified by running)
- Everything from the prior scan, **plus**: aerodynamic drag; atmosphere-coupled forces; honored initial conditions; timestep propagation; reference area / mass / Cd as real inputs; `VacuumAtmosphere`; motor-DB **load** (round-trip tested); the unified `MotorModelDatabase`; the **headless CLI**; the adaptive **RKF45** solver (standalone); and real physics + persistence integration tests. Build is clean; **18/18 CTest cases pass**.

### 🟡 Half-finished / wired-but-wrong (highest-value fixes)
- **GUI**: launch angle disabled ([H1]); no reference-area input ([M1]); dead MainWindow timestep field ([M2]); dead integrator combo ([M3]); unvalidated timestep-0 hang ([H2]); unvalidated mass ([M7]); fixed save path ([M6]); `PERF_` logging flood ([M5]).
- **Engine**: thrust still world +Z; RKF45 implemented but **not wired** into `Propagator`; off-by-one sample labeling ([L3]); `minFlightTime` heuristic.
- **CLI**: no gravity selector ([M4]); `status` omits the timestep.

### 🟥 Stubs / ⬛ dead (largely unchanged)
- `sim/Aero.{h,cpp}` (empty), `sim/WindModel` (returns 0), `gui/RocketTreeView` (no model), `model/Propagatable.cpp` (empty), orientation fields never integrated, `GeoidModel`/`SphericalGeoidModel` (unused), `ThreadPool`/`TSQueue` (unused), Spherical gravity (frame-mismatched/unused).
- New small dead ends: `ThrustCurveAPI::getMotorData`, `Propagatable::nextState`, `Propagator::retainStates`/`setSaveStats` ([L4]).

### ⬛ Missing entirely (unchanged big rocks)
Component **geometry/types** (NoseCone/BodyTube/Fin), **Barrowman CP & stability**, **6-DOF rotation**, **recovery/parachute**, staging, launch rail, wind effects, a **design file format**, component-tree UI, optimization/Monte-Carlo, materials/presets, `.eng` (RASP) import.

---

## 8. Comparison to OpenRocket (delta)

Two rows move and one is new:

| Subsystem | Prior | **Now** | Why |
|---|---|---|---|
| **Simulation engine** | 🟡 ~25% | 🟡 **~30%** | Drag + atmosphere now *consumed*; adaptive RKF45 implemented (though unwired). Still no 6-DOF / recovery / events / wind / staging. |
| **Atmosphere / environment** | 🟡 ~50% | 🟡 **~55%** | The validated US-Standard model is finally *used* by the force model; `Vacuum` baseline added. Frame mismatch for Spherical persists. |
| **Motor / propulsion** | 🟡 ~60% | 🟡 **~62%** | DB **load** now implemented (save+load round-trips losslessly). Still no `.eng`, no clusters, no ejection modeling. |
| **Scripting / headless / batch** *(new row)* | ⬛ 0% | 🟡 **~15%** | `qtrocket-cli` gives a scriptable, pipe-friendly REPL with CSV output — usable for automation, regression scripting, and agent-driven development. OpenRocket has a limited CLI + full Java API; QtRocket has the *shape* of this now, but no design files to script over yet. |

Everything else (geometry/component tree, aerodynamics/CP, UI editor, `.ork` design files, presets) is **unchanged** from the prior scan's assessment. The three biggest gaps are still, in order: **(1) component geometry, (2) aerodynamics/CP feeding the forces, (3) 6-DOF + recovery.**

A genuinely new strength worth recording: **the project is now testable and scriptable end-to-end without a display** — a force-multiplier for everything that follows.

---

## 9. Recommended next steps (refreshed)

The original P0 is done. Before pushing into P2 geometry, **close the GUI/CLI parity gaps and the two traps** — they're cheap and they restore the "don't lie to the user" guarantee on the GUI side.

### P0.5 — GUI parity & safety (hours)
1. **[H2] Validate the timestep in the setter** (`Propagator::setTimeStep`/`QtRocket::setTimeStep`, reject ≤0) so neither front-end can hang. Give the Sim Options field a real default, not just a placeholder.
2. **[H1] Enable the launch-angle field** (and validate 0–90) so the GUI can use the initial-state fix.
3. **[M1] Add a reference-area input** to MainWindow next to `Cd`.
4. **[M2] Remove the dead MainWindow timestep field** (single source of truth in Sim Options) and **[M5] lower the GUI log level / demote per-step logs**.
5. **[M3]/[M4]** Either wire the integrator combo to a `DESolver` factory (RK4 default, RK45 optional) or hide it; add a CLI `setgravity` for symmetry.
6. **[L1]** Delete the stale RSE side-effect comment; **[L2]** fix the NaN guard.

### P1 (resume) — finish "physically believable single-stage"
7. **Apply thrust along the velocity/body axis** after rail exit (fall back to +Z near zero speed).
8. **Recovery / parachute phase** — switch to a parachute Cd·A at apogee, terminate at impact, plot the full profile. *(This is the moment the numbers look like a real flight.)*

### P2+ — unchanged from [TODO.md](TODO.md)
Concrete component types + reference area from geometry; wire `RocketTreeView`; restore `topPart.getCompositeMass()`; Barrowman CP/stability; 6-DOF + event system + frame reconciliation + wind; **wire** the adaptive RK45; design save/load + `.eng` import.

---

## 10. Verification appendix

**Build:** `cmake --build build` → clean, exit 0 (all targets, incl. `qtrocket`, `qtrocket-cli`, `integration_tests`).

**Tests:** `ctest` → **18/18 passed** in ~2.1 s, including:
- `PhysicsIntegrationTest.*` — timestep-reaches-integrator (vacuum), drag-reduces-apogee, terminal-velocity force balance, US-Standard no-crash.
- `RK45SolverTest.*` — exact constant-accel, SHM, exponential decay, step-size adaptation.
- `MotorDatabaseRoundTrip.SaveThenLoadReproducesTheDatabase` + enum/manufacturer mapping (Klima/Quest pinned).

**Live CLI run** (`qtrocket-cli`, motor `G80T`, m=0.5 kg, Cd=0.75, A=1.134e-3 m²):

| Scenario | Apogee | Max speed | Downrange | Steps |
|---|---|---|---|---|
| Vacuum, vertical | 1940.4 m | 195.1 m/s (at impact) | 0 m | 4046 |
| Constant Atmosphere, vertical | 846.7 m | 170.6 m/s (post-burnout) | 0 m | 2711 |
| Constant Atmosphere, 45° @ 30 m/s | 941.2 m | 187.4 m/s | **251.5 m** | 2851 |

These confirm, end-to-end: **drag costs ~56% of apogee**, **max-speed timing shifts** from impact (vacuum) to burnout (atmosphere), and **launch angle/initial velocity now move the rocket downrange** — the central behaviors the P0/P1 work set out to fix. The bundled `data/Aerotech.rse` reports **249** net-new motors (252 `<engine>` entries, 3 duplicate common names collapse in the name-keyed map).

---

*Delta analysis of the `development` branch working tree, 2026-06-07. Every cited claim was read in source and, where behavioral, confirmed by a clean build, the full CTest suite, and a live `qtrocket-cli` run. Baseline: [ANALYSIS_RESULTS.md](docs/ANALYSIS_RESULTS.md); roadmap: [TODO.md](TODO.md).*
