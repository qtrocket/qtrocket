# QtRocket — TODO

Prioritized development roadmap, derived from [§8 "Where to Resume"](ANALYSIS_RESULTS.md#8-where-to-resume-prioritized) of [ANALYSIS_RESULTS.md](ANALYSIS_RESULTS.md). See that document for full context, evidence, and the data-flow trace behind each item.

**🎯 Next milestone: "Make a single-stage flight physically believable."** A launched rocket should respect its GUI inputs, coast up against gravity *and drag*, reach a realistic apogee, and (stretch) descend under a parachute. P0 + P1 get you there.

Tasks tagged **[verified bug]** were confirmed against the source during analysis.

---

## P0 — Unblock, de-risk, stop lying to the user (hours → ~1 day)

- [✅] **Build & run it first.** `cmake -B build -S . && cmake --build build && ./build/qtrocket`; load `data/Aerotech.rse`, set a motor, Calculate Trajectory, view the altitude plot; `ctest` to confirm the atmosphere tests pass. (Newer pinned dependency versions are the likeliest first-build friction.)
- [✅] **[verified bug] Seed `currentState` from `initialState`** at launch (`QtRocket::launchRocket` / `Propagator::runUntilTerminate`) so launch angle & initial velocity actually affect the trajectory. *Highest-value single fix — currently the rocket launches from rest, straight up.*
- [✅] **[verified bug] Make `RocketModel::setMass`/`setDragCoefficient` real** ([RocketModel.h:101-103](model/RocketModel.h#L101-L103)) — add backing members and use them — or remove the GUI fields. Pick one and be consistent.
- [✅] **[verified bug] Fix `Environment::getAvailable*Models()`** ([Environment.h](sim/Environment.h)) — now `reserve` + `back_inserter` instead of size-construct, so the lists return exactly the model names with no leading blanks. Surfaced via the CLI `listatmospheres` command.
- [✅] **[verified bug] Enable "Calculate Trajectory" from the thrustcurve.org path too** — unified a single "a motor is set" signal: `RocketModel::isMotorSet()` (set by `setMotorModel`, the funnel for every selection path) gates the button via `MainWindow::refreshCalculateTrajectoryEnabled()`, called from the RSE "Set Motor" handler and after the thrustcurve.org selector closes. Verified in the GUI — setting a motor from thrustcurve.org now enables the button.
- [✅] **[verified bug] Fix timestep propagation** — `Propagator::setTimeStep` ([Propagator.h:54](sim/Propagator.h#L54)) now pushes `dt` into the `RK4Solver` (and `runUntilTerminate` re-asserts it before each run), so changing the timestep changes the integration step. Verified via `qtrocket-cli`: across dt = 0.001…0.1 the step count scales ~1/dt and the CSV sample interval equals dt, while flight time stays ~constant and apogee converges as dt→0.
- [ ] **Decide the fate of `sim/RK45Solver.h`** — finish it (see P4) or delete the untracked, non-compiling file so it can't break a future build.
- [ ] **Remove dead code** — ✅ deleted `model/MotorModelDatabase.{h,cpp}` + its CMake entry (the duplicate-class hazard, done while unifying motor sourcing behind `utils::MotorModelDatabase`); `QtRocket::runSim()` was already gone. **Remaining:** the stray `QtRocket::states` and the unused `launchSitePosition`.
- [✅] **Audit `utils/ThrustCurveAPI.cpp`** — removed the `debug("1".."6")` traces; fixed the Klima↔Quest swap in `MotorModel::MotorManufacturer::toEnum`; `searchMotors` now maps every modeled manufacturer through `toEnum` (taught to accept thrustcurve.org's full-name forms, e.g. "Estes Industries", "Quest Aerospace") instead of only "AeroTech". Verified live against thrustcurve.org (Estes/Quest/Apogee) plus a deterministic mapping test.

## P1 — Make the physics honest: drag + the atmosphere (keystone, days)

- [✅] **Implement aerodynamic drag** in `RocketModel::getForces`: `F_drag = -½·ρ(altitude)·|v|·v·Cd·A`, with `ρ` from the active atmospheric model. Added a `referenceArea` member (Cd already existed), a `VacuumAtmosphere` model, and `setatmosphere`/`setarea` CLI commands. Altitude is clamped to ≥0 at the density query (USStandardAtmosphere's Bin lookup throws below 0). Terminal velocity verified manually via `qtrocket-cli` and now covered by an automated test (see below).
- [✅] **⚠ Fix RK4 force evaluation while adding drag** — `getForces` now takes `(t, position, velocity)` and the Propagator's ODE lambda passes each RK4 trial state ([Propagator.cpp](sim/Propagator.cpp)), so velocity-dependent drag integrates at full RK4 accuracy instead of degrading to Euler.
- [✅] **Add automated CTest coverage for the new physics** — [tests/PhysicsIntegrationTests.cpp](tests/PhysicsIntegrationTests.cpp) (suite `qtrocket_integration_tests`) drives the real `RocketModel`/`Propagator`/`Environment` end-to-end: timestep invariants under `Vacuum` (step count ∝ 1/dt, flight time ~constant, sample interval == dt), drag-reduces-apogee vs vacuum, terminal-velocity force balance under `Constant Atmosphere`, and a US-Standard no-crash regression guard. No mock Propagatable — atmosphere selection makes the runs drag-free or not as needed.
- [ ] **Apply thrust along the velocity/body direction** after rail exit (fall back to +Z at near-zero speed) instead of fixed world +Z.
- [ ] **Add a recovery / parachute descent phase** — after burnout/apogee, switch to a parachute drag model (constant Cd·A), trigger on vertical-velocity sign change, terminate at ground impact. Plot the full up-and-down profile.

## P2 — A real (if minimal) component model + reference area (1–2 weeks)

- [ ] **Introduce concrete component types** on `model::Part` (NoseCone, BodyTube, FinSet) carrying dimensions; derive mass, CG, and reference/frontal area from geometry (reuse `InertiaTensors` + parallel-axis composition). Replace the hard-coded 1 kg sphere with an assembled design.
- [ ] **Wire `gui/RocketTreeView`** to a `QAbstractItemModel` backed by the `Part` tree — editable exploded view with add/remove/edit; recompute mass/inertia on change.
- [ ] **Restore `topPart.getCompositeMass()` in `RocketModel::getMass()`** ([RocketModel.cpp](model/RocketModel.cpp)) — the P0 mass fix temporarily overrides mass with the GUI-provided `dryMass` because `topPart` is a placeholder 1 kg sphere. `getMass()` = `mm.getMass(t) + topPart.getCompositeMass(t)` is the correct formulation; once concrete Part types carry real masses, drop the `dryMass` override (and reconcile/repurpose the GUI mass field) and add the composite-mass term back.

## P3 — Stability & center of pressure (Barrowman) (1–2 weeks)

- [ ] **Aerodynamic build-up:** compute Barrowman **CP** and a Cd estimate from component geometry in the (empty) `Aero` class, replacing the hand-entered Cd. Surface **CG / CP / static margin** in the UI — the first feature that makes QtRocket a *design* tool.

## P4 — 6-DOF, events & integrator fidelity (later)

- [ ] **Re-enable rotational dynamics** — instantiate the commented-out `RK4Solver<Quaternion>` orientation integrator; implement `getTorques` from aero moments (CP–CG) + thrust offset; integrate orientation; update `StateData` orientation/Euler. Keep 3-DOF behind a flag.
- [ ] **Replace the `z<0` terminate with an event system** — apogee, burnout, recovery deployment, ground impact; record event times and mark them on the plots.
- [ ] **Reconcile coordinate frames** — flat-ground (ENU/local) vs geocentric (ECEF). Add a geodetic launch-site → ECEF transform for the Spherical models, or gate them out of the GUI until supported. Document the chosen frame.
- [ ] **Wind** — flesh out `WindModel` (constant + altitude profile); feed relative airspeed `(v − v_wind)` into drag.
- [ ] **Finish adaptive RK45** behind the `DESolver` interface (step-size control + error-tolerance test); make it selectable, RK4 default.

## P5 — Persistence & polish

- [ ] **Design save/load** — an `.ork`-style project file serializing the component tree + sim options (Boost.PropertyTree already a dependency); File → Save/Open in `MainWindow`.
- [ ] ✅ **Implemented `MotorModelDatabase::loadMotorDatabase`** (mirrors the save; round-trip test in [tests/MotorDatabasePersistenceTests.cpp](tests/MotorDatabasePersistenceTests.cpp)), wired into the CLI (`loaddb`/`savedb`) and the GUI ("Load Motor Database" button). **Remaining:** a **RASP `.eng`** parser alongside `RSEDatabaseLoader`.
- [ ] **Audit logging & threading** — demote per-step logs / add a log-level flag (drop default `PERF_`); confirm the GUI-on-worker-thread model is Qt-safe; consider running long sims on the existing `ThreadPool`.
