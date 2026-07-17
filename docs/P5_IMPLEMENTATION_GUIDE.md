# P5 Component Cd Build-Up — Self-Implementation Guide

A step-by-step walkthrough for implementing the drag build-up **yourself**, phase by phase. The
companion spec, `docs/P5_COMPONENT_CD_BUILDUP_SPEC.md`, is the plan of record — on any conflict the
spec wins, and this guide cites it by section (§) throughout. The spec says *what* and *why*; this
guide adds *in what order, with what checks, and what to watch out for*.

**How to use it**

- Work one phase at a time, in order. Every phase ends with the same gate: `-Werror` build + full
  ctest green. Don't start phase N+1 on a red tree.
- One commit per phase (the spec designed them commit-sized, §8). You're on branch `dynamic-Cd` —
  stay there.
- Each phase has **self-check questions**. They aren't busywork: if you can't answer one, you've
  typed the code without absorbing the design, and the later phases will feel arbitrary. The answers
  are all in the spec (section cited).
- Write comments per `CLAUDE.md`: terse, lowercase, the *why* not the *what*. The spec's rationale
  lines are raw material for the one-clause comments the tree expects — don't paste spec paragraphs
  into headers.
- Rough sizing, so you can plan sessions: P1 ≈ 1–2 h · P2 ≈ half a day · P3 ≈ a full day (it's the
  physics + its oracle tests) · P4 ≈ half a day · P5 ≈ half a day · P6 ≈ 1–2 h · P7 ≈ 1–2 h.

---

## Part 0 — Baseline (do this before touching anything)

1. **Configure and build** (presets only — never ad-hoc `cmake -B build`):

   ```bash
   cmake --preset debug-clang
   cmake --build --preset debug-clang
   ```

2. **Run the full suite and confirm green**, including the heavy ladder:

   ```bash
   ctest --test-dir build -R 'qtrocket_*'            # everything (what CI runs)
   ctest --test-dir build -R 'qtrocket_*' -LE heavy  # the fast loop you'll live in
   ```

3. **Capture a trajectory baseline.** Phases 2 and 3 claim "trajectories bit-identical"; make that
   checkable instead of taking it on faith. Fly a couple of fixtures now and save the output:

   ```bash
   mkdir -p /tmp/p5-baseline
   ./build/cli/qtrocket-cli -c "loadmotors data/Aerotech.rse
   loaddesign tests/data/designs/mid29_basic.qrd
   setmotor G80T
   setatmosphere US Standard 1976
   launch" > /tmp/p5-baseline/mid29-usstd.txt
   ```

   (`-c` feeds one string to the line-oriented REPL, so the embedded real newlines are the command
   separator. Fixtures carry no motor — `setmotor` after `loaddesign`, exactly as the matrix tests
   do. Capture two variants: the atmospheric one above, and a vacuum one (`setatmosphere Vacuum`,
   `setdrag 0`). **What each is for:** the *vacuum* capture must stay byte-identical through the end
   of phase 3 — phases 1–3 never touch the vacuum path. The *atmospheric* capture is expected to
   change in phase 1 (that's the area rewire landing) — you'll re-capture it at the phase-1 gate,
   and *that* version must hold byte-identical through phases 2–3. From phase 4 on the atmospheric
   flights change again — the behavioral flip.)

4. **Skim the two documents** end to end once: the spec (at least §1, §3, §5, §8), and the
   PartsModel whitepaper's caching + flight-walkthrough sections (`docs/PartsModelDesignLatex/`,
   sections 05 and 10). You don't need to absorb every formula yet — phase 3 is where §2 gets real.

### Architecture orientation — the 30-minute code tour

Read these in order. Each is short; the point is to see the seam you're about to extend.

| Read | Notice |
|---|---|
| `core/model/Aero.h` | The whole aero seam is two value types: `AeroComponent` (one part's contribution) and `AeroProfile` (the folded whole). Composition is literally `operator+=` (Aero.h:50-57). Your feature is mostly "add fields to these and give them a consumer." |
| `core/model/parts/Part.h` | `Part` is a pure leaf: geometry accessors (`getLength`, `radiusOuterAt` :84, `axialLength` :92, `getReferenceArea` :109) and `getAero(refArea)` (:75) — a pure function of geometry. No tree code. Leaf setters are private, friended to `PartsModel` (:133-137): mutation is routed, by construction. |
| `core/model/PartsModel.h` | `PartNode` owns a Part + children + **two caches with two gates**: `massDirty_` (composite mass/CM/inertia, rebuilt when mass moves) and `placementDirty_` (:119) guarding `resolved_`/`verdict_` (:120-121), dirtied only by structural verbs. Internalize this pair — your drag geometry cache rides the second gate. |
| `core/model/PartsModel.cpp:224-251` | `compositeAero` (the fold you'll harden, :238) and `maxFrontalReferenceArea` (the A_ref rule: max single disc, never a sum). |
| `core/model/RocketModel.cpp:64-88` | `getForces`: thrust + gravity + the one-line drag law at :84 you're replacing. Note the altitude clamp at :81 — every atmosphere query you add must sit *after* it. |
| `core/sim/Propagator.cpp:34-41` | The derivative closure: `getForces` runs per ODE stage (RK4 4×/step, RKF45 6×/attempt). This is why the geometry must be cached and the per-stage work small (§5.2). |
| `cli/Repl.cpp:565-595` | The `setdrag`/`setarea` handlers you'll extend, and the staging-member pattern (`dragCoeff = d;`) you'll delete in phase 5. |

Data flow you're building, one line per hop:

```
Propagator stage ──▶ RocketModel::getForces(t, pos, vel, env)
                       │  rho, a, mu  ◀── env.getAtmosphericModel()   (clamped altitude)
                       │  area        ◀── effectiveReferenceArea()            [Phase 1]
                       │  geometry    ◀── parts_.root()->compositeAero(area)  [Phase 2, cached]
                       │  cd          ◀── model::totalDragCoefficient(geometry, flow)  [Phase 3]
                       ▼
                     forces += -0.5·rho·|v|·cd·area·v                          [Phase 4]
```

**Self-check before starting:** (a) Why does `getForces` receive `position` from the integrator
instead of reading `currentState`? (b) What dirties `placementDirty_`, and what conspicuously does
not? (c) Why is A_ref a max and not a sum? (Spec §2.0; whitepaper §5; RocketModel.cpp:69-71.)

---

## Phase 1 — Wire the reference area

**Goal:** `getForces` stops using the raw stored `referenceArea` member and starts using
`effectiveReferenceArea()`: manual override → geometry-derived disc → stored fallback. Small,
self-contained, and it fixes the oldest known defect (drag not scaling with airframe class) in
isolation, so any apogee shift in the sweeps is unambiguously *this* change (§8 Phase 1).

**Files:** `core/model/RocketModel.h/.cpp` only.

1. Add to `RocketModel` (spec §3.5):
   - `void clearReferenceAreaOverride() { referenceAreaOverridden = false; }`
   - `double effectiveReferenceArea() const;` — implement exactly the three-way rule: if
     `referenceAreaOverridden` return `referenceArea`; else take
     `deriveReferenceAreaFromGeometry()` (already exists, RocketModel.cpp:44-47) and return it if
     `> 0`, else `referenceArea`. The override check must short-circuit before any tree walk.
2. In `getForces`, replace `referenceArea` in the drag line (RocketModel.cpp:84) with
   `effectiveReferenceArea()`.
3. **Tests** (`core/model/tests/RocketModelTests.cpp` — follow the house style there; note the
   existing `InstallDesignReplacesTreeAndResetsReferenceAreaOverride` at :27-37):
   - geometry area used when not overridden (install a BodyTube design, check the drag force scales
     with π·r_o² — or assert via `effectiveReferenceArea()` directly if you make it public, which
     you should);
   - derived-0 fallback to the stored member — cover both ways it happens: no design at all
     (`deriveReferenceAreaFromGeometry` already guards `hasDesign()`, RocketModel.cpp:46) and a
     HollowSphere design (a root, but no frontal disc);
   - manual override wins;
   - `clearReferenceAreaOverride()` restores geometry mode.

**Gate:**

```bash
cmake --build --preset debug-clang && ctest --test-dir build -R 'qtrocket_*'
```

Run the heavy ladder too (`-L heavy`) — this phase is *expected* to shift atmospheric-sweep apogees
(the area now matches the airframe), and the spec's audit says the existing assertions are ordinal
and survive (§8 Phase 1 impact audit). If something fails, read the failure before "fixing" the
test: it may be telling you your fallback order is wrong.

Then **re-capture the atmospheric baseline** (same Part-0 command, overwrite the file): the
post-phase-1 output is the reference that phases 2 and 3 must reproduce byte-for-byte. Confirm the
*vacuum* capture still diffs clean against Part 0 — the vacuum path must not have moved.

**Self-check:** Why must fineness ratio (coming in phase 3) derive from geometry and never from
`effectiveReferenceArea()`? (§2.0 — a manual area would silently corrupt f_B.)

---

## Phase 2 — Extend the seam, geometry only

**Goal:** every part reports its drag-relevant geometry through `AeroComponent`, the fold sums it,
and `PartNode` resolves + caches the whole-airframe quantities (length, base area, max radius). **No
consumer yet** — trajectories must stay byte-identical to your Part-0 baseline. This phase is where
you learn the composition seam and the caching gates; take it slow.

**Files:** `core/model/Aero.h`, `core/model/PartsModel.{h,cpp}`, `core/model/parts/BodyTube.cpp`,
`ConicalNoseCone.{h,cpp}`, `FinSet.cpp`.

### 2a — The value types (spec §3.1)

Append to `AeroComponent` (after `cd` — **never insert before it**, all construction is positional)
the seven additive fields, all m²: `wettedBody`, `finWetted`, `finTcWetted`, `finFrontal`,
`finFrontalLE`, `noseSin2A`, `noseSinA`. Copy the exact doc comments from spec §3.1 — they carry the
units and intent. `AeroProfile` gains **the same seven fields as accumulators** (mirror the names —
`operator+=` sums component field into profile field; `hasDragGeometry` and the calculator read them
off the profile) **plus** the resolved trio `baseArea`, `length`, `maxRadius`, which the fold never
touches — they're set by the extent resolution in 2d.

### 2b — Fold hardening (spec §3.3)

`PartNode::compositeAero` (PartsModel.cpp:238) currently rebuilds each component positionally:

```cpp
profile += model::AeroComponent{c.cnAlpha, c.cnAlphaXcp + c.cnAlpha * cmStationZ, c.cd};
```

Any field you appended in 2a would be silently zeroed here. Replace with copy-then-modify:

```cpp
model::AeroComponent adj = pl.part->getAero(refArea);
adj.cnAlphaXcp += adj.cnAlpha * cmStationZ;   // re-express x_cp onto the sub-tree-root datum
profile += adj;
```

This is a two-line change with a design lesson in it: the fold must be *oblivious* to future field
additions, or every new field needs a matching edit here — the exact class of bug the refactor
exists to prevent.

### 2c — Per-part geometry (spec §3.4)

- **BodyTube** (`getAero`, BodyTube.cpp:38-44): `wettedBody = getWettedArea()` (the accessor
  exists, BodyTube.h:51). Delete the stale "eventual contribution" comment — it just came true.
- **ConicalNoseCone**: add public `double getWettedArea() const` = π·R·√(R²+L²) (the slant
  expression already lives in `computeVolume`, ConicalNoseCone.cpp:35 — promote it). In `getAero`
  (cpp:57-68) fill `wettedBody`, `noseSin2A = sin²φ·π·R²`, `noseSinA = sinφ·π·R²` with
  sinφ = R/√(R²+L²). Leave every CNalpha/CP line alone.
- **FinSet** (`getAero`, FinSet.cpp:60-90): fill `finWetted = getWettedArea()`,
  `finTcWetted = (thickness/c̄)·finWetted` with the trapezoid MAC
  c̄ = (2/3)·(c_r + c_t − c_r·c_t/(c_r+c_t)) (§2.1 — the ctor guarantees c_r+c_t > 0),
  `finFrontal = N·span·thickness`, `finFrontalLE = finFrontal·s²/(s²+X_t²)`.
  **Split the degenerate guard** (FinSet.cpp:63): `refArea <= 0` still returns `{}` (nothing to
  normalize against), but `bodyRadius == 0` alone must now return the geometry fields with
  cnAlpha = 0 — free-standing fins still have friction and edge drag; only the singular Barrowman
  term drops (§3.4).

### 2d — Whole-airframe resolution + the cache (spec §3.3)

The genuinely new code. In `PartNode`, resolve from `resolvedPlacements()` (each `part::Placed`
carries the part pointer and its resolved `pose`):

- `fore = max(pose.origin.z())`, `aft = min(pose.origin.z() − part->axialLength())`,
  `length = fore − aft` — **over parts with `axialLength() > 0` only**. The Motor is zero-length; an
  aft-protruding motor must not become the aft plane (it would zero the base disc and stretch the
  exposed length — §3.3 explains the trap). If everything is excluded, length stays 0.
- `baseArea = π·r²`, r = max `radiusOuterAt(−axialLength())` over non-zero-length parts whose aft
  plane is within 1e-9 (absolute) of `aft`.
- `maxRadius = sqrt(maxFrontalReferenceArea()/π)`.

**Where to compute it:** inside `ensurePlacementCache()` (PartsModel.cpp:144-157), right after the
resolve + sweep, storing the results in a small mutable cache member next to `resolved_`/`verdict_`
(PartsModel.h:120-121). `compositeAero` then copies the cached trio into the profile it returns.
This is the load-bearing invariant of the whole feature's performance story: **the extent scan runs
once per structural change, and a burn (mass-only) never re-runs it** — it rides `placementDirty_`,
which no mass edit touches. (The spec sketches caching the summed geometry fields too
(`dragGeomCache_`); the additive sums are recomputed by the fold you already hardened, so caching
them is optional — the extent scan is the part that must be gated. Either variant satisfies §5.2.)

### 2e — Tests (model_tests; spec §8 Phase 2, §9.3)

Extend the existing per-part suites with field oracles (hand-compute each):

- BodyTubeTests: `wettedBody == 2π·r_o·L`; other new fields 0.
- NoseConeTests: `wettedBody == π·R·√(R²+L²)`; `noseSinA`, `noseSin2A` against sinφ by hand.
- FinSetTests: `finTcWetted` against the MAC formula; `finFrontal`; `finFrontalLE` with a swept fin
  (cos²Γ = s²/(s²+X_t²)); the `bodyRadius == 0` degenerate now reports geometry with cnAlpha 0.
- AeroTests: extend `CompositeCdIsAdditive` (:112) — every new field sums across the tree.
- New resolution tests (`GeometryProfileTests.cpp` is the natural home): a cone+tube+fins stack has
  the expected length/baseArea/maxRadius; a zero-length Motor at an interior station changes
  nothing; an **aft-protruding Motor** changes nothing (the exclusion-rule pin — build it the way
  `AeroTests.MotorIsMassButNotAeroOrReferenceContributor` (AeroTests.cpp:235) does).
- Gate behavior (PartsModelTests / DiagnosticsGateTests territory): attach/detach/`setLink`
  invalidate the cached extents; `setPartMass` does not. Observe it with a counting stub — a
  minimal `Part` subclass whose `radiusOuterAt`/`axialLength` bump a counter (the `FixedAeroPart`
  pattern at AeroTests.cpp:28 is the precedent for test-local parts): the counter moves after a
  structural verb + read, stays flat after `setPartMass` + read. (There's no test-side friend seam
  into the caches, deliberately — the routed-verb design forbids it.)

**Gate:** full ctest green **and** `diff` your Part-0 baseline flights — byte-identical, because
nothing consumes the new fields yet.

**Self-check:** (a) Why does the drag geometry ride `placementDirty_` and not `massDirty_`? (b) Why
is the aft-plane candidate set filtered by `axialLength() > 0` *and* by the 1e-9 window rather than
just taking the widest part? (c) Why did the `FinSet` guard have to split? (§3.3, §3.4.)

---

## Phase 3 — The calculator

**Goal:** the entire §2 physics as pure functions in `core/model/Aero.{h,cpp}`, oracle-tested,
**still with no production caller**. This is the longest phase and the most fun: every formula has a
closed-form hand value waiting in §9.1 to tell you whether you typed it right.

**Files:** `core/model/Aero.h` (declarations), `core/model/Aero.cpp` (definitions — it's the
deliberately-empty TU, Aero.cpp:3-5), new `core/model/tests/DragBuildupTests.cpp`, and one line in
`core/model/tests/CMakeLists.txt` (add the file to the `model_tests` source list at :2-20 —
`gtest_discover_tests` picks it up from there).

1. **Declare the types** (spec §3.1): `FlowConditions {mach, reynolds, occludedBaseArea}`,
   `DragBreakdown {friction, nose, base, fins, fixed; total()}`, `enum class FinEdge {Rounded,
   Square}`, and the functions `hasDragGeometry(const AeroProfile&)`,
   `dragBreakdown(const AeroProfile&, const FlowConditions&)`, `totalDragCoefficient(...)`.
2. **Declare the helpers in the header too** — `skinFrictionCoeff(re, mach, roughnessOverLength)`,
   `baseDragCoeff(mach)`, `finLeadingEdgeCoeff(mach, FinEdge)`, `finTrailingEdgeFraction(FinEdge)`,
   `noseDrag(S1, S2, mach)`. Not file-local: under `-Werror` the unselected fin-edge branch would
   trip `-Wunused-function`, and a branch reachable only through a constexpr selector is untestable
   (§3.2). The two named constants (`surfaceRoughness = 60e-6`, `finEdge = FinEdge::Rounded`) live
   in Aero.cpp with the derivation comment §2.4 specifies.
3. **Implement test-first, easiest to hardest.** For each helper: copy its §9.1 oracle rows into a
   test, watch it fail, implement from the §2 formulas, watch it pass.
   - `baseDragCoeff` (§2.3): two branches, meet at 0.25 exactly. Oracles: M=0 → 0.12, M=1± → 0.25,
     M=2 → 0.125.
   - `finTrailingEdgeFraction`: 0.5 (Rounded) / 1.0 (Square). Trivial, but it's the seam that
     keeps both edge families compiled. The techdoc's third case — tapered TE, fraction 0 — has
     **no enumerator**: `FinEdge` is deliberately `{Rounded, Square}` (a "tapered LE" law doesn't
     exist, so a Tapered enumerator would leave `finLeadingEdgeCoeff` undefined for it). Tapered
     arrives with the deferred per-fin cross-section attribute (§11 item 2); don't invent the
     enumerator now, and don't write a 0.0 oracle — there's nothing to pass it.
   - `finLeadingEdgeCoeff` (§2.4): three rounded branches + two square. Oracles: rounded M=0.9⁻ vs
     0.9⁺ within 2e-3; M=1 exactly 0.8215 from both sides; square M=0.5 → 0.904453.
   - `skinFrictionCoeff` (§2.1): the max(turbulent, roughness-limited) form, the Re ≤ 1e4 clamp
     **defined as the correlation evaluated at 1e4** (not the 1.48e-2 literal — that's what makes
     the boundary exactly C⁰), Re ≤ 0 → 0, and the M ∈ [0.9, 1.1] compressibility blend. Oracles:
     C_f,turb(1e6) = 4.372e-3; C_f,rough(R_s/L=1.2e-4) = 5.26e-3; blend value at M=1 (smooth) =
     0.9110·C_f.
   - `noseDrag` (§2.2) — the transonic bridge, the only genuinely fiddly function. Implement the
     three anchors first (M ≤ 0.8, M = 1, M ≥ 1.3), test them, then the two bridge segments.
     Mind the honesty note in §2.2: for every realistic cone the (1, 1.3] segment *decreases* — your
     test must not assert monotone-increasing there (§9.1's bridge row spells out what to pin).
   - `dragBreakdown` last: assemble the families per §2, including `hasDragGeometry` (the four-way
     gate — note `noseSinA > 0` is deliberate, §3.1's comment explains the blunt-face rationale),
     the occlusion clamp `max(A_base − A_occl, 0)`, and `Re ≤ 0` zeroing friction outright.
4. **The sweep test** (§9.1 last row): M ∈ [0, 3] step 0.01, fixed Re, a 3FNC-ish profile, **run
   under both `FinEdge` enumerators** (call the helpers with the enum — the production constant
   selects Rounded, but both families stay oracled): every value finite, > 0, adjacent nodes within
   0.05.

**Gate:** `./build/core/model/tests/model_tests --gtest_filter='DragBuildup*'` first, then the full
ctest. Baseline diff: still byte-identical (no caller).

**Self-check:** (a) Why is the low-Re clamp defined by evaluating the correlation rather than a
literal? (b) Why does `hasDragGeometry` require `noseSinA > 0` when a finned tube without a cone is
a perfectly buildable rocket? (c) At which Mach numbers is C_D continuous but not smooth, and why is
that acceptable for RKF45? (§2.1, §3.1, §11 items 9 and 14.)

---

## Phase 4 — Consume it (the behavioral flip)

**Goal:** never-configured rockets and loaded designs fly the computed Cd; a deliberate `setdrag`
still wins. After this phase your Part-0 baseline files *should* diff — that's the feature landing.

**Files:** `core/model/RocketModel.{h,cpp}`, `core/model/DesignSerializer.cpp`.

1. **The override flag** (spec §3.5): `dragCoefficientOverridden{false}` member;
   `setDragCoefficient` now sets it; add `isDragCoefficientOverridden()` /
   `clearDragCoefficientOverride()`. In `installDesign` (RocketModel.cpp:118-122), clear it beside
   the existing area-flag reset (:121).
2. **The `getForces` block**: spec §5.1 gives it verbatim — adapt it in place of the current drag
   lines (RocketModel.cpp:79-85). Read it before pasting; every guard in it is load-bearing:
   the `rho > 0 && speed > 0` early-out (vacuum flights never touch the fold — this is what keeps
   the vacuum-dominated test matrix bit-identical), the `a > 0` / `mu > 0` guards, the mm → m motor
   diameter conversion (MotorModel.h:305 stores mm; Motor.cpp:25 is the precedent), and **no logging
   anywhere in the path** (the sweeps assert no-WARN, DesignMatrixTests.cpp:134-138).
3. **`computedDragBreakdown(mach, altitude, env)`** (§3.5): the display/tests entry point.
   Convention: coast only (`occludedBaseArea = 0`), speed = mach·a(h), Re from ρ·speed·length/μ,
   same guards as flight, `nullopt` when `hasDragGeometry` is false. **First guard:
   `!parts_.hasDesign()` → `nullopt`, before any root access.** A fresh `RocketModel` has a *null
   root* — the old boot HollowSphere placeholder was removed in `aa90de0` — so unlike `getForces`
   (only reachable on a flight, which implies a design), this display path is reachable with no
   design at all, and `parts_.root()->...` would segfault. The phase-5 `cdinfo`-on-a-fresh-session
   test exists to pin exactly this.
4. **The serializer clear — do not skip this.** In `load` (DesignSerializer.cpp:265-272), after the
   unconditional `setDragCoefficient(...)` restore at :266, immediately
   `clearDragCoefficientOverride()`. Why: the setter now raises the override flag, so without the
   clear every loaded design flies *manual* Cd — and `CliDesignCommands.BuildSaveReloadFlyEndToEnd`
   (tests/CliDesignCommandsTests.cpp:63) flies auto before save, manual after reload, and its apogee
   `EXPECT_EQ` (:106) fails under the default Constant Atmosphere. Loaded designs must fly auto
   until phase 6 gives the flag its own attribute. Keep the scalar restore itself — it seeds the
   fallback (§7).
5. **Tests** (spread across RocketModelTests / integration_tests per §8 Phase 4): override-wins
   (manual path bit-identical); `installDesign` clears the cd flag (mirror :27-37); the loader
   clears it; both fallback states — no design (null root ⇒ `computedDragBreakdown` nullopt, no
   crash) and a HollowSphere design (a root, but no drag geometry ⇒ stored-scalar flight, the
   AeroTests.cpp:220-233 situation); vacuum auto flight bit-identical; the 3FNC
   `computedDragBreakdown(0.3, 0)` plausibility band **total ∈ [0.4, 0.85], friction the largest
   family** — §9.2 contains the full hand-derivation; put it in the test comment so the band is
   re-derivable; a transonic auto flight (US Standard 1976) terminating nominally under both
   integrators.

**Gate:** full ctest incl. `-L heavy`. Then re-fly your baseline scripts and *look at* the new
apogees: a 3FNC under auto Cd should land in the same order of magnitude as with Cd = 0.75, not 10×
off. If it's wildly wrong, `cdinfo` doesn't exist yet — instrument via `computedDragBreakdown` in a
scratch test and eyeball which family exploded.

**Self-check:** (a) Why does the manual `dragCoefficient` double as the auto-mode fallback rather
than having a separate default? (b) Why is the loader's flag-clear phase-4 work but the *attribute*
phase-6 work? (§7's two-step rationale.) (c) Why does burnout produce a Cd step, and why is that
fine? (§2.3, §10.)

---

## Phase 5 — CLI surface

**Goal:** users (and the test matrix) can flip modes and inspect the build-up: `setdrag auto`,
`setarea auto`, `cdinfo`, live `status`.

**Files:** `cli/Repl.cpp`, `cli/Repl.h`, `tests/CliDesignCommandsTests.cpp`,
`tests/DesignMatrixTests.cpp`.

1. **`auto` token parsing.** The current handlers call `parseDouble(iss, d)` straight off
   (Repl.cpp:565-595). Restructure: read the token as a string first; if `"auto"`, call the clear
   function and reply `OK setdrag: auto`; else parse it with `parseDoubleStr` (Repl.cpp:122 —
   exists for exactly this read-then-parse pattern). Update the ERR usage strings verbatim to
   `ERR usage: setdrag <cd|auto>` / `ERR usage: setarea <m^2|auto>` — they're the discoverability
   path (§6.1).
2. **`cdinfo [mach [altitude_m]]`** (defaults 0.3, 0): call
   `qtRocket->getRocket()->computedDragBreakdown(mach, alt, *qtRocket->getEnvironment())`
   (QtRocket.h:33) and print the §6.1 block exactly — per-family lines, total, the ref-area line
   with its manual/geometry tag, `ERR cdinfo: no drag geometry (manual/fallback cd = <v>)` on
   nullopt, and the `friction = 0.0000 (n/a: Re=0)` annotation when a = 0.
3. **`status` goes live** (Repl.cpp:720-728): print from the model
   (`isDragCoefficientOverridden()` → `drag_coeff = 0.75 (manual)` vs
   `drag_coeff = auto (0.72 @ M=0.3, h=0)`; `isReferenceAreaOverridden()` →
   `(manual)`/`(geometry)`). Then **delete the staging members** `dragCoeff` (Repl.h:49) and
   `referenceArea` (Repl.h:52) and their three sync sites (:574, :592, :1107-1108) — the compiler
   will find any you miss. Add the help-block lines (:354-355 get the `|auto` forms; `cdinfo` gets
   its own line). **Known pre-existing crash:** `status` on a fresh no-design session already
   segfaults *today* — the mass lines at :724-725 call `getMass(0.0)` on a null root. If your new
   status test crashes there, it's not your regression (see the trap list); write the new
   drag/area lines null-root-safe regardless, and either load a design in every status test or fix
   the mass-line guard as a separate preliminary commit.
4. **cli_tests** (§8 Phase 5): `setdrag auto` → OK + status shows auto; numeric setdrag still OK +
   status manual; `setarea auto`; `cdinfo` fields positive and summing on a built design; `cdinfo`
   ERR on a fresh no-design session (this is the test that pins the `hasDesign()` guard from
   phase 4 — without it, this is a segfault, not an ERR).
5. **The auto-Cd ladder sweep** (design_matrix_tests). Per motor class: `loaddesign`,
   `setatmosphere US Standard 1976`, `setdrag auto`, fly via the `flyApogee` helper (its no-WARN
   assertion is the nominal-termination check); assert apogee > 0 and < the same design's vacuum
   apogee. **Naming trap:** the heavy ctest entry re-runs only
   `--gtest_filter=FlightMatrix.*:Atmosphere.*` (tests/CMakeLists.txt, bottom) — put the sweep in
   the `Atmosphere` suite (or extend that filter in the same commit), or your full-ladder variant
   will silently never run under `-L heavy`. The fast/heavy split itself comes free from the
   existing `QTROCKET_FULL_LADDER` pattern inside the tests — follow the neighboring sweeps.

**Gate:** full ctest incl. `-L heavy`. This is the computed path's first real transonic/supersonic
exercise — an M-class ladder flight crosses Mach 1 both ways. If RKF45 grinds near Mach 1, read §11
item 9 before assuming a bug: brief step-shrink at the C¹ kinks is expected; hangs are not.

**Self-check:** (a) Why did the staging members have to die rather than stay synced? (What could
`status` misreport after `loaddesign` if they lived?) (b) Why is `cdinfo` coast-only by convention?
(§3.5.)

---

## Phase 6 — Persistence

**Goal:** a deliberate manual override survives save/load; everything else keeps loading as auto.

**Files:** `core/model/DesignSerializer.cpp`, `tests/DesignPersistenceTests.cpp`.

1. Writer: bump the version stamp to `"0.3"` (DesignSerializer.cpp:198) and write
   `sim.<xmlattr>.dragCoefficientOverridden` as `"true"`/`"false"` beside the three existing sim
   attributes (:216-219).
2. Loader: gate the phase-4 unconditional clear — clear **unless** the attribute reads exactly
   `"true"` (absent ⇒ false ⇒ auto; that's what keeps all 24 committed fixtures — still version
   0.2, deliberately unregenerated — flying computed-by-default, §7).
3. Tests: re-pin `SaveWritesVersion0_2` (DesignPersistenceTests.cpp:304, assertion :314) to
   `"0.3"` and rename it; manual-cd round-trip survives; absent-attr loads as auto (mirror
   `NonOverriddenReferenceAreaStaysUnoverriddenOnLoad`, :131); extend
   `ReferenceAreaOverrideAndDragRoundTrip` (:110) with the cd-flag assertions.

**Gate:** full ctest incl. `-L heavy`, with **unregenerated** fixtures — if a fixture "needed"
regenerating, your absent-attr default is wrong.

**Self-check:** Why do the fixtures deliberately stay at 0.2? What legacy path would silently lose
coverage if you re-saved them all at 0.3? (§7.)

---

## Phase 7 — Close-out

1. Record the P5 build-up as done wherever the roadmap lives. (`TODO.md` was removed in commit
   `aa90de0`; a line in the final commit message or the docs/ index is enough.)
2. Stale-comment sweep: the `Part::getCompositeAero` mention in Aero.h:21's datum note (it's
   `PartNode::compositeAero` now); any lingering "nothing consumes this" claims around the aero
   seam; the Aero.h `cd` field doc gains "fixed/flow-independent". (BodyTube's "eventual
   contribution" comment died in phase 2.)
3. Coverage run to confirm every new family is exercised:

   ```bash
   cmake --preset coverage-clang
   cmake --build --preset coverage-clang --target coverage
   # report: build-coverage/coverage/html/index.html — look at Aero.cpp specifically
   ```

4. Pre-push CI parity (both are CI-blocking):

   ```bash
   cmake --preset asan-clang && cmake --build --preset asan-clang
   ctest --preset asan-clang -R 'qtrocket_*'
   scripts/run-tidy.sh          # needs the normal build/ present
   ```

---

## Appendix A — Trap list

Collected from the spec's edge-case ledger (§10) and this codebase's known sharp edges; skim before
each phase, re-read when something is mysteriously wrong.

| Trap | Where it bites |
|---|---|
| Positional `AeroComponent{...}` init zeroing appended fields | Any construction site you forgot; the fold (fixed in 2b) was the dangerous one. Grep for `AeroComponent{` after 2a. |
| Motor diameter/length are **mm** in `MotorModel.data` | Occlusion term (phase 4). Motor.cpp:25 is the conversion precedent. Motor *weights* are kg. |
| `USStandardAtmosphere` throws below h = 0 | Every atmosphere query must use the clamped `altitude` (RocketModel.cpp:81), including the two new virtuals. |
| Vacuum: a = 0, μ = 0 | Divide-by-zero without the `a > 0` / `mu > 0` guards; the ρ-early-out means you should never get there anyway. |
| Logging in `getForces` | Sweeps assert no-WARN (DesignMatrixTests.cpp:138) — a hot-path warn is a test failure, and at 6 calls/step it would also be a firehose. |
| Friction correlation is singular at Re ≈ 42 and negative below | Never evaluate it under the 1e4 clamp; Re ≤ 0 zeroes the family outright. |
| `sqrt(M² − 1)` | Only legal at M ≥ 1.3 (the nose supersonic closed form). Nothing else may touch the Prandtl factor near M = 1. |
| Heavy ctest filter `FlightMatrix.*:Atmosphere.*` | Phase 5's sweep must live in a matching suite or extend the filter — otherwise `-L heavy` silently skips it. |
| Serializer sim-block order | The sim block applies **after** `installDesign` (DesignSerializer.cpp:262-272) — that ordering is what lets a restored `true` flag win over the install reset. Don't move it. |
| Fresh session has a **null root** | The boot HollowSphere placeholder died in `aa90de0`; "no design" is a real state. Guard `hasDesign()` in every display path (`computedDragBreakdown`, `cdinfo`, new `status` lines). `status` already segfaults on it today (Repl.cpp:724, pre-existing) — don't mistake that for your regression. |
| `run-clang-tidy -fix` in parallel | Double-applies header fixits. Single-threaded only. |
| Low-thrust from-rest flights | The Propagator's launch-pad support handles these (waits for thrust > weight, warns `NoLiftoff` if it never comes). If a hand-written direct-API test aborts silently at t = 0, give it a small `setvelocity`-equivalent initial speed. |

## Appendix B — Debugging toolbox

- **Interrogate the build-up** without the GUI: after phase 5, `cdinfo` at several Mach numbers is
  the fastest sanity check:

  ```bash
  ./build/cli/qtrocket-cli -c "loadmotors data/Aerotech.rse
  loaddesign tests/data/designs/mid29_basic.qrd
  cdinfo 0.3
  cdinfo 1.1"
  ```

  (Real newlines inside the quoted string — `\n` escapes are not translated by the shell.) Before
  phase 5, call `computedDragBreakdown` from a scratch test.
- **Which family is wrong?** The §9.2 hand-derivation (friction ≈ 0.56, base ≈ 0.12, nose ≈ 0.02,
  fin edges ≈ 0.05 for the reference 3FNC at M = 0.3) is your expectation ruler. A total of ~3 means
  a normalization bug (probably a missing /A_ref); a total of ~0.2 with a cone present means
  `hasDragGeometry` or the friction family is dark.
- **Bisect by phase.** Each phase is a commit; `git stash` / checkout the previous commit and re-fly
  the baseline script to find where a number moved.
- **One flight, one command:** `./build/cli/qtrocket-cli -c "..."` exits 1 on any ERR, so it doubles
  as an assertion.

## Appendix C — Definition of done

- [ ] All seven phases committed, each with a green full ctest (incl. `-L heavy`) at its commit.
- [ ] Baselines: vacuum capture byte-identical from Part 0 through phase 3; atmospheric capture
      byte-identical from its phase-1 re-capture through phase 3; deliberately different from
      phase 4 on, with plausible apogees.
- [ ] ASan+UBSan suite green (`ctest --preset asan-clang -R 'qtrocket_*'`).
- [ ] `scripts/run-tidy.sh` clean.
- [ ] Coverage report shows every §2 family and both `FinEdge` branches exercised.
- [ ] Completion recorded (see Phase 7 — `TODO.md` no longer exists); stale comments swept; every
      new public API has its ≤2-line `@brief` with units, per CLAUDE.md.
- [ ] You can answer every self-check question in this guide without opening the spec.
