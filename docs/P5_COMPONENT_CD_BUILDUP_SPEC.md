# P5 — Component Cd Build-Up: Build-Ready Spec

**Status:** ready to implement (re-based onto the PartsModel architecture) · **TODO item:** P5 ("Component Cd build-up (nose + body friction + base + fins) replacing the single hand-entered Cd, manual override retained. Fill or replace the empty `Aero` struct") · **Date:** 2026-07-17

**Method baseline:** Sampo Niskanen, *OpenRocket technical documentation, for OpenRocket version 13.05* (2013-05-10), §3.4 — "techdoc" hereafter; underlying data S. F. Hoerner, *Fluid-Dynamic Drag* (1965). This is the reference the project's own docs name (DesignWhitepaper appendix A3, TODO.md P5).

**Synthesis basis:** the "minimal-seam" design (winner of the three-judge review, 2–1) with every judge-mandated graft applied: the Hoerner M=1 nose datum with a slope-matched transonic bridge, a per-family `DragBreakdown` + `cdinfo` CLI diagnostic, the auto-Cd motor-ladder sweep test, the `rho <= 0 || speed <= 0` early-out, cache keying on the existing `PartNode` placement-dirty gate (no new revision counter, no parallel invalidation ledger), the trapezoid MAC, `Re <= 0` disabling friction outright, `setarea auto` symmetry, fineness ratio derived from geometry (never from the possibly-overridden reference area), and the fin-edge assumption isolated behind one named constant with both derivations documented.

All file:line references below were verified against the working tree at the PartsModel merge, `c7c7764`.

> **Architecture note (2026-07-17 re-base).** This spec predates the PartsModel refactor. The Part
> tree it was written against — a single `Part` that was also its own composite tree — is gone.
> References throughout now target the as-built architecture (docs/PartsModelDesignLatex/):
>
> | Then (pre-refactor) | Now (PartsModel) |
> |---|---|
> | `sim::AeroComponent` / `AeroProfile` in `core/sim/Aero.h` | `model::AeroComponent` / `AeroProfile` in `core/model/Aero.h` — the whole aero seam moved to the model layer |
> | `Part::getCompositeAero()` (the composite fold) | `PartNode::compositeAero(refArea)` (PartsModel.cpp:224-241) |
> | `Part::maxFrontalReferenceArea()` | `PartNode::maxFrontalReferenceArea()` (PartsModel.cpp:243-251) |
> | `Part::structureRevision()` counter + `RocketModel::aeroCache` keyed on it | the existing `PartNode::placementDirty_` gate — the geometry-static profile caches behind the same gate that already memoizes `resolvedPlacements()` |
> | `RocketModel::setRoot()` | `RocketModel::installDesign()` → `PartsModel::installRoot()` |
> | the drag calculator lives in `sim/` (model calls down) | the calculator lives in `core/model/Aero.cpp`; `getForces` builds `model::FlowConditions` from the `sim::Environment` it already holds and passes them to the pure model-layer calculator |
>
> `Part` is now a pure leaf (per-part mass/geometry/aero only); the tree, placement, composite
> queries, caching, and the single-motor invariant all live in `PartNode`/`PartsModel`, the single
> mutation authority (leaf setters are private to `PartsModel`). Because that layer already carries a
> burn-invariant placement gate, the spec's original revision-counter cache and its pointer-reuse
> (ABA) machinery are dropped — the gate provides the same "compute once per structural change"
> guarantee for free.

---

## 1. Objective, scope & non-goals

Today the entire drag model is one hand-entered scalar: `core/model/RocketModel.cpp:84` (inside
`getForces`) computes `drag = -0.5 * rho * speed * dragCoefficient * referenceArea * velocity` with
`dragCoefficient{1.0}` (RocketModel.h:116) set only by CLI `setdrag` (cli/Repl.cpp:565-576) and the GUI
cannonball tab (gui/CannonballTab.cpp:132-133). The Barrowman seam (`model::AeroComponent.cd`, Aero.h:33)
is populated with literal zeros by every part (`BodyTube::getAero` returns `{}`, BodyTube.cpp:38-44;
cone and fins set `cd = 0`, ConicalNoseCone.cpp:67 / FinSet.cpp:89), `AeroProfile.cd` has no production
consumer, and `deriveReferenceAreaFromGeometry()` (RocketModel.cpp:44-47 — the refactor added it,
delegating to `PartNode::maxFrontalReferenceArea()`) has **zero production callers**: only a test reads
it (AeroTests.cpp:226). Every flight uses the stored area (default 1.134e-3, the 38 mm disc,
RocketModel.h:119), so drag does not scale with airframe class.

**In scope**

- **Computed Cd becomes the default.** `getForces` consumes a per-component drag build-up — skin friction + nose pressure + base + fin pressure — assembled through the `PartNode::compositeAero` seam and evaluated at flight Mach and Reynolds numbers.
- **Manual override retained**, exactly as the P2 spec pre-committed ("manual `dragCoefficient` wins", P2 spec §9): `setDragCoefficient()` gains an override flag mirroring `referenceAreaOverridden` (RocketModel.h:122). CLI `setdrag <cd>` and the GUI cannonball path keep working unchanged.
- **Reference area finally wired**: `getForces` uses a new `effectiveReferenceArea()` — manual override wins, else the geometry-derived max frontal disc (`deriveReferenceAreaFromGeometry()`, already present), else (placeholder body, derived area 0) the stored member. This closes the documented "geometry-derived reference area never reaches the drag law" hardfail.
- **No-design and cannonball stay manual.** A fresh model has no design at all — a *null root* since `aa90de0` removed the boot HollowSphere placeholder; "no design" is a representable state, and every display path guards `hasDesign()` before touching the root. The GUI's cannonball `HollowSphere` design (installed on demand, CannonballTab.cpp:97) has a root but no drag geometry (no `getAero` override, geometry-derived reference area 0 — pinned by AeroTests.cpp:220-233). Both fall back to the stored scalar, bit-identical to today. `HollowSphere` and `Motor` remain aerodynamically inert.
- First production consumers of `AtmosphericModel::getSpeedOfSound` / `getDynamicViscosity` (implemented in all three models since P1, called by nothing).
- CLI: `setdrag auto`, `setarea auto`, `cdinfo`, mode-aware `status`. `.qrd` format 0.2 → 0.3.

**Out of scope (deferred, breadcrumbs left in code — see §11)**

- Angle-of-attack / axial-vs-normal force split (techdoc §3.4.6) — 3-DOF integrates linear DOF only; the force stays `-0.5·rho·|v|·Cd·A·v` along −v̂.
- Per-part surface finish and fin cross-section attributes (`PartParams` + serializer + CLI fields). One global roughness constant and one compile-time fin-edge choice for now.
- Boattail/transition/launch-lug part types and diameter-step pressure drag; recovery/descent modeling.
- Nose shape families beyond the straight cone (the only nose part that exists).
- Barrowman CP consumption / static-margin display — a separate item, gated on the FinSet chordwise sign defect (FinSet.cpp:54-57, :84-88), which this work deliberately does **not** touch: the drag build-up consumes no x_cp.
- Removing `Propagatable::aeroData` (unread back-compat member, Propagatable.h:61) — unrelated cleanup.

---

## 2. Physics

### 2.0 Conventions, symbols, sources

One reference-area convention everywhere: every contribution is normalized to the single shared **A_ref** before summing (the `AeroProfile` additive invariant, `operator+=`, Aero.h:50-57). A_ref = the rocket's max frontal disc, `PartNode::maxFrontalReferenceArea()` (PartsModel.cpp:243-251) — the max over parts, never a sum, never inflated by fins (FinSet.h:71 reports the body disc). SI units throughout.

| Symbol | Meaning | Source at runtime |
|---|---|---|
| M | freestream Mach = \|v\| / a | a = `getSpeedOfSound(h)` |
| Re | Reynolds number on total airframe length = ρ·\|v\|·L/μ | ρ, μ = `getDensity(h)`, `getDynamicViscosity(h)` |
| ρ, μ, a | density, dynamic viscosity, speed of sound at altitude h | atmosphere model |
| L | total exposed airframe length, tip to aft plane (m) | resolved by the composite fold (§3) |
| r_max | max body radius (m) | `sqrt(PartNode::maxFrontalReferenceArea()/π)` — geometry, never the manual area |
| f_B | fineness ratio = L/(2·r_max), clamped ≥ 1 | calculator |
| R_s | surface roughness height = **60 µm** ("regular paint", OpenRocket default finish) | named constant, core/model/Aero.cpp |
| φ | cone half-apex angle; sin φ = R/√(R²+L_n²) for base radius R, nose length L_n | per part |
| N, t, s, c_r, c_t, X_t | fin count, thickness, span, root chord, tip chord, LE sweep length | FinSet accessors |
| cos²Γ | fin LE sweep factor = s²/(s²+X_t²) | per part |
| A_wet | wetted area (m²) | per part |
| A_base, A_occl | aft base disc; motor cross-section occluding it while thrusting | fold / flow state |

Total, decomposed (the `model::DragBreakdown` families, §3):

```
C_D(M, Re) = C_Dfriction + C_Dnose + C_Dbase + C_Dfins + Σ cd_fixed
```

`cd_fixed` is the existing `AeroComponent.cd` pass-through — kept as the flow-independent parasitic-drag hook and the `FixedAeroPart` test contract (AeroTests.cpp:28-39, folded by `CompositeCdIsAdditive` at :112). All concrete parts continue to report `cd = 0` there (the pins at BodyTubeTests.cpp:118, NoseConeTests.cpp:142, FinSetTests.cpp:217 stay green); drag travels in the new geometry fields.

### 2.1 Skin friction — techdoc §3.4.1, eqs. 3.78–3.85

One C_f for the whole rocket from the whole-airframe Re (the calibrated Niskanen convention), applied to each part's wetted area.

Incompressible friction coefficient — the **max** form, exactly continuous in Re (this is how OpenRocket implements the roughness limit; the Re_crit = 51·(R_s/L)^(−1.039) crossover becomes diagnostic only):

```
C_f,turb  = C_f,turb(1e4) ≈ 1.4817e-2        Re ≤ 1e4      (low-Re clamp: one named constant defined as
                                                            the correlation evaluated at exactly Re = 1e4,
                                                            NOT the OpenRocket 1.48e-2 literal — makes the
                                                            clamp boundary exactly C⁰)
C_f,turb  = 1 / (1.50·ln(Re) − 5.6)^2        Re > 1e4      (fully turbulent flat plate)
C_f,rough = 0.032 · (R_s/L)^0.2                            (roughness-limited, Re-independent)
C_f       = max(C_f,turb, C_f,rough)
```

If **Re ≤ 0** (no flow data): the entire friction family is 0 — no boundary-layer claim without flow state. (Defensive only: the §5 early-out means Re > 0 whenever this code runs in a real atmosphere.)

Compressibility (techdoc eqs. 3.81–3.84), blended for C⁰ continuity — the subsonic and supersonic turbulent corrections disagree by ~2.4% at M = 1 (0.900 vs 0.9221), and the roughness corrections by ~6%, so a single linear blend over M ∈ [0.9, 1.1] covers both. That window is the reference implementation's own (OpenRocket blends eqs. 3.82–3.84 over [0.9, 1.1]) and keeps the C¹ kink ~3.3× shallower than a narrow near-M=1 blend would:

```
C_fc,sub(M) = C_f · (1 − 0.1·M²)                                        (both branches, M < 1)
C_fc,sup(M) = max( C_f,turb / (1 + 0.15·M²)^0.58 ,                      (M ≥ 1; the roughness-limited
              C_f,rough / (1 + 0.18·M²) )                                value is floored at the
                                                                          corrected turbulent value)
C_fc(M)    = C_fc,sub                         M ≤ 0.9
           = linear blend sub → sup           0.9 < M < 1.1
           = C_fc,sup                         M ≥ 1.1
```

Scaling to A_ref (techdoc eq. 3.85), body and fin geometry factors carried separately:

```
C_Dfriction = C_fc · [ (1 + 1/(2·f_B)) · A_wet,body  +  A_wet,fins + 2·Σ (t/c̄)·A_wet,finset ] / A_ref
```

- `A_wet,body` = tube lateral areas (`BodyTube::getWettedArea()` = 2π·r_o·L_t, BodyTube.h:51) + cone slant areas (π·R·√(R²+L_n²), new `ConicalNoseCone::getWettedArea()` promoting the private expression at ConicalNoseCone.cpp:35).
- `A_wet,fins` = Σ `FinSet::getWettedArea()` (= 2·N·planform, FinSet.h:70). The `(1 + 2t/c̄)` fin thickness factor is carried additively as `(t/c̄)·A_wet` per set so the composite fold stays field addition.
- `c̄` = trapezoid mean aerodynamic chord = `(2/3)·(c_r + c_t − c_r·c_t/(c_r + c_t))`; the FinSet ctor guarantees c_r + c_t > 0 (FinSet.cpp:29-34).

### 2.2 Nose pressure drag — techdoc §3.4.3; Hoerner ch. 16 cone data

Only the straight cone exists. Each cone exports two additive fields (m²): `noseSin2A = sin²φ·A_n` and `noseSinA = sinφ·A_n`, A_n = π·R². The calculator works on the aggregates **S2 = Σ noseSin2A**, **S1 = Σ noseSinA** (for a single cone this is exactly the per-cone formula; multiple exposed cones share one bridge on the aggregates — a documented approximation for a corner case the current part set barely produces). Define D(M) referenced to A_ref·C_Dnose = D(M)/A_ref:

```
M ≤ 0.8:        D = 0.8·S2                                    (Hoerner low-speed cone pressure drag)
M = 1:          D = S1                                        (Hoerner transonic cone datum, CD• = sin φ)
M ≥ 1.3:        D = 2.1·S2 + 0.5·S1/sqrt(M² − 1)              (Hoerner supersonic cone closed form)

0.8 < M < 1:    D = 0.8·S2 + aₙ·((M − 0.8)/0.2)^bₙ            (constrained power bridge, techdoc eq. 3.87 form)
                aₙ = S1 − 0.8·S2   ( ≥ 0 for any real cone: sinφ ≥ 0.8·sin²φ )
                bₙ = clamp( 0.2·s₁/aₙ , 1, 8 )                 (slope-matched at M = 1 when the clamp
                                                                permits; bₙ = 1 if s₁ ≤ 0)

1 < M < 1.3:    log-linear in M between D(1) = S1 and D(1.3)   (ln D linear in M)
                s₁ = D(1) · ln(D(1.3)/D(1)) / 0.3              (the (1,1.3] segment's dD/dM at M = 1)
```

Continuous by construction at 0.8, 1, and 1.3; equals the Hoerner datum exactly at M = 1; `sqrt(M²−1)` is never evaluated below M = 1.3. Degenerate φ = 0 or no cone (S1 = S2 = 0) contributes 0. This replaces the plain linear fade of the original design (judge-mandated: the linear fade missed the M=1 datum by ~40% of that datum). Honesty note: slope matching engages only for very blunt cones (sinφ ≳ 0.65) — for every practical nose s₁ ≤ 0 or the clamp floors bₙ at 1, leaving a linear bridge with an expected C¹ corner at M = 1 (the model's transonic peak; D legitimately *decreases* on (1, 1.3] whenever sinφ < 0.1896). The bₙ ≥ 1 floor is load-bearing regardless: bₙ < 1 would give an infinite dD/dM at M = 0.8⁺.

### 2.3 Base drag — techdoc eq. 3.94 (Hoerner)

```
C_D•base = 0.12 + 0.13·M²        M < 1
         = 0.25 / M              M ≥ 1        (exactly continuous: both give 0.25 at M = 1 — pinned)

C_Dbase  = C_D•base · max(A_base − A_occl, 0) / A_ref
```

`A_base = π·r_aft²` with r_aft resolved by the composite fold (§3). **A_occl = π·(d_motor/2)²** while the motor produces thrust (`PartsModel::thrust(t) > 0`), else 0 — the techdoc treatment: the exhaust plume fills the base region during burn, full base drag after burnout. `MotorModel.data.diameter` is stored in **mm** (MotorModel.h:305; Motor.cpp:25 confirms the /1000 convention) — convert. Missing diameter (0) degrades to full base drag — safe. The Cd step at burnout is benign: thrust itself steps to 0 there; RKF45 handles it by step rejection, RK4 by design.

### 2.4 Fin pressure drag — techdoc §3.4.4 (cylinder-crossflow LE + TE base)

Fins are attribute-less flat plates (thickness only, FinSet.h:65). **Default edge family: rounded LE and rounded TE** — the hobby norm (builders sand edges), and the only choice that keeps a classic 3FNC inside the empirically observed C_D ≈ 0.4–0.8 band: at realistic fin-frontal/reference-area ratios (N·t·s/A_ref ≈ 0.3–0.8 for thick fins on small tubes), a square-LE stagnation model alone contributes ~0.25–0.6 (verified by hand in the design review). The choice is isolated behind **one named compile-time constant in Aero.cpp** with both laws implemented and both derivations documented in the same comment block, so it flips in one line when a `FinSet` cross-section attribute lands (§11):

```cpp
// Aero.h:  enum class FinEdge { Rounded, Square };   (public: the helpers are edge-parameterized so
//                                                     BOTH laws compile, run, and are unit-tested)
// Aero.cpp: inline constexpr FinEdge finEdge = FinEdge::Rounded;   // the production default
```

Rounded LE (cylinder crossflow; mutually continuous at M = 0.9 and exactly at M = 1.0):

```
M < 0.9:        C_D•LE = (1 − M²)^(−0.417) − 1
0.9 ≤ M < 1:    C_D•LE = 1 − 1.785·(M − 0.9)
M ≥ 1:          C_D•LE = 1.214 − 0.502/M² + 0.1095/M⁴
```

Square LE (the alternative, implemented and unit-tested through the edge-parameterized helpers — not selected by the production constant): stagnation law `C_D•LE = 0.85·(q_stag/q)` with `q_stag/q = 1 + M²/4 + M⁴/40` (M < 1), `1.84 − 0.76/M² + 0.166/M⁴ + 0.035/M⁶` (M ≥ 1).

Trailing edge: rounded TE = **half** the base-drag law; square would be full, tapered zero (techdoc §3.4.4):

```
C_D•TE = finTrailingEdgeFraction(edge) · C_D•base(M)      (0.5 rounded / 1.0 square; the techdoc's
                                                           tapered-TE 0 has no enumerator today —
                                                           it lands with the §11 cross-section attr)
```

Applied to fin frontal areas, additive fields per set: `finFrontal = N·s·t` and `finFrontalLE = finFrontal·cos²Γ`:

```
C_Dfins = [ C_D•LE(M) · Σ finFrontalLE  +  C_D•TE(M) · Σ finFrontal ] / A_ref
```

Fin friction is inside §2.1 via the wetted-area terms — not double-counted here.

### 2.5 Interference

**Neglected, explicitly**, matching the baseline ("interference drag explicitly neglected", techdoc §3.4). Fin-body interference already appears on the normal-force side via K_fb (FinSet.cpp:76); no drag analogue is added.

### 2.6 Envelope behavior (1/4A → M ladder, transonic/supersonic)

Every family has an explicit M ≥ 1 branch valid to M ≈ 3–4 (the Hoerner data range; above that the formulas evaluate finitely and tend to constant or decaying values — base/TE → 0, rounded LE → 1.214, nose → 2.1·S2, friction slowly decaying — documented, unvalidated). Every branch boundary is continuous by construction:

| Boundary | Mechanism | Residual |
|---|---|---|
| Re crossover (turb ↔ rough) | max() form | exactly C⁰ in Re |
| M ∈ [0.9, 1.1] friction | linear blend (the source's own window) | exactly C⁰ (bridges 2.4% turb / 6% rough gaps at M = 1) |
| M = 0.8, 1.0, 1.3 nose | anchored bridge, log-linear segment | exactly C⁰, hits sin φ datum at M = 1 |
| M = 1 base / TE | 0.12 + 0.13 = 0.25 = 0.25/1 | exactly C⁰ |
| M = 0.9 rounded LE | (0.19)^(−0.417) − 1 = 0.9987 vs 1.0 | 0.13% (accepted) |
| M = 1.0 rounded LE | 0.8215 from both branches | exactly C⁰ |
| M = 1.0 square LE (unselected family) | 1.275 vs 1.281 | 0.5% (accepted — matches OpenRocket; the §9 sweep runs under both `FinEdge` values so the one-line flip stays covered) |

`sqrt(M²−1)` is never evaluated below M = 1.3; nothing divides by the Prandtl factor near M = 1. C_D(M) is C⁰ but not C¹ at the boundaries — see §11 for the RKF45 note. A dense-grid continuity/finiteness sweep over M ∈ [0, 3] is a named unit test (§9).

---

## 3. Exact API & file changes

Layering: the aero seam now lives entirely in the model layer — `core/model/Aero.h` (Part.h:15 includes `model/Aero.h`) defines the `model::AeroComponent`/`AeroProfile` value types, and `getForces` already takes `sim::Environment&`. So **both the geometry and the drag calculator live in model/**; `getForces` reads Mach/Re from the `sim::Environment` it already holds, packs them into a `model::FlowConditions` value, and calls the pure model-layer calculator (`core/model/Aero.cpp`, today a near-empty TU — Aero.cpp:3-5). The calculator itself includes no sim/ header (it takes `FlowConditions`, not an atmosphere), so no new layering edge is created. utils/ untouched (Constants.h already has γ, R*, molar mass, Sutherland β and S).

### 3.1 `core/model/Aero.h` — append-only struct growth + new value types

All existing `AeroComponent` construction is positional 3-element aggregate init (the composite fold at PartsModel.cpp:238, ConicalNoseCone.cpp:67, FinSet.cpp:89, AeroTests.cpp:115-120); appended members value-initialize to 0, so nothing breaks. **Never insert before `cd`, and the fold at PartsModel.cpp:238 must copy-then-modify (§3.3) so it can never re-zero an appended field.**

```cpp
struct AeroComponent
{
   double cnAlpha{0.0};      // unchanged
   double cnAlphaXcp{0.0};   // unchanged
   double cd{0.0};           // unchanged: fixed flow-independent contribution, normalized to refArea
   // drag-relevant geometry, all m^2, all additive across parts:
   double wettedBody{0.0};    // body skin area (tube lateral, cone slant)
   double finWetted{0.0};     // both faces of all fins in the set
   double finTcWetted{0.0};   // (t/cbar) * finWetted -- carries the (1 + 2t/cbar) friction factor additively
   double finFrontal{0.0};    // N * span * thickness (TE base reference)
   double finFrontalLE{0.0};  // finFrontal * cos^2(LE sweep)
   double noseSin2A{0.0};     // sin^2(half-angle) * nose base area
   double noseSinA{0.0};      // sin(half-angle)  * nose base area
};

struct AeroProfile
{
   // existing five fields unchanged. the profile mirrors the seven new component fields by name as
   // accumulators (wettedBody ... noseSinA); operator+= sums component field into profile field, and
   // hasDragGeometry/dragBreakdown read them off the profile.
   // resolved by the composite fold (whole-airframe, not additive from components):
   double baseArea{0.0};   // m^2, aft exposed base disc
   double length{0.0};     // m, tip-to-aft exposed length (Re + fineness input)
   double maxRadius{0.0};  // m, widest frontal radius from geometry (fineness input; never the manual area)
};

/// freestream state for one drag evaluation. mach/reynolds 0 mean "no flow data" (vacuum guard).
struct FlowConditions
{
   double mach{0.0};              // |v|/a; 0 when a == 0
   double reynolds{0.0};          // rho*|v|*length/mu; <= 0 disables friction terms
   double occludedBaseArea{0.0};  // m^2 of base filled by motor exhaust while thrusting
};

/// per-family decomposition of the build-up (the cdinfo/status payload). all normalized to refArea.
struct DragBreakdown
{
   double friction{0.0}, nose{0.0}, base{0.0}, fins{0.0}, fixed{0.0};
   double total() const { return friction + nose + base + fins + fixed; }
};

/// true when the profile carries enough geometry for a credible build-up:
/// refArea > 0 && length > 0 && (wettedBody + finWetted) > 0 && noseSinA > 0.
/// the nose term gates nose-less (blunt-fore-face) airframes onto the manual/fallback scalar:
/// the build-up has no stagnation blunt-face family, so friction + base alone (~0.2) would be
/// confidently wrong for a flat-faced cylinder (~0.8). lifting the gate = the deferred
/// techdoc fore-face term (see risks).
bool hasDragGeometry(const AeroProfile& p);

DragBreakdown dragBreakdown(const AeroProfile& p, const FlowConditions& f);   // defined in Aero.cpp
double totalDragCoefficient(const AeroProfile& p, const FlowConditions& f);   // = dragBreakdown(p,f).total()
```

### 3.2 `core/model/Aero.cpp` — the calculator (fills the deliberately-empty TU, per its own comment at Aero.cpp:3-5)

Definitions of `hasDragGeometry`, `dragBreakdown`, `totalDragCoefficient`, plus the named constants:

```cpp
inline constexpr double surfaceRoughness = 60e-6;               // m, "regular paint" (OpenRocket default finish)
inline constexpr FinEdge finEdge = FinEdge::Rounded;            // production default; derivation block in §2.4
```

Helpers, **declared in `Aero.h`**, not file-local — under `-Werror` an unselected file-local branch trips `-Wunused-function`, and a branch only reachable through a constexpr selector is untestable dead code. Edge-parameterized so both fin-edge families stay compiled, called, and oracled: `skinFrictionCoeff(reynolds, mach, roughnessOverLength)`, `baseDragCoeff(mach)`, `finLeadingEdgeCoeff(mach, FinEdge)`, `finTrailingEdgeFraction(FinEdge)` (0.5 rounded / 1.0 square — no Tapered enumerator: a tapered *LE* law doesn't exist, so the enum stays two-valued and the techdoc's tapered-TE 0 defers to the §11 cross-section attribute), `noseDrag(S1, S2, mach)`. `dragBreakdown` selects via the `finEdge` production constant; `DragBuildupTests` calls the helpers with both enumerators.

### 3.3 `core/model/PartsModel.{h,cpp}` — the composite seam (`PartNode`)

The composite aero fold and the whole-airframe resolution now live on `PartNode` (the owned tree node), not on `Part`. `Part` stays a pure leaf: its `getAero(double refArea)` (Part.h:75) is unchanged, so the test-only `FixedAeroPart` override (AeroTests.cpp:33) compiles untouched.

- **Fold hardening** (PartsModel.cpp:238). `PartNode::compositeAero` currently re-aggregates each part positionally:
  ```cpp
  profile += model::AeroComponent{c.cnAlpha, c.cnAlphaXcp + c.cnAlpha * cmStationZ, c.cd};
  ```
  That positional brace-init silently drops any appended `AeroComponent` field to 0. Replace it with copy-then-modify so the seven new geometry fields survive the fold:
  ```cpp
  model::AeroComponent adj = pl.part->getAero(refArea);
  adj.cnAlphaXcp += adj.cnAlpha * cmStationZ;   // re-express x_cp onto the sub-tree-root datum
  profile += adj;
  ```
  and extend `AeroProfile::operator+=` (Aero.h:50-57) to sum the seven geometry fields.
- **Whole-airframe resolution** in `PartNode::compositeAero` (PartsModel.cpp:224-241), after the fold loop, from the already-cached `resolvedPlacements()` (`std::span<const part::Placed>`; each `Placed` carries the part and its resolved `pose`). Parts with `axialLength() == 0` (zero-length point parts — today the Motor) are **excluded from both the extent scan and base-plane candidacy**: production motors link at an interior station, but nothing prevents an aft-protruding one (`AeroTests.MotorIsMassButNotAeroOrReferenceContributor`, AeroTests.cpp:235, builds exactly that), and under an inclusive rule its point would *become* the aft plane — silently zeroing the base disc and inflating the exposed length.
  - `fore = max over placed of pose.origin.z()`; `aft = min over placed of (pose.origin.z() − part->axialLength())`; `profile.length = fore − aft` — all over non-zero-length parts. If every placed part is excluded, `length` stays 0 → `hasDragGeometry` false → fallback.
  - `profile.baseArea = π·r²` with r = max `radiusOuterAt(−axialLength())` over non-zero-length parts whose aft plane is within **1e-9 absolute** of `aft` (a documented convention: abutting parts land near-coincident via summed doubles).
  - `profile.maxRadius = sqrt(maxFrontalReferenceArea() / π)` (PartsModel.cpp:243-251; geometry-only, fins don't inflate it).
- **Caching — the placement gate, not a revision counter.** The geometry-static profile (the seven per-part sums plus baseArea/length/maxRadius) is a pure function of resolved geometry and is burn-invariant — it changes only on a structural or link edit. That is exactly what `PartNode`'s existing `placementDirty_` flag (PartsModel.h:119) already gates: it guards `resolved_` and the overlap `verdict_`, and every structural verb (`attach`/`attachSubtree`/`detach`/`setLink`/`installRoot`) calls `markPlacementDirty()` (PartsModel.h:105-106), which walks to the root. So the refArea-independent geometry is memoized in `PartNode` behind that same gate, alongside `resolved_`:
  ```cpp
  // PartNode, private, next to resolved_ / verdict_ (PartsModel.h:120-121):
  mutable model::AeroProfile dragGeomCache_;   // geometry-static portion; refArea applied by the caller
  ```
  rebuilt inside `ensurePlacementCache()` (PartsModel.cpp:144-157) whenever the placements re-resolve; `compositeAero(refArea)` then reads the memo and applies refArea (normalizing the Barrowman fields, stamping `refArea`) per call. A burn dirties only `massDirty_`, never `placementDirty_`, so the drag geometry is built once per structural change and reused bit-for-bit across every integrator stage of the flight. This drops the spec's original `Part::structureRevision()` counter, the `RocketModel`-side `aeroCache`, and the pointer-reuse (ABA) reset entirely — the gate is the cache key, and `installRoot` already dirties a freshly installed root (PartsModel.cpp:393-394), so a new design cannot read a stale cache.

### 3.4 `core/model/parts/` — concrete parts

- **BodyTube** (`BodyTube.cpp:38-44`): fill `wettedBody = getWettedArea()`. `cnAlpha`/`cd` unchanged (0). Delete the stale "eventual contribution" comment.
- **ConicalNoseCone**: new public `double getWettedArea() const` = `π·R·√(R²+L²)` (promote the private slant expression, ConicalNoseCone.cpp:35). `getAero` (cpp:57-68) additionally fills `wettedBody = getWettedArea()`, `noseSin2A = sin²φ·π·R²`, `noseSinA = sinφ·π·R²`. CNalpha/CP math untouched.
- **FinSet** (`FinSet.cpp:60-90`): fill `finWetted = getWettedArea()`, `finTcWetted = (thickness/c̄)·finWetted` (c̄ = trapezoid MAC, §2.1), `finFrontal = N·span·thickness`, `finFrontalLE = finFrontal·s²/(s²+X_t²)`. The degenerate branch splits: `refArea <= 0` still returns `{}` (nothing to normalize against); `bodyRadius == 0` now returns the geometry fields with cnAlpha 0 — free-standing fins still have friction and edge drag, only the singular Barrowman term drops.
- **Motor, HollowSphere**: unchanged — aerodynamically inert (preserves the AeroTests.cpp:235-268 motor-inert intent; the cannonball contract).

### 3.5 `core/model/RocketModel.h` / `.cpp`

```cpp
// members:
bool dragCoefficientOverridden{false};   // mirrors referenceAreaOverridden (RocketModel.h:122)
// no RocketModel-side aero cache: the geometry-static profile lives in PartNode behind the
// placement gate (§3.3), so RocketModel keeps no pointer/revision key of its own.

// API:
void   setDragCoefficient(double d) { dragCoefficient = d; dragCoefficientOverridden = true; } // was RocketModel.h:64
bool   isDragCoefficientOverridden() const { return dragCoefficientOverridden; }
void   clearDragCoefficientOverride() { dragCoefficientOverridden = false; }
void   clearReferenceAreaOverride()   { referenceAreaOverridden = false; }

/// manual override wins; else the geometry-derived max disc (deriveReferenceAreaFromGeometry(),
/// RocketModel.cpp:44-47); else (derived == 0, placeholder body) the stored member. the drag
/// law's area, always. the override check short-circuits before any tree walk; the derived disc is
/// a cheap uncached max-over-parts walk (PartNode::maxFrontalReferenceArea, PartsModel.cpp:243-251),
/// on the order of the existing per-stage compositeMass walk (RocketModel.cpp:36).
double effectiveReferenceArea() const;

/// build-up at one condition for display/tests (cdinfo, status). nullopt with no design
/// (!parts_.hasDesign() -- the guard runs before any root access; a fresh model's root is null)
/// or when the tree has no drag geometry. display convention: speed = mach * a(h), Re = rho*speed*length/mu,
/// occludedBaseArea = 0 (coast) — always; burn-state occlusion is a unit-test concern (§9.1).
/// under an atmosphere with a == 0 (vacuum) the display Re is 0 and friction reads 0 —
/// callers annotate rather than imply a computed zero. re/mach guards identical to flight.
std::optional<model::DragBreakdown> computedDragBreakdown(double mach, double altitude,
                                                          sim::Environment& env);
```

- `installDesign` (cpp:118-122) already resets `referenceAreaOverridden` after the atomic root replace (cpp:121); add `dragCoefficientOverridden = false;` beside it — a freshly installed airframe inherits neither manual override. No aero-cache reset is needed: the geometry-static profile lives in `PartNode` behind the placement gate, and `installRoot` dirties the new root (PartsModel.cpp:393-394), so the first `getForces` after an install rebuilds it. Motor in-place swaps (`PartsModel::setMotor`, PartsModel.cpp:470-484) need nothing — `Motor` is aero-inert and the occlusion diameter is read live per stage.
- `getForces` (cpp:64-88): see §5.
- `getDragCoefficient()` semantics unchanged: returns the stored scalar (the manual value / auto-mode fallback).

### 3.6 Other files

| File | Change |
|---|---|
| `core/model/DesignSerializer.cpp` | §7 in two steps: Phase 4 flag-clear after the scalar restore (load, cpp:266); Phase 6 version "0.3" (writer stamp cpp:198) + `dragCoefficientOverridden` attr (writer cpp:216-219) + gated restore |
| `cli/Repl.cpp`, `cli/Repl.h` | §6: `setdrag auto`, `setarea auto`, `cdinfo`, live `status`; delete the `dragCoeff`/`referenceArea` staging members (Repl.h:49, :52) and their sync sites (:574, :592, :1107-1108) |
| `core/model/tests/` | new `DragBuildupTests.cpp` (+ CMakeLists registration) — Aero is a model-layer TU now, so its tests live beside the other `core/model/tests/` aero suites |
| `gui/` | **no change** (CannonballTab.cpp:132-133 already means "manual mode") |
| `core/QtRocket.h/.cpp` | **no change** — clients reach everything through `getRocket()` |
| `core/sim/AtmosphericModel.h` + models | **no change** (§4) |
| `data/`, `tests/data/designs/*.qrd` | **no change** (§7 — fixtures load as auto without regeneration) |
| roadmap | record P5 build-up completion (phase 7) — `TODO.md` was removed in `aa90de0`; note it in the final commit message / docs index |

---

## 4. Atmosphere models: what the build-up consumes

**No interface change.** `sim::AtmosphericModel` (AtmosphericModel.h:13-18) has had all five pure virtuals — `getDensity`, `getPressure`, `getTemperature`, `getSpeedOfSound`, `getDynamicViscosity` — since P1. This work lands the **first production consumers** of `getSpeedOfSound` and `getDynamicViscosity` (previously called by nothing, including tests — the documented dead-code entry).

| Model | ρ, T | a | μ | Hazards |
|---|---|---|---|---|
| `USStandardAtmosphere` | 1976 layer tables | `sqrt(γ·R*·T/M̄)` (cpp:143-147) | Sutherland `β·T^1.5/(T+S)` (cpp:150-153) | **throws `std::out_of_range` below h = 0** (via `utils::Bin`) — every query must go through the existing altitude clamp (RocketModel.cpp:81). Above 71 km: extrapolates the last layer, no throw. |
| `ConstantAtmosphere` | 1.225, 288.15 | 340.294 | 1.78938e-5 | none. Mach and Re vary only with speed — sea level everywhere. |
| `VacuumAtmosphere` | 0, 0 | **0** | **0** | a = 0 (getSpeedOfSound, VacuumAtmosphere.h:22) is the divide-by-zero to guard; μ = 0 at :24. Handled twice over: the ρ ≤ 0 early-out (§5) skips all flow math, and the `mach = a > 0 ? speed/a : 0` guard stands anyway. μ = 0 → Re = 0 → friction disabled (§2.1). |

Semantics contract, stated once in the spec and enforced by guards: **Vacuum** = no medium (drag force identically zero via ρ = 0; no flow state is ever computed). **Constant** = sea-level ISA at every altitude (altitude-independent Mach/Re at a given speed). **US Standard 1976** = full altitude dependence; all new queries reuse the clamped altitude.

---

## 5. Flow-state plumbing per integrator stage + caching

### 5.1 The `getForces` block

Per ODE stage (RK4 4×, RKF45 6× per attempted step — the derivative closure at Propagator.cpp:34-41 calls `getForces`; a rejected RKF45 step re-runs all 6), replacing RocketModel.cpp:79-85:

```cpp
auto atmosphere = environment.getAtmosphericModel();
const double altitude = position[2] > 0.0 ? position[2] : 0.0;   // existing clamp: guards ALL queries
const double rho   = atmosphere->getDensity(altitude);
const double speed = velocity.norm();

if(rho > 0.0 && speed > 0.0)   // early-out: vacuum flights never build flow state or touch the fold
{
   const double area = effectiveReferenceArea();
   double cd = dragCoefficient;                     // manual value doubles as the auto-mode fallback
   if(!dragCoefficientOverridden)
   {
      // placement-gated: geometry rebuilds only on a structural edit (§3.3), so this is a cache
      // read plus the cheap Barrowman re-fold. refArea is stamped into the returned profile.
      const model::AeroProfile geo = parts_.root()->compositeAero(area);
      if(model::hasDragGeometry(geo))
      {
         const double a  = atmosphere->getSpeedOfSound(altitude);
         const double mu = atmosphere->getDynamicViscosity(altitude);
         model::FlowConditions flow;
         flow.mach     = (a  > 0.0) ? speed / a : 0.0;
         flow.reynolds = (mu > 0.0) ? rho * speed * geo.length / mu : 0.0;
         if(parts_.thrust(t) > 0.0)
         {
            const double rm = parts_.motorModel()->data.diameter * 1e-3 / 2.0;  // mm -> m
            flow.occludedBaseArea = std::numbers::pi * rm * rm;
         }
         cd = model::totalDragCoefficient(geo, flow);
      }
   }
   forces += -0.5 * rho * speed * cd * area * velocity;   // |v|*v form retained (zero-safe)
}
```

`parts_.root()` is non-null on any flight (`getMass` already dereferences it, RocketModel.cpp:36) — but only there: the display path (`computedDragBreakdown` → `cdinfo`/`status`) is reachable on a fresh no-design session, whose root is null, and must guard `hasDesign()` itself (§3.5). No logging anywhere in this path — the flyApogee no-WARN assertion (DesignMatrixTests.cpp:138) makes hot-path warns test failures.

### 5.2 Geometry-static vs flow-dependent split

| Class | Contents | Recomputed |
|---|---|---|
| **Geometry-static** | the `AeroProfile` geometry: wetted areas, fin ratios, nose sines, base area, length, maxRadius | once per structural change, via `PartNode`'s placement gate (§3.3) |
| **Flow-dependent** | Mach, Re, occlusion, the §2 branch evaluations | every stage: 2 extra atmosphere virtuals + ~8 `pow`/`sqrt`, ~50 flops, zero allocation |

**Dirty-flag interaction.** The cache rides `PartNode::placementDirty_` (PartsModel.h:119) — the same gate that already memoizes `resolvedPlacements()`/`placementDiagnostics()`. Every structural edit routes through a `PartsModel` verb (`attach`/`detach`/`setLink`/`installRoot`), each of which calls `markPlacementDirty()` up to the root (PartsModel.h:105-106); leaf setters are private to `PartsModel` (the single mutation authority), so there is no back door that could edit geometry without dirtying the gate — the "mutate via `getTopPart()`" corner the old design guarded against is now structurally unrepresentable. A burn dirties only `massDirty_`, never `placementDirty_` — correct, geometry is burn-invariant. Motor install (first `setMotor`) attaches a child → placement dirty → one harmless rebuild; an in-place motor swap dirties only mass and needs nothing. `installRoot` dirties a freshly installed root, so there is no pointer-reuse (ABA) corner and no hand-maintained reset.

**Budget.** `PartNode::compositeAero` re-folds over the *already-cached* `resolvedPlacements()` each call (placement resolution is cached, PartsModel.cpp:144-169); memoizing the geometry-static profile behind the placement gate removes even that per-stage fold, leaving a cache read plus the Barrowman re-fold. Net per stage: ~10² flops + two atmosphere virtuals — noise against the existing per-stage `compositeMass` walk (RocketModel.cpp:36) and US-Std `exp`/`pow` density evaluation. Rejected RKF45 steps re-run 6 stages against the warm cache. Vacuum suites (the bulk of the matrix) take the early-out and are bit-identical to today, at today's cost.

---

## 6. CLI / GUI surface

### 6.1 Command grammar (`cli/Repl.cpp`)

```
setdrag <cd>                 unchanged: manual Cd override (parseDouble; flag now set via setter)
setdrag auto                 NEW: clearDragCoefficientOverride(); computed build-up applies
setarea <m^2>                unchanged: manual area override (>= 0 enforced at :588)
setarea auto                 NEW: clearReferenceAreaOverride(); geometry-derived disc applies
cdinfo [mach [altitude_m]]   NEW: print the DragBreakdown at the given condition (defaults 0.3, 0)
```

Token check for `auto` before `parseDouble` in both handlers. Replies:

```
OK setdrag: auto
OK setarea: auto
OK cdinfo: M=0.30 h=0 m Re=3.5e6
  friction = 0.5213
  nose     = 0.0229
  base     = 0.1211
  fins     = 0.0512
  fixed    = 0.0000
  total    = 0.7165   (ref_area 4.524e-4 m^2, geometry)
ERR cdinfo: no drag geometry (manual/fallback cd = 1)
```

Under an atmosphere with a = 0 (Vacuum) the display Re is 0 (§3.5 convention); `cdinfo` prints `friction = 0.0000 (n/a: Re=0)` rather than implying a computed zero. ERR usage strings are updated verbatim — they are the discoverability path for the new token: `ERR usage: setdrag <cd|auto>` (Repl.cpp:570), `ERR usage: setarea <m^2|auto>` (:583); `cdinfo` gains its line in the `#` help block (:345-374) alongside the existing `setdrag`/`setarea` lines (:354-355). All owned by Phase 5.

`cdinfo` calls `computedDragBreakdown(mach, altitude, env)` via `qtRocket->getEnvironment()`; ERR when it returns nullopt. Existing replies `OK setdrag: <cd>` (:575) and `OK setarea: <a> m^2` (:593) are load-bearing (`ok()` assertions, DesignMatrixTests.cpp:126, :545, :549) — unchanged.

### 6.2 `status` (:720-728) — reads the model live

The `dragCoeff`/`referenceArea` staging members (Repl.h:49, :52) exist only for this display (read at :726-727); delete them and their sync sites (setdrag :574, setarea :592, loaddesign :1107-1108). No test asserts on status output (verified by grep) — display-only risk.

```
drag_coeff = 0.75 (manual)
drag_coeff = auto (0.72 @ M=0.3, h=0)          # via computedDragBreakdown(0.3, 0, env).total()
drag_coeff = auto (no drag geometry; fallback 1)
ref_area   = 0.001134 m^2 (manual)
ref_area   = 0.000452 m^2 (geometry)
```

Help text (:354-355) gains the `auto` forms and `cdinfo`.

### 6.3 GUI

**No change.** `CannonballTab.cpp:132-133` pushes textbox Cd/area through the setters, which now mean "manual override" — exactly its semantics today. The cannonball HollowSphere design it installs (behind its `hasDesign()` guard, :97) additionally has no drag geometry, so a GUI cannonball flies the stored scalar, bit-identical to today.

---

## 7. `.qrd` persistence & fixture migration

- **Format version `0.2` → `0.3`** (writer stamp at DesignSerializer.cpp:198). The loader is already forward-tolerant within major 0 (:233), so no loader version-handling change; the bump is honest about the semantic change.
- **Writer** (:216-219): add `sim.<xmlattr>.dragCoefficientOverridden` (`"true"/"false"`) beside the existing three sim attributes (`dragCoefficient` :216, `referenceArea` :217, `referenceAreaOverridden` :218-219).
- **Loader** (:265-272), in two steps matching §8: **Phase 4** — keep restoring the `dragCoefficient` scalar **unconditionally** (:266; it seeds the auto-mode fallback — do not adopt the drop-the-scalar variant, which makes `loaddesign` state-dependent), then immediately `clearDragCoefficientOverride()`, so loaded designs fly auto from Phase 4 onward. This clear is what keeps `BuildSaveReloadFlyEndToEnd` green mid-migration (auto before save ⇒ auto after reload; see the Phase 4 impact audit). **Phase 6** — gate that clear on the new attr: cleared unless it reads exactly `"true"`. `referenceArea` load semantics unchanged (override-gated, :267-272). Order preserved: the sim block applies after `installDesign` (:262), so a restored `true` flag wins over the install's reset (the comment at :265 stays accurate for both flags).
- **Present-but-not-overridden semantics / fixture migration: none needed.** All 24 flyable fixtures carry `dragCoefficient="1"` (large54_multi.qrd: `"0"`) with no override attr → absent ⇒ false ⇒ **every fixture loads as auto** with the stored value as inert fallback. Computed-by-default is not defeated by legacy files, and large54_multi's fallback-0 scalar is never consulted (it has full geometry). Fixtures stay at version 0.2 deliberately — they pin the legacy-load path.
- **Documented cost:** a pre-0.3 file saved after a deliberate manual `setdrag` loads as auto (none exist in-tree outside tests that set it programmatically); one `setdrag` restores it.

---

## 8. Implementation phases

Tree green after every phase: `-Werror` build + full `ctest --test-dir build -R 'qtrocket_*'` (incl. `-L heavy` where named). Each phase is one landable commit-sized step. Comments follow CLAUDE.md (terse, lowercase, why-not-what).

**Phase 1 — wire the reference area** (the docs' named prerequisite defect, isolated so any drag-sweep shift is unambiguous).
Files: RocketModel.h/.cpp. Add `effectiveReferenceArea()` + `clearReferenceAreaOverride()` and have `getForces` consume it — `deriveReferenceAreaFromGeometry()` already exists (RocketModel.cpp:44-47), so this is only the wrapper plus its first production consumer.
Tests: RocketModelTests — geometry area used when not overridden; placeholder (derived 0) falls back to the stored member; override still wins; `installDesign` still clears the flag (already pinned at RocketModelTests.cpp:27-37).
Impact audit: PhysicsIntegrationTests set the area manually (:69) — unaffected; DesignMatrix drag sweep assertions are ordinal (`0 < drag < vac`, :544-554) — robust to magnitude change; CliDesignCommands end-to-end compares a design against itself — self-consistent.
Exit: full ctest green incl. `-L heavy`; `deriveReferenceAreaFromGeometry` has its production caller.

**Phase 2 — extend the seam, geometry only** (no consumer; trajectories bit-identical).
Files: Aero.h (append the seven `AeroComponent` fields; `baseArea`/`length`/`maxRadius` on `AeroProfile`; extend `operator+=`), PartsModel.{h,cpp} (the copy-then-modify fold fix in `PartNode::compositeAero`; extents/base/maxRadius resolution; the placement-gated `dragGeomCache_` memo + its rebuild in `ensurePlacementCache()`), BodyTube.cpp, ConicalNoseCone.h/.cpp (`getWettedArea()`), FinSet.cpp (incl. the degenerate-branch split).
Tests (model_tests): per-part field oracles (tube 2π·r_o·L; cone slant; fin t/c̄, frontal, cos²Γ) — extend BodyTubeTests/NoseConeTests/FinSetTests; composite additivity of every new field (extend AeroTests `CompositeCdIsAdditive`); length/baseArea/maxRadius resolution on a cone+tube+fins stack, on a stack with a zero-length Motor at an interior station, and on one with an **aft-protruding Motor** (length and baseArea identical with and without it — pins the §3.3 exclusion rule, alongside AeroTests.cpp:235; `GeometryProfileTests` is the natural home); the placement gate invalidates on attach/detach/setLink but not on `setPartMass` (PartsModelTests / DiagnosticsGateTests territory); existing cd == 0 pins untouched-green.
Exit: all suites green; trajectories bit-identical.

**Phase 3 — the calculator** (still no production consumer).
Files: Aero.h (FlowConditions, DragBreakdown, declarations), Aero.cpp (definitions + named constants). New `core/model/tests/DragBuildupTests.cpp` + CMake registration (Aero is a model-layer TU now).
Tests: every §9.1 formula oracle; the M ∈ [0, 3] continuity/finiteness sweep; guard cases (Re ≤ 0, mach 0, occlusion ≥ base, `hasDragGeometry` false forms).
Exit: the new `DragBuildupTests` green (model_tests); nothing else moves.

**Phase 4 — consume it** (the behavioral flip: never-configured rockets *and* loaded designs go auto).
Files: RocketModel.h/.cpp — cd override flag on the setter, `clearDragCoefficientOverride`, the `installDesign` reset, the §5.1 `getForces` block, `computedDragBreakdown` (the geometry memoization already landed in Phase 2's PartNode gate — RocketModel keeps no aero cache); **DesignSerializer.cpp** — the loader's `clearDragCoefficientOverride()` after its unconditional scalar restore (§7, Phase-4 step).
Tests: override-wins (scalar path bit-identical); `installDesign` clears the cd flag (companion to the area-override reset at RocketModelTests.cpp:27-37); **loaddesign clears it too** (loader companion); placeholder fallback; vacuum auto flight — no NaN, bit-identical to phase 3 via the early-out; 3FNC `computedDragBreakdown(0.3, 0)` plausibility band (§9.2); direct-API transonic auto flight (H-class-equivalent thrust curve, US Standard 1976) terminates nominally with finite states under both integrators.
Impact audit: the loader clear is load-bearing, not optional — without it, `CliDesignCommands.BuildSaveReloadFlyEndToEnd` (tests/CliDesignCommandsTests.cpp:63, flight :88-106) flies auto before save but manual Cd = 1.0 after reload (the setter now raises the flag) under the default drag-significant Constant Atmosphere, and its apogee `EXPECT_EQ` (:106) fails. With the clear, both flights are auto and the equality holds. Matrix sweeps that need manual Cd all issue `setdrag` *after* `loaddesign` (DesignMatrixTests.cpp:544-549), and every other matrix flight is vacuum — verified unaffected.
Exit: full ctest incl. `-L heavy` green.

**Phase 5 — CLI surface** (gives tests and users the auto switch before the persistence flip).
Files: Repl.cpp/.h — `setdrag auto`, `setarea auto`, `cdinfo`, live `status`, staging-var deletion, help text.
Tests (cli_tests): `setdrag auto` → OK + status shows auto; `setdrag 0.75` still OK + status shows manual; `setarea auto`; `cdinfo` breakdown fields positive and summing on a built design; `cdinfo` ERR on the placeholder.
New (design_matrix_tests): **auto-Cd ladder sweep** — per motor class: `loaddesign`, `setatmosphere "US Standard 1976"`, `setdrag auto`, fly; assert nominal termination (no WARN via the flyApogee hook), apogee > 0, and apogee < the same design's vacuum apogee. Fast variant one motor per class; heavy variant (`QTROCKET_FULL_LADDER=1`) the complete 1/4A→M ladder — this is the computed path's Mach-1+ suite coverage.
Exit: full ctest incl. `-L heavy` green.

**Phase 6 — persistence: make a deliberate manual override survive the round trip.**
Files: DesignSerializer.cpp (version "0.3", `dragCoefficientOverridden` attr write, and the Phase-4 unconditional flag-clear becomes gated: cleared unless the attr reads `"true"`).
Tests: `SaveWritesVersion0_2` (DesignPersistenceTests.cpp:304, version assertion :314) → re-pin `"0.3"` and rename `SaveWritesVersion0_3`; cd-override round-trip (manual survives save/load); legacy-absent-attr loads-as-auto (mirrors `NonOverriddenReferenceAreaStaysUnoverriddenOnLoad`, DesignPersistenceTests.cpp:131); extend `ReferenceAreaOverrideAndDragRoundTrip` (:110) with flag assertions.
(Loaded fixtures already fly computed-by-default since Phase 4; this phase only adds persistence for a deliberate manual `setdrag`. Re-run `-L heavy` and watch the no-WARN hooks anyway.)
Exit: integration_tests + cli_tests + `-L heavy` green with unregenerated fixtures.

**Phase 7 — close-out.**
Record the P5 build-up as complete (TODO.md was removed in `aa90de0` — the final commit message / docs index carries it); stale-comment sweep (BodyTube.cpp:41-42 "eventual contribution", the pre-refactor `Part::getCompositeAero` reference in the `AeroComponent` datum note at Aero.h:21 → `PartNode::compositeAero`, any residual P2-era "nothing in production consumes this" notes in the aero seam, Aero.h `cd` field doc gains "fixed/flow-independent"); coverage run (`cmake --build --preset coverage-clang --target coverage`) to confirm the new families are exercised.
Exit: full ctest green; docs/TODO consistent with the tree.

---

## 9. Test matrix

### 9.1 Unit oracles (phase 3, `core/model/tests/DragBuildupTests.cpp`) — closed-form hand values

| Oracle | Expected |
|---|---|
| C_f,turb(1e6) = 1/(1.50·ln 1e6 − 5.6)² | 4.372e-3 |
| clamp boundary: C_f(Re = 1e4⁻) vs C_f(1e4⁺) | identical to machine precision (clamp ≡ correlation at 1e4 ≈ 1.4817e-2, §2.1) |
| C_f,rough(R_s/L = 1.2e-4) = 0.032·(1.2e-4)^0.2 | 5.26e-3 |
| max() crossover: C_f(1e6) with R_s/L = 1.2e-4 | roughness-limited (5.26e-3 > 4.37e-3) |
| Re ≤ 0 | friction family exactly 0 |
| compressibility blend endpoints at M = 0.9 / 1.1; value at M = 1 (smooth case) | 0.5·(0.9 + 0.9221)·C_f = 0.9110·C_f |
| baseDrag: M=0 / M=1⁻ / M=1⁺ / M=2 | 0.12 / 0.25 / 0.25 / 0.125 |
| rounded LE: M=0.9⁻ vs 0.9⁺; M=1⁻ vs 1⁺ | 0.9987 vs 1.0 (NEAR 2e-3); 0.8215 exact both sides |
| rounded TE | 0.5·baseDrag(M) exactly |
| square LE (helper, unselected family): M=0.5 / M=2 | 0.85·1.064063 = 0.904453 / 0.85·1.660922 = 1.411784 |
| square TE | finTrailingEdgeFraction(Square) = 1.0 ⇒ C_D•TE = baseDrag(M) exactly |
| occlusion ratio: `dragBreakdown` at occludedBaseArea = ½·baseArea vs 0 | base family exactly halved (burn-vs-coast lives here, not in `computedDragBreakdown` — §3.5) |
| nose (R=0.019, L_n=0.1 ⇒ sinφ=0.18666): M=0.5 / M=1 / M=2 | 0.8·S2 / S1 exactly (Hoerner datum) / 2.1·S2 + 0.5·S1/√3 |
| nose bridge continuity at M = 0.8, 1.0, 1.3 (to 1e-12); monotone non-decreasing on [0.8, 1.0] (holds by construction: aₙ ≥ 0, bₙ ≥ 1); on (1, 1.3] monotone with the sign of s₁ | for every realistic cone (sinφ < 0.1896 — incl. this oracle's 0.18666, D(1.3)/D(1) = 0.9939) D *decreases* past M = 1: the transonic peak, not a bug — do not assert non-decreasing there |
| trapezoid MAC (c_r=0.05, c_t=0.025) | 0.0388889 |
| f_B clamp at length ≤ 2·maxRadius | factor uses f_B = 1 |
| occlusion clamp: A_occl ≥ A_base | base family 0, never negative |
| `hasDragGeometry`: empty (default-constructed) profile / refArea 0 / length 0 / nose-less (noseSinA = 0) | false |
| whole-calculator sweep, M ∈ [0, 3] step 0.01, fixed Re + 3FNC geometry, **run under both `FinEdge` enumerators** | finite, > 0, \|ΔC_D\| between adjacent nodes < 0.05 |

### 9.2 Integration assertions (phases 4–6)

- **3FNC plausibility** (24 mm tube, L ≈ 0.5 m, conical nose, 3 fins ~2 mm): `computedDragBreakdown(0.3, 0)` total ∈ **[0.4, 0.85]**, all families ≥ 0, friction the largest subsonic family. (Hand estimate with rounded edges and 60 µm roughness at Re ≈ 3.5e6 — ρ·v·L/μ = 1.225·102.1·0.5/1.789e-5 at M = 0.3, sea level; the case is roughness-limited either way, Re_crit = 51·(1.2e-4)^(−1.039) ≈ 6.0e5 < Re, so C_f = 5.26e-3 — friction ≈ 0.56 (body 0.45 + fins 0.11), base ≈ 0.12, nose ≈ 0.02, fin edges ≈ 0.05 ⇒ ≈ 0.75; the derivation lives in the test comment so the band is re-derived, not defended, if constants change.)
- Mach trend: total(M=1.2) > total(M=0.3) at fixed Re. (Burn-vs-coast occlusion is a §9.1 unit oracle where `FlowConditions` is constructed directly — `computedDragBreakdown` is coast-only by convention, §3.5.)
- Auto-Cd apogee < vacuum apogee for the same design/motor; auto flights terminate nominally (no WARN) under RK4 and RKF45.
- **Auto-Cd ladder sweep** (phase 5, fast + heavy): the computed path's transonic/supersonic coverage — finite states and sane ordering across the full 1/4A→M ladder under US Standard 1976.
- Reload determinism: an auto-mode design saved and reloaded flies to the identical apogee (extends `BuildSaveReloadFlyEndToEnd`'s existing equality — the flag round-trips, geometry is identical).

### 9.3 Every existing test affected, with exact remediation

| Test | Impact | Remediation | Phase |
|---|---|---|---|
| `DesignPersistenceTests` `SaveWritesVersion0_2` (tests/DesignPersistenceTests.cpp:304, assertion :314) | **breaks** — writer stamps "0.3" | re-pin to `"0.3"`, rename `SaveWritesVersion0_3` | 6 |
| `ReferenceAreaOverrideAndDragRoundTrip` (:110) | passes as-is (setter → flag → round-trip holds) | extend with `isDragCoefficientOverridden()` assertions | 6 |
| `NonOverriddenReferenceAreaStaysUnoverriddenOnLoad` (:131) | passes | add the cd-flag companion test | 6 |
| `PhysicsIntegrationTests` `TerminalVelocityForceBalance` (:251), `DragReducesApogeeVersusVacuum` (:232) | none — SetUp (:68-69) sets Cd + area manually ⇒ override wins ⇒ scalar path bit-identical, speed-independent, getter-exact (the 1e-6 balance holds) | none | — |
| `DesignMatrixTests` all sweeps (`setdrag 0` at :126/:545/:589; `setdrag 0.75` at :549) | none — `setdrag` grammar and `OK` reply preserved; explicit setdrag = manual; FlightMatrix flies vacuum | none; **add** the auto-Cd sweep | 5 |
| `BodyTubeTests.cpp:118`, `NoseConeTests.cpp:142`, `FinSetTests.cpp:217` (cd == 0 pins) | none — `AeroComponent.cd` stays 0; drag rides the new fields | augment with new-field oracles | 2 |
| `AeroTests` `FixedAeroPart` (:28-39, getAero :33) | none — `getAero(double)` signature unchanged | none | — |
| `AeroTests` `CompositeCdIsAdditive` (:112) | none — positional 3-field init still compiles append-only, but the fold at PartsModel.cpp:238 must become copy-then-modify (§3.3) before the new fields survive it | add new-field additivity assertions | 2 |
| `AeroTests` `ManualReferenceAreaOverrideWins` (:220) | none — the derived-area-0 pin (:226) now backs the fallback rule | none | — |
| `AeroTests` `MotorIsMassButNotAeroOrReferenceContributor` (:235) | none — Motor contributes zero geometry | add zero-geometry-field assertions (pins the intent) | 2 |
| `RocketModelTests` `InstallDesignReplacesTreeAndResetsReferenceAreaOverride` (:27-37) | none | add the cd-flag-reset companion | 4 |
| `CliDesignCommandsTests` `BuildSaveReloadFlyEndToEnd` (:63, flight :88-106, apogee `EXPECT_EQ` :106) | **would break at Phase 4 without the loader flag-clear** — auto pre-save vs manual Cd=1.0 post-reload under the default Constant Atmosphere ⇒ apogee `EXPECT_EQ` fails; green with the clear (both flights auto) | loader clear lands inside Phase 4 (§7, §8); extend with the reload-determinism check under auto | 4/6 |
| `PlacementInvarianceTests` (cp-only snapshots), `PropagatorTests`, `PokeThroughRegressionTests`, `DesignRebuildTests`, `MotorDatabasePersistenceTests`, all `core/sim/tests/` | none (verified: zero drag/Cd references) | none | — |
| CLI `status` output | display changes | no test asserts on it (verified) | 5 |

---

## 10. Edge-case ledger

| Case | Handling |
|---|---|
| v → 0 (Re → 0) | speed ≤ 0 early-out skips the drag term entirely; in the calculator, Re ≤ 1e4 clamps C_f,turb (the 1/(1.50·ln Re − 5.6)² correlation is singular at Re ≈ 42 and negative below — never evaluated there), Re ≤ 0 zeroes friction. Force is ∝ \|v\|·v regardless — zero-safe. |
| Vacuum | ρ = 0 → early-out before any flow math (dominant test suites bit-identical). Belt-and-braces: a = 0 → mach 0; μ = 0 → Re 0. No NaN, no throw. |
| Below-ground trial states | every atmosphere query reuses the existing clamp (RocketModel.cpp:81) — USStandardAtmosphere's sub-zero `std::out_of_range` stays unreachable. |
| No design (fresh session — null root) | display paths guard `hasDesign()` before any root access (`computedDragBreakdown` → nullopt, `cdinfo` → ERR); flights are unreachable without a design. `effectiveReferenceArea` falls back to the stored member (derived 0 via the existing `hasDesign()` guard, RocketModel.cpp:46). |
| Missing drag geometry (cannonball HollowSphere design) | `hasDragGeometry` false → stored scalar fallback; `effectiveReferenceArea` falls back to the stored member (derived = 0). Bit-identical to today. |
| `setarea 0` (legal today) | stays legal manual behavior: area 0 ⇒ drag 0; the fold at area 0 yields `refArea = 0` ⇒ computed path gated off — moot, force is zero. |
| Burnout base step | occlusion tied to `getThrust(t) > 0` — the same signal the thrust term trusts. Cd steps up at burnout exactly when thrust steps to 0; RKF45 rejects/shrinks across it, RK4 marches. |
| Motor diameter 0 / no motor | A_occl = 0 → full base drag — conservative, safe. |
| Fat motor (A_occl > A_base) | `max(A_base − A_occl, 0)` — never negative. |
| Free-standing fins (bodyRadius = 0) | degenerate branch keeps friction/edge geometry, drops only the singular Barrowman CNalpha term. |
| refArea ≤ 0 passed to `getAero` | uniform `return {}` guard (FinSet already has it; cone/tube get it) — nothing to normalize against. |
| Multiple nose cones / fin sets | all fields additive by construction; the nose bridge runs on the aggregate S1/S2 (documented approximation, §2.2). |
| Supersonic (M-class ladder) | explicit M ≥ 1 branches throughout; √(M²−1) only at M ≥ 1.3; C⁰ across every boundary (§2.6); finite to M ≈ 4 and beyond (decaying), validity above ~4 documented as unclaimed. |
| Descent / tumble | forward-flight C_D while falling ballistically — no worse than today's constant; resolves with recovery modeling (out of scope). |
| Structural edit mid-session | the geometry-static profile caches behind `PartNode::placementDirty_`; every mutation routes through a `PartsModel` verb (the single mutation authority — leaf setters are private), so it dirties the gate up to the root and a stale read is unrepresentable. `installRoot` dirties a freshly installed root; no pointer-keyed cache ⇒ no ABA (§3.3, §5.2). |
| Zero-length / aft-protruding Motor | excluded outright from extent and base-plane resolution (§3.3) — a protruding motor can neither zero the base disc nor inflate the exposed length. Pinned by the Phase 2 aft-protruding-motor oracle. |
| Nose-less airframe (bare BodyTube root — buildable today via `newdesign BodyTube`) | `hasDragGeometry` requires `noseSinA > 0`: no blunt-fore-face family exists, so the build-up would read ~0.2 where a flat-faced cylinder is ~0.8. Falls back to the manual/stored scalar (default 1.0 — the right order of magnitude). Lifting the gate = the deferred fore-face term (§11). |

---

## 11. Risks & explicitly-deferred items

1. **Transonic fidelity.** The anchored bridge (M=1 = Hoerner sinφ datum, slope-matched) replaces OpenRocket's stored experimental transonic curves. Cones have closed forms at both ends and a datum in the middle, so the error is bounded; matters only in the ladder's upper classes. Revisit if apogee validation against OpenRocket shows > 10% transonic disagreement.
2. **Fin edge family — the biggest modeling lever.** Rounded LE/TE is assumed without an attribute; square-edged fins would add ~0.25–0.6 to C_D at realistic frontal ratios. Both laws are implemented behind the one named constant (§2.4), so the default flips in one line. **Deferred:** a `FinSet` cross-section attribute (`PartParams` + `DesignSerializer` + CLI parser + `.qrd` field), at which point the 3FNC test band is re-derived per its in-test derivation comment.
3. **Single global roughness (60 µm).** **Deferred:** per-part finish attribute, same `PartParams`/serializer path as #2.
4. **`AeroComponent` field accretion.** The seven geometry fields encode the current formulas' sufficient statistics; the encoding works while every formula is linear in additive per-part geometry with global Mach/Re coefficients. Named redesign triggers: per-part Reynolds/roughness, stored experimental nose curves, or a shape-family part — any of these moves the seam to a flow-aware `getAero(refArea, flow)` signature (the runner-up design; compiler-enforced migration, all overrides use `override`). Until then this is the smallest honest diff.
5. **FinSet chordwise mirrored-sign defect** (the CP/CM chordwise terms at FinSet.cpp:54-57, :84-88 — confirmed still present). The architecture review gates *CP consumption* on it; the drag build-up consumes no x_cp — every formula here is station-independent except base-plane resolution, which uses placement poses. Cd lands independently; the fix stays queued for the static-margin feature.
6. **Motor casing diameter as nozzle proxy** slightly over-occludes the base during burn (less base drag). `data.diameter` is the only diameter available; direction is conservative for apogee prediction.
7. **No boattail/transition part** ⇒ diameter-step pressure/base drag between mismatched tubes is unmodeled — only the aft base is. Deliberately *not* inferred from `radiusOuterAt` discontinuities (epsilon-fiddly, no consumer case in the current part set). Becomes real work when a Transition part lands.
8. **Legacy pre-0.3 manual-Cd files load as auto** (absent flag ⇒ auto). Only in-tree consumers are the fixtures, where auto is the desired outcome; a user's deliberate manual Cd is one `setdrag` away. Accepted, documented in the phase-6 commit message.
9. **C⁰-not-C¹ boundaries** at M ∈ {0.8, 0.9, 1.0, 1.1, 1.3} may briefly shrink adaptive RKF45 steps. **M = 1.0 is the dominant kink** — base-drag slope flips +0.26 → −0.25 per unit M, rounded-LE −1.785 → +0.566, nose bridge +5aₙ → s₁ ≤ 0 — so heavy-ladder runtime monitoring should watch the Mach-1 crossing, not the blend edges. Cubic blends are a drop-in if it thrashes.
10. **Descent-phase C_D is wrong** (forward-flight model while falling) — unchanged from today's constant-Cd wrongness; resolves with recovery modeling (separate roadmap item).
11. **Cd observability in flight output.** CSV/state columns gain nothing here; `cdinfo` covers static queries. Deferred: max-Mach / Cd-at-max-q in the `launch` summary — decide when users ask.
12. **`status`/`cdinfo` representative condition** (M=0.3, h=0) is a convention, not max-q. Cosmetic; deferred.
13. **`Propagatable::aeroData`** (unread member, Propagatable.h:61 — now a `model::Aero` = `AeroProfile`) stays untouched — unrelated cleanup, tracked separately.
14. **No blunt-fore-face family.** Nose-less airframes are gated to the fallback scalar by `hasDragGeometry` (§3.1) rather than modeled. The upgrade is the techdoc stagnation blunt-cylinder term applied to the exposed fore annulus (resolve the fore-plane radius symmetrically to the §3.3 base plane, subtract the cone-covered area); landing it lifts the `noseSinA > 0` gate.
