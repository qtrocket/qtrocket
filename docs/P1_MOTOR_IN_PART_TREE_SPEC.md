# P1 — Put the Motor in the Part Tree (`MotorPart`): Build-Ready Spec

**Status:** ready to implement · **TODO item:** P1 / finding F5 · **Date:** 2026-06-16
**Scope:** make the motor a first-class `Part` so composite mass and **CG(t)** become honest, closing the old-P2 read/write reconciliation. Deliberately *no* 6-DOF work.

This spec is the merge of the two strongest independent plans: plan #1's *time-aware composite-mass path + Part-first sequencing* and plan #2's *single `computeCompositeMassAndCm(t)` helper + "CG(t) logic belongs in `Part`, not `RocketModel`" framing*.

---

## 1. Objective & scope

Today the motor lives **beside** the part tree as a by-value `MotorModel mm` on `RocketModel`, and `RocketModel::getMass(t)` adds it on by hand:

```cpp
// model/RocketModel.cpp (current)
double RocketModel::getMass(double t)
{
    double mass = mm.getMass(t);
    mass += topPart->getCompositeMass(t);   // structural composite (no motor in the tree)
    return mass;
}
```

We introduce `model::MotorPart : public Part` that wraps a `MotorModel` and overrides the `Part::getMass(double t)` hook ([Part.h:99](../model/Part.h#L99)) — the hook that exists for exactly this. The motor attaches as a child of `topPart`, so the part tree becomes the single source of truth for mass and CG.

**In scope**
- New `model::MotorPart` leaf part wrapping a `MotorModel` by value.
- Make `Part`'s composite **mass and CM** time-aware (the load-bearing fix; see §2).
- Rewire `RocketModel` so the motor lives in the tree; `getMass(t)` collapses to `topPart->getCompositeMass(t)` (no double-count); thrust/ignition route through the `MotorPart`.
- Resolve the `setMass` write-side (closes old-P2).
- Tests: new `MotorPartTests.cpp` + a no-double-count regression pin in the integration suite.

**Out of scope (deferred, with breadcrumbs left in the code)**
- Time-varying **inertia tensor** during burn → **P6** (3-DOF reads no inertia).
- A real, GUI-driven motor axial **offset** → **P2/P5** (we use a zero placeholder; numerically identical to today).
- Multi-motor / clusters, staging, a `Part` detach/remove API.

---

## 2. The one constraint everything hinges on

The existing composite cache is **time-invariant by construction**, and a naïve `MotorPart` would be silently ignored:

- [`getCompositeMass(double t)`](../model/Part.h#L108) marks `t` `[[maybe_unused]]` and returns the cached scalar.
- [`recomputeInertiaTensor()`](../model/Part.cpp#L135) builds that scalar from the **member `mass`** (`compositeMass = mass; … += child->compositeMass`), **never** from `getMass(t)`, and only runs when the *dirty flag* is set. Time advancing does **not** set the dirty flag.

So if `MotorPart` only overrides `getMass(t)` and we route `RocketModel::getMass` through `getCompositeMass(t)`, the rocket's mass would **freeze at pre-ignition total weight** — propellant burn-off would vanish and apogee would be wrong. (This is the exact trap two of the five candidate plans fell into.)

**Fix:** give `Part` a *live, non-caching* time-aware path for mass and CM, computed by walking the tree and summing `getMass(t)`, while leaving the cached static path in place **solely** for the inertia tensor (`getCompositeI()`), which 3-DOF never reads.

---

## 3. Detailed changes

### 3.A `model/Part.h` / `model/Part.cpp` — time-aware composite mass & CM

Make `getCompositeMass`/`getCompositeCm` **live** (non-cached) and symmetric on `t`, backed by one shared walk. Leave `getCompositeI()`, `recomputeInertiaTensor()`, the dirty flag, and the cached `compositeMass`/`compositeCm`/`compositeInertiaTensor` members **unchanged** (the members become internal scratch for the tensor recompute only).

**`Part.h`** — add `#include <utility>`; replace the two composite-mass/CM accessors and add a private helper:

```cpp
/**
 * @brief Composite mass of this part plus all descendants at simulation time @p t (kg).
 *
 * Computed LIVE from each node's getMass(t) (so a time-varying override such as MotorPart is
 * reflected) -- NOT from the cached static composite, which is maintained only for the inertia
 * tensor (getCompositeI()). Cheap: the tree is small and this is a read-only walk; no caching, so
 * it is safe to call every ODE step.
 */
virtual double getCompositeMass(double t)
{
   return computeCompositeMassAndCm(t).first;
}

/**
 * @brief Composite center of mass at time @p t, relative to this part's own CM (zero for a leaf).
 *        Time-aware companion to getCompositeMass(t): as a child's mass changes (a burning motor),
 *        this CG(t) shifts forward. This is the P5 static-margin / P6 inertia prerequisite.
 */
virtual Vector3 getCompositeCm(double t)
{
   return computeCompositeMassAndCm(t).second;
}
```

…and in the `private:` section:

```cpp
/// @brief Live (non-cached) composite mass and CM at time @p t: walks this sub-tree summing
///        getMass(t). Mirrors recomputeInertiaTensor()'s pass 1 but evaluated at t.
///        @return {composite mass (kg), composite CM relative to this part's own CM}.
std::pair<double, Vector3> computeCompositeMassAndCm(double t);
```

> Note the signature change: `getCompositeCm()` → `getCompositeCm(double t)` (it was previously no-arg). This makes it symmetric with `getCompositeMass(double t)` and avoids a confusing dual-CM API. The four existing call sites are updated in §3.E.

**`Part.cpp`** — implement the helper (mirrors [pass 1 of `recomputeInertiaTensor`](../model/Part.cpp#L142-L159) but uses `getMass(t)`):

```cpp
std::pair<double, Vector3> Part::computeCompositeMassAndCm(double t)
{
   double m = getMass(t);              // this node's own (possibly time-varying) mass
   Vector3 weighted = Vector3::Zero(); // sum of subtreeMass * (pos + subtree CM), about own CM
   for(auto& [child, pos] : childParts)
   {
      const auto [childMass, childCm] = child->computeCompositeMassAndCm(t);
      m += childMass;
      weighted += childMass * (pos + childCm);
   }
   Vector3 cm = Vector3::Zero();
   if(m > 0.0)                         // guard: a massless subtree has no meaningful CM
   {
      cm = weighted / m;
   }
   return {m, cm};
}
```

**Optional cleanup (same change):** delete the now-redundant dead `Part::getChildMasses(double)` ([Part.h:209-210](../model/Part.h#L209), [Part.cpp:70-79](../model/Part.cpp#L70)) — it has no callers and its role is subsumed by the helper.

Update the class-level doc comment near the cached members to note they now back only `getCompositeI()`.

### 3.B `model/parts/MotorPart.h` (new)

```cpp
#ifndef MODEL_PARTS_MOTORPART_H
#define MODEL_PARTS_MOTORPART_H

/// \cond
// C++ headers
#include <memory>
#include <string>
/// \endcond

// qtrocket headers
#include "model/Part.h"
#include "model/MotorModel.h"
#include "utils/math/MathTypes.h"

namespace model
{

/**
 * @brief A leaf Part that wraps a MotorModel so the motor's time-varying mass participates in the
 *        composite mass and center of mass of the rocket's part tree.
 *
 * Owns one MotorModel by value and overrides Part::getMass(double t) to return the motor's
 * burn-time-dependent mass. Attached as a child of the airframe (see RocketModel), it makes
 * composite mass(t) and CG(t) honest.
 *
 * Inertia: the per-unit-mass geometric tensor is a solid cylinder (InertiaTensors::Tube, ri = 0)
 * from the motor's diameter/length; the time-varying full inertia TENSOR is deferred to P6 (3-DOF
 * reads no inertia). See Part.h on the per-unit-mass vs composite tensor convention.
 */
class MotorPart : public Part
{
public:
   /// @brief Construct a MotorPart wrapping a copy of @p motor.
   MotorPart(const std::string& name, const MotorModel& motor);

   ~MotorPart() override = default;

   /// @brief This part's own mass at time @p t: the motor's burn-time-varying mass (kg).
   ///        Pre-ignition = loaded total weight; during burn falls to the casing (empty) mass;
   ///        after burnout stays at the casing mass. @see MotorModel::getMass
   double getMass(double t) override { return mm.getMass(t); }

   /// @brief Read access to the wrapped motor (e.g. to plot its thrust curve).
   const MotorModel& getMotorModel() const { return mm; }
   /// @brief Mutable access (e.g. startMotor()/getThrust() during a flight).
   MotorModel& getMotorModel() { return mm; }

   /// @brief Replace the wrapped motor in place (when the user re-selects a motor). Re-seeds the
   ///        static mass/inertia and flags the tree for composite recompute. Part has no detach
   ///        API, so replacement mutates this node rather than removing/re-adding it.
   void setMotorModel(const MotorModel& motor);

protected:
   /// @brief Protected defaulted copy ctor + cloneShallow() implement clone() for this type, exactly
   ///        as HollowSphere does. MotorModel is copyable, so the wrapped motor deep-copies by value.
   MotorPart(const MotorPart&) = default;

   std::shared_ptr<Part> cloneShallow() const override
   {
      return std::shared_ptr<Part>(new MotorPart(*this));
   }

private:
   /// @brief Per-unit-mass geometric inertia tensor (m^2): a solid cylinder from the motor's
   ///        diameter/length (mm -> m), or Zero (point mass) if either is non-positive. Static so it
   ///        can be evaluated in the Part base-class initializer.
   static Matrix3 motorTensor(const MotorModel& motor);

   MotorModel mm; ///< The wrapped motor, owned by value.
};

} // namespace model

#endif // MODEL_PARTS_MOTORPART_H
```

### 3.C `model/parts/MotorPart.cpp` (new)

```cpp
#include "model/parts/MotorPart.h"

// qtrocket headers
#include "model/InertiaTensors.h"

namespace model
{

MotorPart::MotorPart(const std::string& name, const MotorModel& motor)
   // Part stores the inertia tensor per-unit-mass and applies the mass internally. Seed the base
   // mass with the motor's pre-ignition total weight; the time-varying truth comes via getMass(t).
   : Part(name, motorTensor(motor), motor.getMass(0.0), Vector3::Zero()),
     mm(motor)
{ }

void MotorPart::setMotorModel(const MotorModel& motor)
{
   mm = motor;
   // setI() and setMass() each mark this part (and its ancestors) for composite recompute, so the
   // airframe's cached static inertia tensor is rebuilt to reflect the new motor on the next read.
   setI(motorTensor(mm));
   setMass(mm.getMass(0.0));
}

Matrix3 MotorPart::motorTensor(const MotorModel& motor)
{
   const double diameter_m = motor.data.diameter / 1000.0; // RSE / DB store mm
   const double length_m   = motor.data.length   / 1000.0;
   if(diameter_m <= 0.0 || length_m <= 0.0)
   {
      return Matrix3::Zero(); // unknown geometry -> point mass (tensor unused in 3-DOF anyway)
   }
   return InertiaTensors::Tube(0.0, diameter_m / 2.0, length_m); // solid cylinder, per unit mass
}

} // namespace model
```

### 3.D `model/parts/Parts.h` + `model/CMakeLists.txt` — wiring

`Parts.h` — register under the umbrella header (one-line add):

```cpp
#include "model/parts/HollowSphere.h"
#include "model/parts/MotorPart.h"     // <-- add
```

`CMakeLists.txt` — add the two sources to the `model` library (next to `parts/HollowSphere.*`):

```cmake
   parts/HollowSphere.cpp
   parts/HollowSphere.h
   parts/MotorPart.cpp     # <-- add
   parts/MotorPart.h       # <-- add
   parts/Parts.h
```

### 3.E `model/RocketModel.h` / `model/RocketModel.cpp` — rewire

**`RocketModel.h`:**
- Forward-declare the part: `namespace model { class MotorPart; }` (the raw pointer needs no full type).
- **Remove** `model::MotorModel mm;` and `bool motorSet{false};`.
- **Add** members:
  ```cpp
  /// Borrowed (non-owning) handle to the motor node; the owning shared_ptr lives in topPart's
  /// childParts. Valid for the RocketModel's lifetime (the node is never detached). nullptr = no
  /// motor set. RocketModel is only ever held via shared_ptr (QtRocket::getRocket()), never
  /// value-copied, so this never dangles; if deep-copy is ever needed, re-resolve via findById.
  MotorPart* motorPart{nullptr};

  /// Body-frame offset of the motor CM relative to the airframe (topPart) CM. Zero today
  /// (numerically identical to the pre-tree behaviour); GUI-driven geometry lands in P2/P5.
  Vector3 motorOffset{Vector3::Zero()};
  ```
- `isMotorSet()` becomes: `bool isMotorSet() const { return motorPart != nullptr; }`.
- `getMotorModel()` — move the body to the `.cpp` (it now needs the complete `MotorPart` type): declare `MotorModel getMotorModel();`. Keep `#include "model/MotorModel.h"` (return-by-value needs the complete type, and `AnalysisWindow` relies on the by-value return).
- Update the `setMass` doc comment per §4.

**`RocketModel.cpp`** (already includes `model/parts/Parts.h`, which now pulls `MotorPart.h`):

```cpp
double RocketModel::getMass(double t)
{
    // The motor is now a child Part, so the composite already includes its time-varying mass.
    return topPart->getCompositeMass(t);
}

void RocketModel::setMotorModel(const MotorModel& motor)
{
   if(motorPart == nullptr)
   {
      auto mp = std::make_shared<MotorPart>("Motor", motor);
      motorPart = mp.get();                       // borrow before ownership moves into the tree
      topPart->addChildPart(std::move(mp), motorOffset);
   }
   else
   {
      motorPart->setMotorModel(motor);            // in-place swap (Part has no detach API)
   }
}

MotorModel RocketModel::getMotorModel()
{
   return motorPart ? motorPart->getMotorModel() : MotorModel{};
}

double RocketModel::getThrust(double t)
{
   return motorPart ? motorPart->getMotorModel().getThrust(t) : 0.0;
}

void RocketModel::launch()
{
   setCurrentState(initialState);
   if(motorPart) { motorPart->getMotorModel().startMotor(0.0); }
}
```

In `getForces`, the thrust line changes from `mm.getThrust(t)` to:

```cpp
Vector3 forces{0.0, 0.0, motorPart ? motorPart->getMotorModel().getThrust(t) : 0.0};
```

`getCompositeInertiaTensor` is **unchanged** (still `topPart->getCompositeI()`).

### 3.F Existing test call-site updates (signature change for `getCompositeCm`)

In [`model/tests/PartTests.cpp`](../model/tests/PartTests.cpp), update the four no-arg calls to pass `0.0` (values unchanged — these parts are time-invariant, so `getMass(0.0) == mass`):

| Line | Before | After |
|---|---|---|
| 189 | `parent->getCompositeCm();` | `parent->getCompositeCm(0.0);` |
| 241 | `root->getCompositeCm()(0)` | `root->getCompositeCm(0.0)(0)` |
| 324 | `assembly->getCompositeCm();` | `assembly->getCompositeCm(0.0);` |
| 352 | `assembly->getCompositeCm();` | `assembly->getCompositeCm(0.0);` |

(`getCompositeMass(0.0)` calls already pass `t` and are unaffected.)

---

## 4. `setMass` write-side resolution (closes old-P2)

**Decision:** `setMass` keeps its current single line and signature — it sets the **structural (dry) airframe mass = the top part's own mass** — but the meaning is now *honest* because the motor mass has left `topPart` and lives in the `MotorPart` child:

```cpp
void setMass(double m) { if(m > 0.0) topPart->setMass(m); }   // unchanged body
```

- **Read side** (already composite): `getMass(t) = topPart->getCompositeMass(t)` = airframe-own-mass + motor(t) + any future structural children.
- **Write side** (this item): `setMass` writes only the airframe-own term and never the motor. No double-count, no distribution, no deprecation.
- **Backward compatibility:** the GUI/CLI "mass" field ([CannonballTab.cpp:115](../gui/CannonballTab.cpp#L115), [Repl.cpp:364](../cli/Repl.cpp#L364)) and the integration test's `setMass(0.5)  // kg structural (dry)` keep their exact meaning and call sites — zero signature churn.

Update the [RocketModel.h:119-127](../model/RocketModel.h#L119) doc comment to state the clean separation and drop the "eventual composite-aware GUI mass story" caveat (resolved). A future *per-part* mass editor would call `Part::setMass` on a `findById`-located node — already supported.

---

## 5. Inertia handling — computed now vs deferred

Convention preserved exactly (per [Part.h:29-35](../model/Part.h#L29)): leaf parts store a **per-unit-mass** geometric tensor (m²); composites are the **full mass-weighted** tensor (kg·m²) about the composite CM via parallel-axis ([Part.cpp:135-177](../model/Part.cpp#L135)).

**Computed now**
- Time-varying composite **mass** `getCompositeMass(t)` and **CG** `getCompositeCm(t)` — the honest, per-step quantities (§3.A). CG(t) shifts forward as propellant burns.
- A sensible per-unit-mass solid-cylinder tensor for the motor (`motorTensor`), so the cached static composite tensor is reasonable for the loaded mass.

**Deferred to P6 (documented in code)**
- The full composite inertia **tensor** stays on the existing dirty-flag-cached path and is **frozen at the construction (pre-ignition) mass** — it does *not* track CG(t) during the burn. This is safe because 3-DOF reads no inertia: [`getTorques` returns zero](../model/RocketModel.cpp#L77) and the propagator only divides by `getMass(t)`. Threading `t` through the tensor now would be over-build toward 6-DOF.

> This is the one deliberate asymmetry: two of the three composite accessors honor `t`, the tensor does not. It is the smallest change that makes CG(t) honest. A `// NOTE (P6): time-aware inertia tensor` breadcrumb goes on `getCompositeI()` / `getCompositeInertiaTensor`.

---

## 6. Backward-compatibility / numerical-invariance argument

With `motorOffset = 0`, the new `getMass(t)` is **bit-for-bit** the old one:

```
OLD: getMass(t) = mm.getMass(t)            + topPart->getCompositeMass(t)   // body only, no motor child
NEW: getMass(t) = topPart->getCompositeMass(t)
               = topPart.ownMass + motorPart.getMass(t)                     // mass-sum is offset-independent
               = topPart.ownMass + mm.getMass(t)
```

Since `topPart.ownMass` is what `setMass` always wrote, `NEW == OLD`. Therefore **the existing apogee / flight-time / terminal-velocity integration tests must pass unchanged** — that's the primary regression guard. The no-motor default case also matches (default `MotorModel` reports mass 0; with no `MotorPart`, the composite is body-only).

---

## 7. Test plan

### New: `model/tests/MotorPartTests.cpp` (add to [`model/tests/CMakeLists.txt`](../model/tests/CMakeLists.txt) `model_tests` sources)

No data dir needed — build a synthetic motor in-test (masses in kg; `addThrustCurve` **before** `setMetaData`, because `computeMassCurve` reads the curve):

```cpp
#include <gtest/gtest.h>
#include <memory>
#include <utility>
#include <vector>

#include "model/parts/Parts.h"   // pulls MotorPart.h + HollowSphere.h
#include "model/MotorModel.h"
#include "model/ThrustCurve.h"

namespace
{
// Synthetic single-use motor, flat thrust over [0, burnTime]. emptyMass = totalWeight - propWeight.
model::MotorModel makeTestMotor(double totalWeight, double propWeight,
                                double burnTime, double totalImpulse)
{
   std::vector<std::pair<double, double>> samples{ {0.0, totalImpulse / burnTime},
                                                   {burnTime, totalImpulse / burnTime} };
   ThrustCurve tc(samples);

   model::MotorModel m;
   m.addThrustCurve(tc);                 // must precede setMetaData

   model::MotorModel::MetaData md;
   md.totalWeight = totalWeight; md.propWeight = propWeight;
   md.burnTime = burnTime;       md.totalImpulse = totalImpulse;
   md.diameter = 24.0; md.length = 70.0; // mm, for the geometric tensor
   m.setMetaData(md);                    // triggers computeMassCurve()
   return m;
}
} // namespace
```

Cases:

1. **`GetMassFollowsMotorModelBurn`** — `MotorPart` from `makeTestMotor(0.100, 0.060, 2.0, 80.0)`; `getMass(0)==0.100` pre-ignition; after `getMotorModel().startMotor(0.0)`: `getMass(1.0)` strictly between `0.040` and `0.100`; `getMass(2.0)≈0.040`; `getMass(5.0)≈0.040`.
2. **`CompositeMassEqualsAirframePlusMotorAtTime`** — `HollowSphere` body + `MotorPart` child; `body->getCompositeMass(t) == bodyMass + motor.getMass(t)` at t=0 and t=2 (post-burnout). *This is the core fix — would fail if §3.A were skipped.*
3. **`CompositeCmShiftsForwardAsMotorBurns`** — motor attached at `Vector3{0,0,-0.2}` (aft); `getCompositeCm(0)(2) < 0` (CG pulled aft), `getCompositeCm(2)(2) > getCompositeCm(0)(2)` (CG moves forward as propellant burns). *CG(t)-honesty / P5 prerequisite.*
4. **`CloneIsDeepIndependentAndTypePreserving`** — clone a `MotorPart` (ignited); `dynamic_cast<MotorPart*>` succeeds, fresh `getId()`; mutating the original via `setMotorModel(heavier)` leaves the clone's `getMass(1.0)` unchanged. Mirrors [`CloneIsADeepIndependentTypePreservingCopy`](../model/tests/PartTests.cpp#L249).
5. **`ReplaceMotorChangesCompositeMass`** — `setMotorModel` with a heavier motor changes `getCompositeMass(0)` accordingly (live path) and invalidates the static cache (`getCompositeI()` reflects it). Mirrors the dirty-flag intent of [`SetMassAndSetIInvalidateCompositeCache`](../model/tests/PartTests.cpp#L271).

### Extend: `tests/PhysicsIntegrationTests.cpp`

6. **`RocketMassEqualsDryPlusMotorNoDoubleCount`** (new) — reuse the fixture (`setMass(0.5)` + G80T). Before `launch()`: `getMass(0) ≈ 0.5 + motor.getMass(0)`. After `launch()`: `getMass(largeT) ≈ 0.5 + motorEmptyMass`. **The explicit double-count pin.**
7. **Regression (no code change):** the existing apogee / flight-time / `TerminalVelocityForceBalance` cases must pass unchanged (see §6).

### Run

```bash
cmake --build --preset debug-clang
ctest --test-dir build -R 'qtrocket_*'          # all suites
./build/model/tests/model_tests --gtest_filter='MotorPartTest.*'
```

---

## 8. Implementation order (Part-first; prove the mechanism before building on it)

1. **`Part` time-aware path (§3.A) + tests first.** Add `computeCompositeMassAndCm(t)`, repoint `getCompositeMass`/`getCompositeCm`, update the 4 PartTests call sites (§3.F). Add a tiny mass-varying `Part` subclass *inside the test* and assert `getCompositeMass(t)`/`getCompositeCm(t)` track it. Build `model_tests` green — this isolates the riskiest edit.
2. **`MotorPart` class (§3.B/§3.C)** + register (§3.D).
3. **`RocketModel` rewire (§3.E):** `setMotorModel` create-or-replace; `getMass → topPart->getCompositeMass(t)`; route thrust/ignition/`getMotorModel` through `motorPart`; drop `mm`/`motorSet`.
4. **`setMass` doc update (§4)** — confirm no signature change, so GUI/CLI compile untouched.
5. **`MotorPartTests.cpp` (§7)** + the integration double-count pin.
6. **Build `-Werror`, run all four ctest suites**, then a CLI flight sanity check.

---

## 9. Risks & mitigations

| Risk | Mitigation |
|---|---|
| **Double-count** if the old `mm.getMass(t)` add isn't removed from `getMass` | Highest-risk edit; pinned by integration test #6 and the §6 invariance argument. |
| **Composite stays time-static** if §3.A is skipped → frozen propellant mass | §3.A is step 1 and proven by its own test before `MotorPart` exists; integration test #6 catches it end-to-end. |
| **Dirty-flag vs per-call time** | The live mass/CM path **does not cache** and does not touch `needsRecomputing`; the cached path is untouched and used only by `getCompositeI()`. |
| **Motor not set** (default rocket) | Every `motorPart` deref is null-guarded; `getMass` returns body-only; `getThrust` returns 0; `getMotorModel` returns a default `MotorModel` (matches today). |
| **No detach API** on `Part` | Motor replacement mutates the existing node in place (`MotorPart::setMotorModel`), never removes/re-adds. |
| **Raw `MotorPart*` dangling** | `RocketModel` is only held via `shared_ptr` (`getRocket()`), never value-copied; node lives in `topPart` for the rocket's life. Documented fallback: re-resolve via `findById`. |
| **Ignition timing** | `MotorModel::getMass` returns total weight until `startMotor`; `launch()` ignites `motorPart`'s motor at 0.0 exactly as before. |
| **Motor geometry units / missing data** | `data.diameter`/`length` are mm → `/1000`; non-positive → `Matrix3::Zero()` (tensor inert in 3-DOF). |
| **Mass units** | Verified kg via [RSEDatabaseLoader.cpp:82-83](../model/RSEDatabaseLoader.cpp#L82) (g→kg). The DB save/load path round-trips the same fields; the stale "grams" doc comments in `MotorModel::MetaData` should be corrected opportunistically (not required here). |

---

## 10. Follow-ups this unblocks

- **P5** static margin: `CG(t) = topPart->getCompositeCm(t)` is now available; add Barrowman CP and show CG/CP/margin live.
- **P6** time-varying inertia: make `getCompositeI()` honor `t` (or recompute per step) and feed `getTorques` from the CP–CG lever; `MotorPart`'s tensor is already in the tree.
- **P2** geometry: replace `motorOffset = 0` with a real axial position derived from part geometry; the seam (`motorOffset` member + `addChildPart` position) is already in place.
