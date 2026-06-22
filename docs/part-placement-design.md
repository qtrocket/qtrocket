# Part Placement — Station-Pair Backbone with Extent-Based Diagnostics: Design Spec

**Status:** design / ready to implement · **TODO item:** replace CM-to-CM child offsets · **Date:** 2026-06-21
**Scope:** replace QtRocket's fragile **CM-to-CM** child-offset link with an edge-defined, geometry-driven
placement model in which inter-part relationships are stated as physical intent ("this rim seats against that
rim", "this tube nests inside that bore"), a **single resolver** produces absolute placement consumed
identically by the simulator and the visualizer, and the center of mass becomes a purely **derived**
quantity. A path to 6-DOF is shaped in but not used. No force-path or integrator changes.

This spec is the synthesis of **five independent design sketches** (feature-mating, datum-pair, declarative
constraint-solver, local-frame scene graph, joint-as-entity graph) adjudicated by **three judges**
(correctness/robustness, simplicity/migration, extensibility/ergonomics), then hardened against an
adversarial critique. The judges converged on the **datum/station-pair** chassis, upgraded with the
extent-based overlap rigor of the constraint-solver sketch, the `Pose`/snap-operator mechanics of the
scene-graph sketch, and the direction-constrained nesting of the joint sketch.

Citations are pinned to the **current working tree** (branch `PartPlacement`, namespace `model::part`),
verified against the live source — not the older commit the original sketches referenced. The
working tree is entirely **pre-refactor**: the new types named below (`StationLink`, `SeatKind`,
`Station`, `Pose`, the resolver) do not yet exist, and the geometry virtuals (`stationAt`,
`radiusOuterAt/InnerAt`, `axialLength`, `isSolid`, a base `getLength()`) are *not* on `Part` today —
the only base geometry virtual is `getReferenceArea()` (`Part.h:176`). This document describes the
target and, where it matters for correctness, flags what must be added versus what already exists.

---

## 0. The problem this replaces

QtRocket stores each child as `std::vector<std::tuple<std::shared_ptr<Part>, Vector3>> childParts`
(`model/parts/Part.h:326`), where the `Vector3` is the child's center of mass **relative to the parent's
center of mass** (CM-to-CM). That offset is consumed in `Part::computeCompositeAt` (`Part.cpp:169`, the
two-pass mass/CM loop that shifts each child tensor to the composite CM via the parallel-axis theorem at
`Part.cpp:179-200`) and threaded as the aero axial station in `accumulateAeroAt` (`Part.cpp:214`, recursing
with `axialStation + pos.z()` at `Part.cpp:224`).

This is fragile because **a part's CM is not, in general, at its mid-length.** A solid cone's CM is `L/4`
from the base (`L/3` for a thin shell — `ConicalNoseCone::coneCmOffset`, `ConicalNoseCone.cpp:52`). The
visualizer ignores CM entirely: `RocketMesh::walk` recurses with `station + offset` and translates a
**geometric-centered** primitive (`visualizer/RocketMesh.cpp:427`, the legacy tuple read at `468-469`,
recursion at `472`). So for a nose cone the simulator
(CM-to-CM) and the renderer (geometric-center-to-geometric-center) **disagree by exactly the cone's
(geometric-center − CM) offset**, and authors must hand-compute every offset. The disagreement silently
produced a coupler that interpenetrated the nose in `tests/data/designs/xl75_multi.qrd`.

The fix removes the divergence by construction: one resolver produces a **geometric** placement, both
consumers read it, and CM falls out as a consequence.

---

## 1. Design philosophy & the longitudinal convention

**Three non-negotiable rules:**

1. **Placement is DERIVED, recomputed by ONE resolver.** Never store absolute coordinates; never store the
   resolved pose. This mirrors QtRocket's existing "derive, don't store" discipline (`Part.h:152`,
   `Part.h:310-316`). Storing a resolved transform was considered and rejected: it reintroduces a staleness
   class (edit a part's length and its stored transform is wrong) — the *inverse* of today's bug.
2. **The stored link is physical INTENT** — feature-to-feature relationships — never an opaque offset. CM
   falls out as a consequence.
3. **The common case costs zero authored numbers** — a child with no link abuts its parent.

**The longitudinal convention, settled once: `+z = FORWARD` (toward the nose tip), with every part's local
origin on the longitudinal axis at its *fore* (forward) plane.** A part therefore occupies `z ∈ [−length, 0]`:
its fore plane is the origin (`z = 0`) and its aft plane is at `z = −length`.

Putting the origin at the *fore* plane (rather than the aft) is what makes the whole-rocket datum a
recognizable point. The resolver plants the root (the nosecone) at its own origin — its fore plane, i.e. the
**nose tip** — at the world origin, and because every child is composed off its parent's origin, the entire
assembly is referenced to that one point. The derived center of mass therefore comes back relative to the
**nose tip** (a fixed, recognizable geometric datum), not relative to some part's center of mass. The rocket
extends aft into `−z`; the nose tip is `z = 0` and the fin can is the most negative `z`.

`+z = FORWARD` is also the cheapest correct *direction*, because the code and files already use it:

- `buildCone` uses `zBase = -L/2`, `zTip = +L/2` (`RocketMesh.cpp:93-94`) → tip at `+z`; `buildTube` agrees
  (`RocketMesh.cpp:156`, `zTop = +length/2 // forward`). The visualizer is already `+z = forward`.
- Legacy `xl75_multi.qrd` places the body at `z = -0.6` (aft → `-z`) and the coupler at `z = +0.49` relative
  to the body (forward, toward the nose). The files are already `+z = forward`.

Choosing `+z = aft` would force a whole-axis inversion of the renderer (cone base-cap normal at
`RocketMesh.cpp:132`, sphere poles at `298-300`, every primitive) **and** a sign-negating migration of every
legacy file. `+z = forward` makes the renderer change a *consumer swap* with **no sign inversion**, and the
file migration a pure re-expression with **no axial flip**.

Per part type, in the forward frame (fore plane at the origin, part spanning `z ∈ [−length, 0]`):

- **`ConicalNoseCone`**: fore point (tip) at `z = 0`, `r = 0`; aft plane (base rim) at `z = −length`,
  `r = baseRadius`. Outer radius tapers as `radiusOuterAt(z) = baseRadius·(−z/length)` (so `r → 0` at the tip,
  `r → baseRadius` at the base).
- **`BodyTube` / `FinSet` / `HollowSphere`**: fore plane at `z = 0`, aft plane at `z = −length`.

**One frame for every part — including the cone. No special parts.** Every part uses the local frame just
defined (fore-plane origin, `+z` forward); the cone is no exception. Achieving that requires **correcting a
pre-existing defect**, not designing around it: `coneCmOffset` (`ConicalNoseCone.cpp:52`)
currently reports the cone CM in a *reversed* `mid-length-origin, base-at-+z` frame whose `+z` is the *opposite* of our chosen
forward. That reversal is an implementation mistake, not a design decision. The fix is to **remove** it —
`coneCmOffset` is corrected to report the CM in the shared `+z=forward` frame (`L/2 − hbar` → `hbar − L/2`).
The cone's centroidal inertia tensor is **unchanged**: it is axisymmetric and invariant under the `z → −z`
flip (`Ixx = Iyy`, and `Ixx = ∫(y²+z²)dm` is unaffected by `z → −z`), so only the CM sign was ever wrong.
With this, there is **no frame adapter and no per-part CM override** anywhere; CM is read by one uniform rule
for every part (§2e). The physics tests written around the old sign (`NoseConeTests.cpp`, `AeroTests.cpp`)
are updated in lock-step (§7). Adjusting a handful of tests written around a defect is the price of a model
with no warts to remember, paid once.

---

## 2. Core data model — the exact C++ types

### 2a. Station-pair link

New header `model/parts/Placement.h`. A relationship reduces to: *a fractional axial station on the parent, a
fractional axial station on the child, an axial gap, and a seat kind that governs the radial check and the
gap direction.* (A rich named-datum enum was rejected as simultaneously too rigid — needing a parametric
escape hatch — and too rich: `FORE`/`MID`/`AFT` are merely fractions `1`/`0.5`/`0`.)

```cpp
namespace model::part {

// How a child seats against its parent. Governs (a) the radial compatibility CHECK and
// (b) the SIGN of the gap (insertion direction).
enum class SeatKind : std::uint8_t {
   Abut,         // rim-to-rim, same radius; gap is a forward standoff (signed +z)
   NestInBore,   // child OD seats inside parent ID; gap is insertion DEPTH (child moves AFT, -z, into bore)
   OnSurface     // child seats radially on parent's outer wall at the station; gap is an axial standoff
};

// Physical intent for one parent->child attachment. Stations are FRACTIONS of live length,
// resolved against getLength() every read, so editing a length moves the seam.
struct StationLink {
   double   parentStation01{0.0};   // 0 = aft plane, 1 = fore plane, on the PARENT
   double   childStation01 {1.0};   // 0 = aft plane, 1 = fore plane, on the CHILD (default: fore)
   double   gap{0.0};               // metres; meaning per SeatKind (standoff or insertion depth)
   SeatKind seat{SeatKind::Abut};
   Quaternion childRot{Quaternion::Identity()};   // 6-DOF seam; identity today (see §8)
};

} // namespace model::part
```

An arbitrary station (e.g. a rail button mid-tube) is just a fraction in `[0,1]`; there is no enum to extend.
`SeatKind` stays closed because a rigid axisymmetric stack genuinely expresses only three radial
relationships: rim-to-rim of equal radius (`Abut`), one part's OD inside another's ID (`NestInBore`), and a
part seated on another's outer wall (`OnSurface`). There is no fourth radial relationship to add for the
parts QtRocket models, so leaving room to extend would only invite half-defined cases.

**The `gap` sign is part of each seat's contract.** The resolver places a child by
`childOriginZ = p.z + signedGap − c.z` (§3), with `signedGap = +gap` for `Abut`/`OnSurface` and `−gap` for
`NestInBore`, so `gap` is a displacement of the child along `+z=forward`. The admissible range follows: a
positive `Abut`/`OnSurface` gap stands the child off forward of its flush seat (flush at `gap = 0`); a
**negative `Abut` gap** symmetrically drives the child *into* the part it mates — a self-intersecting
placement, not a standoff. `OnSurface` alone accepts **either sign** — a fore (`gap > 0`) or aft (`gap < 0`)
standoff along the wall. `NestInBore`'s gap is a **non-negative insertion depth** that drives the child aft
into the bore; a negative depth would withdraw it back out of the mouth (not seated). Those degenerate signs
are out-of-contract inputs the authoring helpers (§4) are built not to produce — distinct from the radial
and envelope violations the diagnostics of §5 detect between legitimately-seated parts.

### 2b. Resolved landmark (geometry-derived, never stored)

A part answers, for a fractional station, the live `{z, rOuter, rInner}` in its own `+z=forward` local frame:

```cpp
struct Station {
   double z{0.0};        // axial station, part-local +z-forward (m) = (station01 - 1) * getLength()
                         //   station01: 0 = aft (z = -length), 1 = fore (z = 0, the origin)
   double rOuter{0.0};   // outer radius at z (m)
   double rInner{0.0};   // inner radius at z (m); 0 for solids
};

// On Part. Base implementation uses getLength() + the radius virtuals below, so symmetric
// parts need NO override. Clamps station01 to [0,1] (see §2d).
virtual Station stationAt(double station01) const;
```

The base `stationAt` maps the fraction to `z = (station01 − 1)·getLength()` and queries the radius virtuals,
so every symmetric part inherits a correct `stationAt` with no override. This requires machinery the base
class does not have today: **`getLength()` is promoted to a base `Part` virtual.** There is currently *no*
`getLength()` on `Part` — each concrete part carries its own non-virtual accessor (`BodyTube.h:51`,
`ConicalNoseCone.h:53`) and `HollowSphere` has none (a sphere has no length member). The refactor lifts
`getLength()` onto the base so the shared `stationAt` can call it polymorphically; `HollowSphere` then supplies
a definition (axial extent = diameter, `2·outerRadius`) and the existing accessors become overrides. This is
*added* work, not a pre-existing facility (§11).

### 2c. Extent profile for diagnostics

```cpp
// On Part. Closed-form per part type. Base returns 0 (geometrically inert).
virtual double radiusOuterAt(double zLocal) const { return 0.0; } // cone tapers; tube const; sphere bulges
virtual double radiusInnerAt(double zLocal) const { return 0.0; } // BodyTube bore; 0 for solids
virtual double axialLength()  const { return getLength(); }       // span is [-axialLength(), 0] in local +z
```

**The solid-host rule.** A naive "offender outer radius vs host *inner* radius" test breaks on a solid cone,
which has no bore (`radiusInnerAt → 0`) and would flag everything. The correct, unambiguous rule:

> A host *occupies* radius `[0, hostCapacity(z)]` where `hostCapacity(z) = radiusInnerAt(z)` for a bored part
> and `= radiusOuterAt(z)` for a solid part. An offender at station `z` with outer radius `rOff` *fits* iff
> `rOff ≤ hostCapacity(z) + tol`.

For a solid cone the coupler must fit inside the cone's *outer skin* (`radiusOuterAt`) — exactly the
poke-through test. For a bored tube it must fit inside the bore (`radiusInnerAt`). One predicate, correct for
both, exposed as a non-virtual helper `double Part::innerCapacityAt(double zLocal) const`. A part advertises
`virtual bool isSolid() const { return radiusInnerAt(0.0) <= 0.0; }`, overridden by `BodyTube`/`HollowSphere`
to report their wall honestly.

### 2d. Degenerate-geometry guards

- `stationAt` clamps `station01` to `[0,1]` (an out-of-range link is a load-time warning, not UB).
- `radiusOuterAt(zLocal)` for the cone: `length ≤ 1e-9` → returns `baseRadius` (a zero-length disc/ring), no
  `0/0`.
- A zero-length part (ring/transition) has `axialLength() == 0`, so `stationAt(any) → z=0`; its overlap span
  is a single station, handled by the sweep as a point sample. Documented, not crashing.

### 2e. Center of mass in the local frame (uniform — no adapter)

Because every part lives in the one fore-plane-origin, `+z=forward` frame (§1, after the cone defect is
corrected), center of mass is read the same way for all of them. A part's mid-plane is at `z = −L/2` (midway
between the fore origin at `0` and the aft plane at `−L`), and `getCenterMassOffset()` (`Part.h:96`) reports
the CM relative to that mid-plane, in the same frame. The CM's local axial station is therefore one uniform
expression, evaluated identically for every part:

```cpp
const double cmLocalZ = -getLength()/2.0 + getCenterMassOffset().z();  // mid-plane (-L/2) + mid-referenced offset
```

There is **no `cmStationLocal()` virtual** and **no cone special case**. `getCenterMassOffset()` is *relative
to mid-length*, so it is independent of the origin choice — the cone/FinSet corrections below are the same as
they would be in any frame; only the mid-plane term (`-L/2`) reflects the fore origin. The values fall out
directly:

- **`BodyTube` / `HollowSphere`**: centroidal tensor about a geometric-center CM, `getCenterMassOffset() = 0`,
  so `cmLocalZ = −L/2` (CM at mid-length, half a body aft of the fore plane).
- **`ConicalNoseCone`** (after the §1 defect fix): `getCenterMassOffset().z() = hbar − L/2` (`−L/4` solid,
  `−L/6` shell), so `cmLocalZ = hbar − L` — i.e. `−3L/4` for a solid cone: the CM sits `3L/4` aft of the tip,
  equivalently `hbar = L/4` forward of the wide base, where the mass is. The hand-checked value (§7, Phase 2)
  is `cmLocalZ = hbar − L = −0.225` for `L = 0.30`.
- **`FinSet`**: its CM is at the fin-set mass centroid `x_c`, but `finSetCmOffset` (`FinSet.cpp:55`) currently
  reports `x_c` from one end (the root LE), not from mid-length — a *second* latent reference defect, of the
  same kind as the cone's. It is corrected the same way: report `x_c − L/2`, so
  `cmLocalZ = −L/2 + (x_c − L/2) = x_c − L` falls out of the uniform rule with no override (`FinSetTests`
  updated in lock-step, §7).

The remedy for both the cone and the fin set is the **same** — report the CM relative to mid-length, in the
shared `+z=forward` frame — so the codebase carries **no** reversed frame and **no** non-mid CM reference.
No implicit cross-frame vector addition survives anywhere in the composition path: correcting the cone's
`coneCmOffset` sign removes the only reversed frame in the codebase, closing the simulator/visualizer
divergence at its root rather than papering over it.

### 2f. The link type that replaces `(Part, Vector3)`

`Part.h:326` changes by a single in-slot struct swap (single-vector storage is kept; parallel
`childParts`/`childLinks` arrays were rejected as a desync hazard across `clone()`/`removeChildById`):

```cpp
std::vector<std::pair<std::shared_ptr<Part>, StationLink>> childParts;
```

The swap is in-slot: the storage stays one vector of pairs, the same accessor pattern reads it, and the
composition routines that destructure it (§3) change only their second binding. Because the second element
is no longer consumed as a coordinate but as intent fed to the resolver, the `<tuple>` dependency the header
carries solely for this member can be dropped once the swap lands.

### 2g. What lives where

| Lives on `Part` | Lives in `Placement.h` (free / POD) |
|---|---|
| `stationAt`, `radiusOuterAt/InnerAt`, `axialLength`, `isSolid`, promoted `getLength()` (virtuals) | `Station`, `StationLink`, `SeatKind`, `Pose` |
| `childParts` (now `pair<ptr, StationLink>`); non-virtual helper `innerCapacityAt` | `resolvePlacements(...)`, `sweepOverlaps(...)` |
| `addChildPart(child, StationLink)` + transitional `Vector3` overload | `OverlapDiagnostic`, `SolveResult`, `Placed` |

`getCenterMassOffset()` finally gets consumed for its documented purpose — **uniformly**, via
`−L/2 + getCenterMassOffset().z()`, the same for every part (§2e), never as the link.

---

## 3. The single resolver

One function is the sole authority for absolute placement. Both `computeCompositeAt` and `RocketMesh::walk`
call it.

```cpp
struct Pose {
   Vector3    origin {Vector3::Zero()};        // part-local-frame origin (fore plane, on axis), in ROOT frame
   Quaternion orient {Quaternion::Identity()}; // (x,y,z,w); identity in 3-DOF

   Pose compose(const Pose& childInThis) const {
      return Pose{ origin + orient * childInThis.origin,
                   (orient * childInThis.orient).normalized() };
   }
};

struct Placed { const Part* part; Pose pose; };   // deterministic DFS (attachment) order

// THE resolver. DFS over the ownership tree from rootPose. Pure geometry; no CM, no time.
std::vector<Placed> resolvePlacements(const part::Part& root, const Pose& rootPose);
```

### Algorithm (3-DOF today; 6-DOF-shaped), in `+z = forward`

For a child of an already-placed parent, with `StationLink m`:

```cpp
const Station p = parent.stationAt(m.parentStation01);  // {z, rOuter, rInner} in parent local frame
const Station c = child .stationAt(m.childStation01);

// signedGap encodes seat direction in +z = FORWARD:
//   Abut       => +m.gap   (forward standoff; flush at gap = 0)
//   OnSurface  => +m.gap   (axial standoff along the wall)
//   NestInBore => -m.gap   (m.gap = insertion depth >= 0; child seats AFT, -z, INTO the bore)
const double childOriginZ = p.z + signedGap(m) - c.z;   // child's fore-plane origin, in parent frame

// Coaxial today: x = y = 0. (Radial seat r is enforced as a CHECK, not a placement — see below.)
const Pose childInParent{ Vector3(0.0, 0.0, childOriginZ), m.childRot /* Identity in 3-DOF */ };
const Pose childInRoot   = parentPose.compose(childInParent);
```

The root is visited at `rootPose` (conventionally identity: the root's fore plane — for a nose-led rocket,
the **nose tip** — at the world origin, `+z` forward, the rocket extending into `−z`). Every part is thus
placed relative to that one point, so the derived composite CM (below) comes back **relative to the nose
tip**, not relative to any part's center of mass.

**On radial placement (deliberate scope line):** the radial value `r` is used **only** for the seam/overlap
*check*; the pose solve sets `x = y = 0`. That is correct and sufficient for every axisymmetric part QtRocket
has today (tubes, cones, spheres, and fin sets — whose CM is on-axis by symmetry). True off-axis radial
placement (a single rail button, an asymmetric pod) is **deferred to the 6-DOF work**, because placing a part
at `r > 0` is only physically meaningful once `childRot` and a per-part body-frame offset are integrated —
doing it in 3-DOF would store a half-correct placement, exactly the error class this design exists to
eliminate.

### How CM becomes DERIVED (the core fix)

`computeCompositeAt(t)` is rewritten to consume the resolver's *geometric* placement and derive CM from it,
using the uniform local-CM rule (§2e):

```cpp
// For each child, from its resolved Pose and its OWN-frame CM. Every part lives in the
// same fore-origin, +z-forward frame, so the local CM is one uniform expression:
//   cmLocalZ      = -child.getLength()/2 + child.getCenterMassOffset().z();
//   childCmInRoot = childPose.origin + childPose.orient * Vector3(0, 0, cmLocalZ)
//                   (+ child.getCenterMassOffset() radial components, zero for all current parts)
// Pass 1 (mass-weighted CM):  accumulate childMass * childCmInRoot
// Pass 2 (parallel-axis):     d = childCmInRoot - compositeCm; I += childI + childMass*(|d|^2 I - d d^T)
//                             AND (6-DOF) rotate child tensor: I' = R I R^T, R = childPose.orient
```

`getCenterMassOffset()` is consumed here, by one expression identical for every part type — no per-type
adapter, no cone special case. Placement is geometric (station-to-station); CM is its derived consequence. The
two-pass mass/CM/parallel-axis structure of `computeCompositeAt` (`Part.cpp:169`; the displacement tensor
helper `parallelAxisTerm`, `Part.cpp:20`, applied at `196` and `200`) is **preserved unchanged in shape** —
only its *input* (geometric stations from one resolver) changes.

Because the resolver plants the root at its fore origin — the nose tip — at the world origin, every resolved
station, and therefore `getCompositeCm()`, is expressed relative to the **nose tip**. For a conventional
nose-led rocket the whole-body CG comes back as `(0, 0, −d)` with `d > 0` — the CG sits `d` metres aft of the
tip. (The migration's invariance gate accounts for this deliberate datum change; see §7.)

`accumulateAeroAt` threads each child's resolved `childPose.origin.z()` (relative to the root datum) instead
of `axialStation + pos.z()`. `RocketMesh::walk` translates each geometric-centered primitive by the
resolver's `Pose.origin` (and rotates by `Pose.orient` when 6-DOF arrives). **Both consumers, one resolver,
identical numbers.**

### Caching — resolver split out of the mass gate

`computeCompositeAt` is rebuilt on the *mass*-delta gate — `ensureCompositeCache` (decl `Part.h:291`), keyed
on `builtAtCompositeMass` (`Part.h:308`) — which fires **every step during a burn** (the composite mass
differs each integration step while a motor burns). Placement must not: the station landmarks, radii, seat
gaps, and therefore every resolved `Pose` are rigid functions of the parts' lengths and seat intents,
invariant under mass loss. Placement gets its **own** structural-dirty-gated cache:

- New member `std::vector<Placed> resolvedCache;` rebuilt by `ensurePlacementCache()` **only when the tree is
  structurally dirty** (a part added/removed, or a geometry/length edit), gated by a *separate*
  `placementDirty` flag — distinct from the existing single `needsRecomputing` flag (`Part.h:322`, propagated
  up by `markAsNeedsRecomputing`, `Part.h:294`) — set in `addChildPart`/`removeChildById`/geometry setters,
  **not** by mass change.
- `computeCompositeAt(t)` reads `resolvedCache` (rigid, mass-independent) and only re-weights by `getMass(t)`.
  A burning motor re-runs the cheap mass/CM/parallel-axis arithmetic over **fixed geometry**, never the
  resolver and never the overlap sweep.
- The overlap sweep (§5) runs once per structural change, with the placement resolve — never per ODE step.

---

## 4. Authoring API & the zero-config default

The present tree exposes exactly one attachment entry point — the legacy
`addChildPart(std::shared_ptr<Part>, Vector3)` at `Part.h:239`, whose `Vector3` is the CM-to-CM offset. The
refactor replaces that signature with a link-valued one and keeps the old one alive, deprecated, only long
enough for the phased migration (§7) to complete:

```cpp
// Primary API — replaces addChildPart(child, Vector3):
virtual void addChildPart(std::shared_ptr<Part> child, StationLink link = {});

// Transitional shim (kept through migration so existing tuple call sites + *.qrd tests compile):
//   synthesizes a link reproducing the legacy CM-to-CM station; LOGS a deprecation note. See §7.
[[deprecated]] void addChildPart(std::shared_ptr<Part> child, Vector3 position);
```

The new overload is a strict superset of the old one (a default `StationLink` is the most common
relationship), so the migration is additive. The shim is `[[deprecated]]` rather than deleted: a hard
deletion would break the build the instant the type changes, before any fixture could be ported; the
attribute keeps the tree green while emitting a compile-time warning at each unported call site — turning the
migration into a punch list the compiler maintains.

**Zero-config default, in `+z = forward`.** `StationLink{}` defaults to `parentStation01 = 0` (parent aft
plane), `childStation01 = 1` (child fore plane), `gap = 0`, `seat = Abut` — meaning *the child's fore plane
meets the parent's aft plane*, i.e. a new child **stacks aft of its parent.** This is the natural
nose→body→… build order. With `+z = forward` fixed and stations as explicit fractions, the default is
unambiguous: `parentStation01 = 0` is *always* the aft plane.

```cpp
nose->addChildPart(body);                              // body fore plane abuts nose aft plane — ZERO numbers
body->addChildPart(fins, {.parentStation01 = 0.06,     // wall station near body aft
                          .childStation01 = 0.0,       // fin root aft
                          .seat = SeatKind::OnSurface});
body->addChildPart(coupler, {.parentStation01 = 1.0,   // body fore plane (bore mouth)
                             .childStation01 = 0.0,     // coupler AFT plane (inserted reference)
                             .gap = 0.04,               // insertion depth
                             .seat = SeatKind::NestInBore});
```

**Optional snap-operator verbs** (produce a `StationLink`, never store a pose):

```cpp
StationLink abut       (double gap = 0.0);                 // {0, 1, gap, Abut}   child fore -> parent aft
StationLink nestInBore (double depth);                     // {1, 0, depth, NestInBore}  child aft inserted
StationLink seatOnWall (double parentStation01);           // {parentStation01, 0, 0, OnSurface}
```

---

## 5. Overlap / constraint diagnostics

Three layers, escalating from cheapest to hardest.

```cpp
struct OverlapDiagnostic {
   Part::Id    offender;
   Part::Id    host;          // the part it intrudes into (may be a non-tree neighbour, e.g. the nose)
   double      zWorld;        // located station of worst violation (root frame, +z forward)
   double      penetration;   // metres the offender's radius exceeds the host capacity
   std::string message;
};
struct SolveResult {
   bool ok{true};
   std::vector<OverlapDiagnostic> diagnostics;
};
```

`offender`/`host` are `Part::Id` (`std::uint64_t`, `Part.h:56`) values from `getId()` (`Part.h:191`), not
names or pointers. The choice is deliberate: names are explicitly **not** required to be unique
(`Part.h:193`), so a name-keyed diagnostic could point at the wrong part, and a raw pointer would dangle if
the tree were edited between resolve and report. A `Part::Id` is assigned once at construction, re-minted to a
fresh value on `clone()`, and never changed thereafter (`Part.h:270`) — the only safe handle to carry in a
cached verdict.

**Layer 1 — radial seam check, on every resolve.** Compare the two stations' radii by seat:
- `Abut`: `|p.rOuter − c.rOuter| ≤ tol` (nose base rim `0.0395` == body fore rim `0.0395` → clean seam).
- `OnSurface`: `c.rOuter` (fin `bodyRadius`) `== p.rOuter` (tube OD) within tol.
- `NestInBore`: `c.rOuter ≤ p.rInner + tol` (coupler OD `0.0376` ≤ body ID `0.0376` → fits).

**Layer 2 — envelope sweep, after the whole tree resolves, with a concrete spatial query.**

1. From the resolver's `std::vector<Placed>` (DFS order), build world axial intervals `[zAft_i, zFore_i]` for
   every part with a non-trivial envelope. Sort by `zAft` — **O(N log N)**.
2. For each offender, sample its world span at *feature breakpoints* (its own endpoints, plus every other
   interval's endpoints that fall within its span — analytic, since cone/tube/sphere `r(z)` are monotone or
   closed-form; no thin-protrusion miss). At each sample `zWorld`, find candidate hosts by a sweep over the
   sorted interval list (the active set), and test `offender.radiusOuterAt(local) > host.innerCapacityAt(local)
   + tol` using the **solid-host rule from §2c** (`innerCapacityAt` = bore for tubes, outer skin for solids).
3. **Deterministic tie-break:** when two intervals cover the same station, the host is the one with the
   **smallest `Part::Id`** among covering parts that are not the offender itself nor the offender's own
   ancestor chain via the seat it legitimately occupies. Ids are assigned at construction and re-minted
   deterministically in DFS load order, so attribution is reproducible.

For the coupler nested at the body fore end that projects forward past the body fore rim, the sweep walks into
the **nose cone** (a non-tree neighbour): `nose.radiusOuterAt(local)` is the tapering skin, the solid-host
rule compares the coupler OD against it, and the forward poke-through is flagged with a located `zWorld`.

**FinSet is handled honestly.** A FinSet is *not* axisymmetric, so it does **not** report a scalar
`radiusOuterAt` for the sweep:
- It contributes its **body-disc** radius (`bodyRadius`) to the envelope for *axial* occupancy (so it hosts
  nothing inside, and is not falsely treated as a `bodyRadius + span` disc colliding with every nearby tube).
- Fin-to-fin and fin-to-tube *radial* interference (the `span` extent in azimuthal sectors) is **explicitly
  scoped out of v1** and documented as unchecked, because azimuth-aware sectored interference needs a fin
  azimuth/clocking model QtRocket does not yet have; faking it with a full `bodyRadius + span` disc produces
  false positives against every adjacent body tube. Layer 1 still checks fin-root radial compatibility, and
  the axial placement is exact. `getSpan` (`FinSet.h:64`), `getSweep` (`FinSet.h:65`), and `getTipChord`
  (`FinSet.h:63`) remain available for the future sectored check; the body disc is `getBodyRadius`
  (`FinSet.h:67`, the same disc it reports as `getReferenceArea`, `FinSet.h:72`), and the rejected full-disc
  extent would be `getMaxRadius` = `bodyRadius + span` (`FinSet.h:73`).

**Layer 3 — DOF accounting (scoped).** In 3-DOF a single axial station-pair fully pins `z` because coaxiality
is implicit; we record a one-axial-constraint invariant. Once 6-DOF is enabled, a single `StationLink` cannot
express "concentric to a third part" (that needs the deferred joint graph, §10), so Layer 3 in v1 is a
**detector that emits a warning** when a child has only one link under free orientation; the *resolution* (a
companion concentricity constraint) is deferred.

**The gate:** if `SolveResult.ok == false`, **both** `computeCompositeAt` (`Part.cpp:169`; the routine the
sim drives every ODE step through `ensureCompositeCache`, `Part.h:291` — throws/logs rather than integrate a
self-intersecting solid) and `RocketMesh::walk` (`RocketMesh.cpp:427` — renders the offender in an error
colour + surfaces the diagnostic) refuse the solve. The coupler-through-nose case is a hard, located failure
in *both* views — never a silent render. The `SolveResult` is produced once per structural resolve (§3
caching) and cached, so each consumer reads a single cached boolean and the gate costs nothing per ODE step.

---

## 6. Serialization to `.qrd`

`<offset x y z/>` is replaced per child by a `<link>` element, kept *inline* with the part (names need not be
unique — `Part.h:193` — so a sibling joints block keyed by name is a hazard we avoid).

```xml
<QtRocketDesign version="0.2">                            <!-- minor bump; loader still accepts 0.1 -->
  <part type="NoseCone" name="MultiNose">
    <params baseRadius="0.0395" length="0.30" wallThickness="0" density="1700" solid="true"/>
    <!-- root: no <link> -->
    <children>
      <part type="BodyTube" name="MultiBody">
        <params innerRadius="0.0376" outerRadius="0.0395" length="0.90" density="1700"/>
        <link seat="Abut" parentStation="0.0" childStation="1.0" gap="0"/>   <!-- = default; omittable -->
        <children>
          <part type="BodyTube" name="MultiCoupler">
            <params innerRadius="0.036" outerRadius="0.0376" length="0.08" density="1700"/>
            <link seat="NestInBore" parentStation="1.0" childStation="0.0" gap="0.04"/>
          </part>
          <part type="FinSet" name="MultiFins">
            <params rootChord="0.10" tipChord="0.04" span="0.06" sweep="0.04"
                    thickness="0.003" bodyRadius="0.0395" finCount="6" density="1700"/>
            <link seat="OnSurface" parentStation="0.06" childStation="0.0" gap="0"/>
          </part>
        </children>
      </part>
    </children>
  </part>
  <motor commonName="M1350W"/>
  <sim .../>
</QtRocketDesign>
```

- `DesignSerializer::writeOffset` (`DesignSerializer.cpp:53`) → `writeLink(const StationLink&)`: emits
  `seat`/`parentStation`/`childStation`/`gap`. `SeatKind` ↔ string via a small shared table. A link equal to
  the default is omittable.
- `buildPart` (`DesignSerializer.cpp:117`) reads `link.<xmlattr>.parentStation/childStation/seat/gap` with the
  `0.0`/`1.0`/`Abut`/`0` defaults, then calls `addChildPart(child, StationLink{...})`. A minimal `<part>` with
  no `<link>` still abuts. An unknown `seat` string → load error (same path as today's `makePart` throw).
- **Resolved coordinates are NEVER serialized** — recomputed on load (also the migration validation point).
- **Version gate:** the target writer emits `0.2`; the loader's existing `major == "0"` tolerance
  (`DesignSerializer.cpp:186` — it splits on the dot at `185` and rejects only an unrecognized *major*)
  already accepts both `0.1` (legacy `<offset>`) and `0.2` (`<link>`). The reader distinguishes the two
  element shapes *structurally* (which element is present on the child), not by the version string, so **no
  loader gate change is needed.**
- **Current vs. target:** in the present tree the writer hard-codes `"0.1"` (`DesignSerializer.cpp:156`) and
  `writeOffset` still emits the `x`/`y`/`z` triple (`:53`), added to the child as the `offset` sub-element in
  `writePart` (`:70`). The `0.2`/`<link>` encoding above is the **target** of this design, not current
  behavior; the only file-format work the loader gate needs is none.

---

## 7. Migration plan (phased, build stays green, sign-correct)

The migration is **analytic and physics-invariant**, gated by a unit test built **first**.

**Phase 0 — invariance gate FIRST.** Before any type changes: add a test that loads every
`tests/data/designs/*.qrd` through the *current* reader and snapshots `getCompositeCm(0)`,
`getCompositeI(0)`, and the resolved axial station of every part in the legacy `+z=forward` CM-to-CM
convention. These snapshots are the ground truth the migration must reproduce — compared against the legacy
reader's own output, not a re-derivation that could share a bug. **Datum note:** `getCompositeMass` and the
composite inertia *about the CM* are datum-independent and are compared bit-for-bit directly. `getCompositeCm`
is datum-relative, and this design **deliberately changes its datum**: the incumbent reports the CG relative to
the root part's *own* CM, whereas the new model reports it relative to the root's *fore plane* — the **nose
tip** (§1, §3). That is the intended improvement (a recognizable geometric datum), so the gate does **not**
expect a bit-identical `getCompositeCm`; instead it compares the CG after re-expressing the legacy snapshot
into the tip datum by the one fixed, known offset between them — the root's own CM station,
`cmLocalZ_root = −L/2 + getCenterMassOffset().z()` (e.g. `−0.225` for a solid `L=0.30` nose). The shim
reproduces the per-part CM-to-CM *differences* exactly, so once that single datum shift is applied the CG
matches as well.

**Phase 1 — types & resolver land, both APIs coexist.** Add `Placement.h`, the
`stationAt`/`radius*At`/`isSolid` virtuals + promoted `getLength()` (base + 4 concrete parts), the
`StationLink` pair storage, the resolver + sweep, and the `[[deprecated]] addChildPart(child, Vector3)` shim.
**Correct the two latent CM-reference defects** in the same phase so the uniform CM rule (§2e) holds with no
special parts: fix `coneCmOffset` (`ConicalNoseCone.cpp:52`) to report the CM in the shared `+z=forward` frame
(`L/2 − hbar` → `hbar − L/2`), and fix `finSetCmOffset` (`FinSet.cpp:55`) to report relative to mid-length
(`x_c` → `x_c − L/2`). Both centroidal tensors are unchanged. Update the tests written around the old
references (`NoseConeTests.cpp`, `FinSetTests.cpp`, `AeroTests.cpp`) in lock-step. Swap the storage slot
(`Part.h:326`) in place, and rewrite `computeCompositeAt` (`Part.cpp:169`), `accumulateAeroAt`
(`Part.cpp:214`), and `RocketMesh::walk` (`RocketMesh.cpp:427`) to consume the resolver. Build green; the shim
reproduces legacy behaviour:

```
// All in +z = forward. legacyOffset.z is child CM minus parent CM (CM-to-CM).
// Recover the child's fore-plane origin in the parent frame:
childOriginZ_fwd = parentCmLocalZ - childCmLocalZ + legacyOffset.z
//   where *CmLocalZ = -getLength()/2 + getCenterMassOffset().z() -- the same uniform local-CM
//   expression the forward derivation uses (§2e), so it inverts cleanly for every part.
// Back out the link from the chosen station pair:
gap_unsigned = (childOriginZ_fwd + childStation.z - parentStation.z)   // before seat sign
seat         = inferred from the part pair (tube fore-rim<->tube aft-rim => Abut;
               small-tube-OD inside large-tube-ID => NestInBore; fin root on wall => OnSurface)
gap          = (seat == NestInBore) ? -gap_unsigned : +gap_unsigned    // SEAT SIGN APPLIED LAST
```

Two load-bearing details: (1) the sign convention is `+z = forward` throughout, so the legacy coupler's
`+0.49` is correctly read as *forward* (toward the nose) and the migration does **not** mirror the rocket;
(2) the seat is inferred **before** the gap sign is applied (because `NestInBore` flips the sign), so the
"reproduces the exact station" invariant holds for *every* seat, not only `Abut`/`OnSurface`.

**Phase 2 — run the invariance gate against the migrated reader.** Load every fixture through the
`0.2`-capable reader (legacy `<offset>` → shim → `StationLink`) and assert **bit-identical**
`getCompositeCm(0)`, `getCompositeI(0)`, and resolved stations against the Phase-0 snapshots. A dedicated
sub-test pins the **nose CM** to a hand-computed value (solid cone, `L = 0.30`:
`cmLocalZ = −L/2 + (hbar − L/2) = hbar − L = −0.225`, i.e. `3L/4 = 0.225` aft of the tip, equivalently
`hbar = L/4` forward of the base) — the guard that the cone-frame correction (§1) landed correctly.

**Phase 3 — cut over fixtures & retire the shim.** Re-save the corpus to `0.2`. The `xl75_multi` coupler —
legacy-placed forward into/through the nose — migrates to a `NestInBore` link whose **Layer-2 sweep fires**
(the coupler OD exceeds the tapering nose skin forward of the body fore rim), so the migration **reports the
pre-existing bug** rather than silently preserving it. Fix the fixture (reduce insertion depth / shorten the
coupler) until it resolves clean, re-run the gate against the fresh `0.2` files, then remove the
`[[deprecated]]` overload.

---

## 8. 6-DOF path

Shaped in from day one at zero schema cost:

- `Pose` carries `Quaternion orient`; `compose()` rotates the child translation correctly. Today
  `m.childRot` is identity, so behaviour is bit-identical to the current pure-translation tree.
- `StationLink.childRot` is the per-seam orientation (canted fins, side boosters) — a *value* in the existing
  struct, no storage reshape.
- The one new composition line (§3) rotates each child tensor `I' = R I R^T` (`R = Pose.orient`) before the
  parallel-axis shift. `Part.h:43-45` documents "no rotation" as the *current* limit; this localizes the
  change to one resolver line.
- **Off-axis radial seating** (rail buttons, asymmetric pods) lights up here, not before: with `childRot`
  integrated and a per-part body-frame CM offset, `OnSurface` can place a part at `x = r` *correctly
  oriented*. Deferred deliberately (see §3 rationale). The second ingredient is the latent purpose of
  `getCenterMassOffset()` (`Part.h:96`, returning the `cm` member documented at `Part.h:310-316` as the
  natural home for that offset once asymmetric parts or 6-DOF need it): in 3-DOF only its *axial* component is
  consumed (via `−L/2 + getCenterMassOffset().z()`); under 6-DOF its *radial* components, rotated by
  `childPose.orient`, place a genuinely off-axis part's mass correctly in the root frame.
- A future body-frame integrator multiplies `rootPose` by the rocket attitude quaternion; the same resolver
  yields every part's world pose for aero/thrust. No schema or resolver-signature change.

---

## 9. Worked example — `xl75_multi`'s four relationships, in `+z = forward`

Root = NoseCone "MultiNose" (`L = 0.30`, `baseRadius = 0.0395`) with its **fore point (tip) at world `z = 0`**
(the resolver plants the root origin there), aft plane (base rim) at world `z = −0.30`. Stations: tip
`{z=0, rOuter≈0}`, aft `{z=−0.30, rOuter=0.0395}`; `radiusOuterAt(z) = 0.0395·(−z/0.30)`. Solid →
`innerCapacityAt(z) = radiusOuterAt(z)` (the skin). Nose CM (solid) at `cmLocalZ = hbar − L = −0.225`
(world `z = −0.225`, i.e. `3L/4` aft of the tip / `L/4` forward of the base), read uniformly as
`−L/2 + getCenterMassOffset().z()` like every part (§2e). The whole rocket extends into `−z` from the tip.

**(1) body's fore plane ABUTS nose's aft plane** — `addChildPart(body)` (defaults, **zero authored
numbers**): `StationLink{parentStation01=0, childStation01=1, gap=0, seat=Abut}`.
`p = nose.stationAt(0) = {z=−0.30, rOuter=0.0395}` (nose aft); `c = body.stationAt(1) = {z=0, rOuter=0.0395}`
(body fore). `childOriginZ = −0.30 + 0 − 0 = −0.30`. The body's fore origin lands at world `−0.30`, so the
body spans world `z ∈ [−1.20, −0.30]` (it extends *aft*, `−z`, from the nose base — physically correct).
Layer 1: `|0.0395 − 0.0395| = 0` → clean seam.

**(2) fin root lies on the body's outer surface near the aft end** — `seat=OnSurface`:
`StationLink{parentStation01=0.06, childStation01=0, gap=0, seat=OnSurface}`.
`p = body.stationAt(0.06) = {body-local z=(0.06−1)·0.90=−0.846 → world −1.146, rOuter=0.0395}`. Layer 1: fin
`bodyRadius 0.0395 == tube OD 0.0395` → OK. The fin root (chord `0.10`) seats axially near the body aft
(world `≈ −1.20`). (Radial seat is the §3-deferred 6-DOF concern; the fin's mass is axisymmetric, so its
3-DOF placement is exact.)

**(3) coupler NESTS inside the fore end of the body** (OD `0.0376` == body ID `0.0376`) — `seat=NestInBore`,
with the coupler's **aft** plane as the inserted reference:
`StationLink{parentStation01=1.0, childStation01=0.0, gap=0.04, seat=NestInBore}`.
`p = body.stationAt(1.0) = {body-local z=0, rInner=0.0376}` (the body's fore bore mouth, world `z=−0.30`).
`signedGap(NestInBore) = −0.04`. `c = coupler.stationAt(0.0) = {z=−0.08}` (coupler aft plane).
`childOriginZ = 0 + (−0.04) − (−0.08) = +0.04` (body frame) → world `−0.30 + 0.04 = −0.26` (coupler fore
origin). The coupler spans world `z ∈ [−0.34, −0.26]`: its aft `0.04` (down to `−0.34`) is buried in the bore
(mouth at `−0.30`), and its fore `0.04` (up to `−0.26`) *projects past the body fore rim toward the nose* (the
`0.08` coupler is longer than its `0.04` insertion depth). Layer 1: coupler OD `0.0376 ≤ body ID 0.0376` →
fits radially.

**(4) the coupler must NOT poke through the tapering nose — how it's PREVENTED.** The coupler's fore `0.04`
lies at world `z ∈ [−0.30, −0.26]` — *forward of the body fore rim (−0.30), inside the nose region*. Layer 2
envelope sweep:
- Over `z ∈ [−0.34, −0.30]`: host is the body bore, `innerCapacity = rInner = 0.0376` → coupler
  `0.0376 ≤ 0.0376` → OK.
- Over `z ∈ [−0.30, −0.26]`: forward of the body fore rim — the host found by the sorted-interval query is the
  **nose cone** (solid). At the rim, `nose.radiusOuterAt(−0.30) = 0.0395·(0.30/0.30) = 0.0395` (the coupler
  fits); at the coupler fore, `nose.radiusOuterAt(−0.26) = 0.0395·(0.26/0.30) = 0.0342`. The solid-host rule
  compares coupler OD `0.0376` against the skin: `0.0376 > 0.0342` → `OverlapDiagnostic{offender=coupler,
  host=nose, zWorld=−0.26, penetration=0.0034, "coupler OD 0.0376 exceeds nose skin radius 0.0342 at z=−0.26
  (0.04 m forward of the body rim; poke-through 0.0034 m)"}`, `solve.ok = false`. Both sim and viz refuse.

This is the exact bug the legacy file produced silently: `<offset z="0.48999...">`
(`xl75_multi.qrd:19`, CM-to-CM, `+z=forward`) slid the coupler *forward* through the cone, and both the
simulator (threading `axialStation + pos.z()` at `Part.cpp:224`, shifting child tensors to the composite CM
in `computeCompositeAt`, `Part.cpp:169`) and the renderer (walking with `station + offset` at
`RocketMesh.cpp:472`, the legacy tuple destructure at `:468`) drew it there — diverging from each other by the
cone CM offset *on top of* the shared overlap error. In the new model the relationship is "nest 40 mm into the body's fore bore", `NestInBore`
constrains the insertion direction, and the envelope sweep makes any forward projection past the bore mouth a
hard, located failure with the correct solid-host comparison. Reduce `gap` (or shorten the coupler) until it
resolves clean.

---

## 10. Deliberately deferred / not adopted

- **`+z = aft`** — rejected; sign-inverted relative to the code and files. The visualizer
  (`buildCone`/`buildTube`) and every legacy `.qrd` are `+z = forward`. `+z = aft` would force a full renderer
  inversion and a sign-negating file migration. We adopt `+z = forward`: zero renderer flip, no axial flip in
  migration.
- **A stored resolved transform as ground truth** — rejected; reintroduces placement staleness (the inverse of
  today's bug) and contradicts "derive, never store". We take the `Pose`/`compose`/tensor-rotation mechanics
  and snap verbs, but re-derive on every read and store only intent.
- **A rich named-datum enum with an escape hatch** — rejected as over-engineering; it pays the cost of a rigid
  enum *and* the cost of escaping it. Replaced by two fractional stations `∈ [0,1]` + `SeatKind`. Edges
  (`FORE`/`AFT`/`MID`) are just fractions; an arbitrary mid-tube station is a fraction; nothing to extend.
- **A full joint graph + sibling `<joints>` block** — deferred, not dismissed. Correct long-term for
  clusters/struts/off-axis pods and the 6-DOF companion-concentricity constraint Layer 3 needs. For a rigid
  inline stack the graph is over-engineering today; the inline `pair<ptr, StationLink>` can later widen to a
  graph if real multi-host designs demand it. Note "single parent-to-child seam" qualifies the *incoming*
  edge, not the fan-out: `childParts` is a *vector*, so a parent may own many children (branching is
  supported today) — but each child has exactly one parent and one mate, the defining property of a *tree*.
  The deferred cases need not more *children* but a second *incoming* edge (a part bound to more than one
  host), which closes a loop a tree can't hold — the jump from tree to graph.
- **Parallel `childParts` + `childLinks` vectors** — rejected as a desync hazard across
  `clone()`/`removeChildById`/`addChildPart`. Single `pair<ptr, StationLink>` vector kept.
- **"Every link mandatory"** — rejected; punishes the trivial stack. The defaulted abut is kept.
- **Scalar `radiusOuterAt` for FinSet** — rejected as dishonest (non-axisymmetric). Body disc for axial
  occupancy; sectored fin interference scoped out of v1, pending a fin-clocking model.
- **Off-axis (`r > 0`) placement in 3-DOF** — deferred to 6-DOF. Placing mass at `x = r` without orientation
  is itself a half-correct stored placement. The `r` value drives the seam/overlap *check* now; true off-axis
  seating arrives with `childRot` (§8).

---

## 11. Files this design touches

- `model/parts/Part.h` — storage member `childParts` (`Part.h:326`) swapped `tuple<…,Vector3>` →
  `pair<…,StationLink>`; geometry virtuals (`stationAt`, `radiusOuterAt/InnerAt`, `axialLength`, `isSolid`,
  promoted base `getLength()` — none exist today; the only base geometry virtual is `getReferenceArea`,
  `Part.h:176`); non-virtual `innerCapacityAt` helper; the new `StationLink` `addChildPart` overload + the
  `[[deprecated]]` shim (promoting from `Part.h:239`); the `getChildParts` return type (`Part.h:212`); a new
  `placementDirty` flag + `resolvedCache` member.
- `model/parts/Placement.h` (new) — `Station`, `StationLink`, `SeatKind`, `Pose`, `Placed`,
  `resolvePlacements`, `sweepOverlaps`, `OverlapDiagnostic`, `SolveResult`.
- `model/parts/Part.cpp` — `computeCompositeAt` (`Part.cpp:169`, consume resolver, derive CM via
  `−L/2 + getCenterMassOffset().z()`, preserve the two-pass mass/parallel-axis shape), `accumulateAeroAt`
  (`Part.cpp:214`, thread each child's resolved `Pose.origin.z()` instead of `axialStation + pos.z()` at
  `:224`), new `ensurePlacementCache`, and the tuple→pair destructure sites updated.
  **Verified destructure sites (live tree):** `clone` (`:117`, incl. the `emplace_back` at `:121`),
  `getCompositeMass` (`:131`), `computeCompositeAt` pass 1 (`:179`), `accumulateAeroAt` (`:221`),
  `maxFrontalReferenceArea` (`:231`), `findById` (`:244`), and both loops of `removeChildById` (the
  `std::get<0>` direct-child scan at `:261` and the recursive descent at `:272`). **Two corrections to the
  original sketch's list (`179, 197, 221, 231, 244, 261, 272`):** line **197 is NOT a `childParts` site** (it
  destructures the local `vector<pair<Vector3, CompositeProperties>>` accumulator in `computeCompositeAt`
  pass 2, unaffected by the swap), and the sketch **omits `clone` (`:117`) and `getCompositeMass` (`:131`)** —
  both bind the element shape and must be migrated, or `clone`/`getCompositeMass` are left broken.
- `model/parts/ConicalNoseCone.{h,cpp}` — adds `radiusOuterAt` (linear taper `baseRadius·(1 − z/L)`) and
  `isSolid`; **corrects the reversed-frame defect** in `coneCmOffset` (`ConicalNoseCone.cpp:52`,
  `L/2 − hbar` → `hbar − L/2`); centroidal tensor unchanged (§1).
- `model/tests/NoseConeTests.cpp`, `model/tests/AeroTests.cpp` — update the cone CM-offset (`L/2 − hbar` →
  `hbar − L/2`, e.g. solid `+L/4` → `−L/4`) and aero `x_cp` assertions to the corrected frame, in lock-step
  with the `coneCmOffset` fix.
- `model/parts/BodyTube.{h,cpp}`, `FinSet.{h,cpp}`, `HollowSphere.{h,cpp}` — each gains `radiusOuterAt`/
  `radiusInnerAt` + `isSolid` backed by its existing wall radii (`BodyTube.h:49-51`, `HollowSphere.h:52-53`);
  `FinSet` reports its **body-disc** radius only (`getBodyRadius`, `FinSet.h:67`) for axial occupancy, with
  sectored fin interference scoped out (§5). **`FinSet::finSetCmOffset` corrected** (`FinSet.cpp:55`) to
  mid-reference its CM (`x_c` → `x_c − L/2`), §1/§2e. **`HollowSphere` gains a `getLength()` override** — a
  sphere has no length member (`HollowSphere.h:52` exposes only radii), so its axial extent is reported as its
  diameter, `2·outerRadius`; without it the inherited default `axialLength()` has nothing to read.
- `model/tests/FinSetTests.cpp` — update the CM-offset assertion to the mid-referenced value
  (`off.z() == x_c` → `x_c − L/2`), in lock-step with the `finSetCmOffset` fix.
- `visualizer/RocketMesh.cpp` — `walk` (`RocketMesh.cpp:427`) consumes the resolver's `Pose` instead of
  accumulating `station + offset` (`:472`); a pure consumer swap with **no axis flip** because the convention
  is `+z=forward`, already the visualizer's own frame (`buildCone` tip at `+z`, `:94`; `buildTube` forward at
  `+z`, `:156`).
- `model/DesignSerializer.cpp` — `writeOffset` (`:53`) → `writeLink`, `buildPart` (`:117`) reads `<link>`,
  version `0.1` → `0.2` (`:156`); the loader's `major == "0"` tolerance (`:186`) already accepts it, so no gate
  change.
- **A new base virtual: `getLength()`.** Easy to overlook — today there is *no* `getLength()` on `Part`; it
  exists only as a non-virtual accessor on `ConicalNoseCone` (`:53`) and `BodyTube` (`:51`), while `FinSet` and
  `HollowSphere` have none. The base `stationAt` and the default `axialLength()` both need a part-level length,
  so the refactor promotes `getLength()` to a base virtual and the existing accessors become overrides.
- **Blast radius beyond `Part.cpp` (the public `getChildParts()` return type).** The destructure sites above
  are private to `Part`. The change with reach beyond the class is the accessor: `getChildParts` (`Part.h:212`)
  changes its return type from the `tuple` vector to the `pair<ptr, StationLink>` vector, so every external
  iterator breaks at compile time and must be migrated — a `std::get<0>` → `.first` / `std::get<1>` →
  `.second` (or structured-binding) rewrite. **The shim does NOT cover the accessor:** the element type is part
  of the public contract and there is no way to return both shapes from one signature, so accessor consumers
  migrate explicitly, in lockstep with the storage swap. Confirmed call sites (verified against the live tree): the
  visualizer tree walk (`RocketMesh.cpp:466`, reaching the element via `std::get<0>`/`std::get<1>` at
  `:468-469`) and the serializer's child-writing loop (`DesignSerializer.cpp:73`) — both already appear above
  for their primary changes, so the accessor migration folds in regardless. Beyond those, the verified external
  iterators are: the CLI tree walk (`cli/Repl.cpp:223`, structured binding), the model's recursive walk
  (`model/RocketModel.cpp:19`, structured binding), the visualizer's part count (`VisualizerWindow.cpp:65-67`,
  `std::get<0>`), and the tests `DesignPersistenceTests.cpp` (`:71, 83, 237-239`), `PartTests.cpp`
  (`:412-415, 516, 518`, plus the friend-accessor `childAt` at `:530` that reads `childParts` directly). Each
  is a `std::get<N>` → `.first`/`.second` (or structured-binding) rename — and the tests that assert on the
  stored `Vector3` (e.g. `DesignPersistenceTests.cpp:83`, `PartTests.cpp:414-415`) must be rewritten to assert
  on the `StationLink` fields or the resolved station instead. Mechanical and enumerated, not hidden.

---

## 12. Incremental build sequence & test plan

The migration of §7 is the *outer* spine (build stays green, invariance gate first). This section refines it
into an ordered sequence of small, independently buildable steps — each with a crisp definition-of-done (DoD)
and the exact gating test(s). It does not add scope beyond §7; it makes §7 executable. The two distinct cache
gates (structural `placementDirty` vs. the per-step mass-delta gate, §3) and the degenerate-geometry guards
(§2d) are first-class test targets, not afterthoughts.

**Step 0 — invariance gate (Phase 0).** New test in `tests/` (e.g. `PlacementInvarianceTests.cpp`): load all
24 `tests/data/designs/*.qrd` through the *current* reader; snapshot `getCompositeMass(0)`, `getCompositeI(0)`
(both datum-independent), and the per-part resolved axial station. *DoD:* test compiles and passes against the
unmodified tree; snapshots are the legacy reader's literal output, not a re-derivation. *Gate:* the new test
binary is green; no production type changed yet.

**Step 1 — `Placement.h` value types only.** Add `SeatKind`, `StationLink`, `Station`, `Pose`, `Placed`,
`OverlapDiagnostic`, `SolveResult` as pure POD/value types; no `Part` change. *DoD:* header compiles
standalone (include-what-you-use), and `Pose::compose` with identity orient is a pure translation. *Gate:* a
unit test `compose(identity)` returns vector addition bit-for-bit; default `StationLink{}` is
`{0, 1, 0, Abut}`.

**Step 2 — geometry virtuals on `Part` + the 4 concrete parts.** Promote `getLength()` to a base virtual
(override on `ConicalNoseCone`, `BodyTube`; new `HollowSphere::getLength() = 2·outerRadius`); add `stationAt`,
`radiusOuterAt/InnerAt`, `axialLength`, `isSolid`, and the non-virtual `innerCapacityAt`. No storage swap yet.
*DoD + tests* (new cases in the existing per-part suites):
  - `BodyTubeTests`: `radiusOuterAt(z) == outerRadius`, `radiusInnerAt(z) == innerRadius` for all `z`;
    `isSolid()` false when `innerRadius > 0`, true when `innerRadius == 0`.
  - `NoseConeTests`: `radiusOuterAt(0) ≈ 0`, `radiusOuterAt(−L) == baseRadius`,
    `radiusOuterAt(−L/2) == baseRadius/2`; `isSolid()` true (no bore); the **degenerate guard**
    `radiusOuterAt` with `L ≤ 1e-9` returns `baseRadius` (no `0/0`).
  - `InertiaTensorsTests`/`PartTests`: `stationAt(1).z == 0`, `stationAt(0).z == −L`, clamp `stationAt(2)` to
    `z == 0` and `stationAt(−1)` to `z == −L` (degenerate-station guard); zero-length part →
    `stationAt(any).z == 0`.
  - `HollowSphereTests`/`PartTests`: `getLength() == 2·outerRadius`, `axialLength() == 2·outerRadius`.

**Step 3 — correct the two CM-reference defects (with their tests, in lock-step).** Fix
`coneCmOffset` (`ConicalNoseCone.cpp:52`, `L/2 − hbar` → `hbar − L/2`) and `finSetCmOffset`
(`FinSet.cpp:55`, `x_c` → `x_c − L/2`); both centroidal tensors unchanged. *DoD + gating tests:*
  - `NoseConeTests`: `getCenterMassOffset().z() == −L/4` (solid), `−L/6` (shell); the **uniform local-CM**
    `−L/2 + getCenterMassOffset().z() == hbar − L`, i.e. **`−0.225` for a solid `L = 0.30` cone** (`3L/4` aft
    of the tip). The cone's `getI()` tensor is **unchanged** (re-assert the old expected tensor — invariant
    under `z → −z`).
  - `FinSetTests`: `getCenterMassOffset().z() == x_c − L/2`, so `cmLocalZ == x_c − L`; tensor unchanged.
  - `AeroTests`: `x_cp`-from-CM assertions that read `getCenterMassOffset` adjusted to the corrected sign.

**Step 4 — the resolver + Layer-1 seam check.** Implement `resolvePlacements` (DFS, per-child
`childOriginZ = p.z + signedGap − c.z`) and the Layer-1 radial check. *DoD + tests* (new
`PlacementResolverTests.cpp`): build a 2-part stack in code; assert the resolved poses match hand-computed
values. **Worked numbers from `xl75_multi`:** Abut body origin `z == −0.30`, body span `[−1.20, −0.30]`;
OnSurface fin root at body-local `z == −0.846` (world `≈ −1.146`); NestInBore coupler origin `z == −0.26`,
span `[−0.34, −0.26]`. Layer-1 seam: clean `Abut` `|0.0395 − 0.0395| == 0`; clean `NestInBore`
`0.0376 ≤ 0.0376`; a deliberately over-wide coupler trips the `NestInBore` radial check.

**Step 5 — Layer-2 envelope sweep + solid-host rule.** Implement `sweepOverlaps` (build/sort intervals
`O(N log N)`, sample at feature breakpoints, `innerCapacityAt` per host, smallest-`Part::Id` tie-break
excluding the offender's own ancestor chain). *DoD + tests:* the `xl75_multi` coupler poke-through fires —
`OverlapDiagnostic{offender = coupler, host = nose, zWorld == −0.26, penetration ≈ 0.0034}` with `ok == false`
(coupler OD `0.0376` > nose skin `radiusOuterAt(−0.26) == 0.0342`); the in-bore span `[−0.34, −0.30]` is clean
(`0.0376 ≤ body ID 0.0376`); a clean fixture (reduced depth / shorter coupler) yields `ok == true`. A solid
cone alone hosts a thin inner rod correctly (solid-host rule uses the skin, not a `0` bore).

**Step 6 — storage swap + consume the resolver (the green-keeping cut, Phase 1).** Swap `childParts`
(`Part.h:326`) to `pair<…, StationLink>`; rewrite `computeCompositeAt` (`Part.cpp:169`), `accumulateAeroAt`
(`Part.cpp:214`), and all destructure sites (clone `:117`, getCompositeMass `:131`, … per §11); migrate the
`getChildParts()` accessor consumers (§11); add the `[[deprecated]]` `Vector3` shim with the §7 recovery
formula. Add the two caches: `resolvedCache` gated by `placementDirty`, and the gate on `SolveResult.ok`.
*DoD + tests:*
  - **Phase-2 invariance gate (Step 0 re-run):** every fixture, through the shim, reproduces
    `getCompositeMass(0)`/`getCompositeI(0)` **bit-identically** and `getCompositeCm(0)`/stations after the one
    known datum shift (`cmLocalZ_root`); equality is exact, not within tolerance.
  - **Cache-gate tests** (new in `PartTests`): mutating only mass (a burning motor) does **not** set
    `placementDirty` and does **not** re-run the resolver (resolve count unchanged); `addChildPart`/
    `removeChildById`/a length edit **does** set it; `computeCompositeAt(t)` over fixed geometry re-weights by
    `getMass(t)` only.
  - **Regression:** `DesignPersistenceTests`, `PartTests`, `RocketModelTests`, `PhysicsIntegrationTests`,
    `PropagatorTests`, `DesignMatrixTests` (incl. `-L heavy`) all green — the physics path is byte-stable.

**Step 7 — serialization to `<link>` (writer 0.2, reader both).** `writeOffset` → `writeLink`, `buildPart`
reads `<link>` with `{0.0, 1.0, Abut, 0}` defaults, version → `0.2`. *DoD + tests* (`DesignPersistenceTests`):
round-trip a `0.2` design (write → read → compare links field-by-field); a `0.2` file with **no** `<link>`
abuts; an unknown `seat` string is a load error (same path as unknown part type); a legacy `0.1` `<offset>`
file still loads via the shim; a default-equal link is omitted by the writer and reconstructed by the reader.

**Step 8 — cut over fixtures & retire the shim (Phase 3).** Re-save the corpus to `0.2`; the `xl75_multi`
coupler's Layer-2 sweep fires on migration — fix the fixture (reduce insertion depth / shorten coupler) until
`ok == true`; re-run the invariance gate against the fresh `0.2` files; remove the `[[deprecated]]` overload.
*DoD:* corpus is `0.2`, every fixture resolves clean, the deprecated setter is gone, and a `git grep`
finds no remaining `std::tuple<…, Vector3>` / `<offset>` writer.

| Component | Unit | Integration | Migration |
|---|---|---|---|
| `stationAt` / radius profile / guards | per-part suites (Step 2), incl. clamp + zero-length + taper-divide | — | — |
| CM correction (cone, fin set) | `NoseConeTests` (`−0.225`), `FinSetTests` (`x_c − L`), tensor-unchanged | — | Phase-2 nose-CM hand-check |
| Resolver + Layer 1 | `PlacementResolverTests` (worked `xl75_multi` stations + seams) | — | — |
| Layer 2 sweep + solid-host | poke-through (`0.0034` @ `−0.26`), clean fixture, solid-host rod | — | Phase 3 migration fires the sweep |
| Storage swap + derived CM | cache-gate tests | full ctest suite byte-stable | Phase-0/2 bit-identical invariance |
| Serialization `<link>` 0.2 | seat ↔ string table, default omit, unknown-seat error | `DesignPersistenceTests` round-trip | `0.1`→shim→`0.2` round-trip |
