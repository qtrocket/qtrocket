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

Citations are pinned to commit `6078c5e` (branch `3Dviewer_test`).

---

## 0. The problem this replaces

QtRocket stores each child as `std::vector<std::tuple<std::shared_ptr<Part>, Vector3>> childParts`
(`model/parts/Part.h:326`), where the `Vector3` is the child's center of mass **relative to the parent's
center of mass** (CM-to-CM). That offset is consumed in `Part::computeCompositeAt` (`Part.cpp:179-201`, it
shifts each child tensor to the composite CM via the parallel-axis theorem) and threaded as the aero axial
station in `accumulateAeroAt` (`Part.cpp:221-224`).

This is fragile because **a part's CM is not, in general, at its mid-length.** A solid cone's CM is `L/4`
from the base (`L/3` for a thin shell — `ConicalNoseCone::coneCmOffset`, `ConicalNoseCone.cpp:52-56`). The
visualizer ignores CM entirely: `RocketMesh::walk` recurses with `station + offset` and translates a
**geometric-centered** primitive (`visualizer/RocketMesh.cpp:427-475`). So for a nose cone the simulator
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

**The longitudinal convention, settled once: `+z = FORWARD` (toward the nose tip).** Every part's local
origin sits on the longitudinal axis at its **aft plane**, so the part occupies `z ∈ [0, +length]` going
forward.

This is the convention the code and files already use, so it is the cheapest correct choice:

- `buildCone` uses `zBase = -L/2`, `zTip = +L/2` (`RocketMesh.cpp:93-94`) → tip at `+z`; `buildTube` agrees
  (`RocketMesh.cpp:156`, `zTop = +length/2 // forward`). The visualizer is already `+z = forward`.
- Legacy `xl75_multi.qrd` places the body at `z = -0.6` (aft → `-z`) and the coupler at `z = +0.49` relative
  to the body (forward, toward the nose). The files are already `+z = forward`.

Choosing `+z = aft` would force a whole-axis inversion of the renderer (cone base-cap normal at
`RocketMesh.cpp:132`, sphere poles at `298-300`, every primitive) **and** a sign-negating migration of every
legacy file. `+z = forward` makes the renderer change a *consumer swap* with **no sign inversion**, and the
file migration a pure re-expression with **no axial flip**.

Per part type, in the forward frame:

- **`ConicalNoseCone`**: aft plane (base rim) at `z=0`, `r = baseRadius`; fore point (tip) at `z = +length`,
  `r = 0`. Outer radius tapers as `radiusOuterAt(z) = baseRadius·(1 − z/length)`.
- **`BodyTube` / `FinSet` / `HollowSphere`**: aft plane at `z=0`, fore plane at `z = +length`.

**The one frame collision we reconcile explicitly: the cone's internal CM frame.** `coneCmOffset`
(`ConicalNoseCone.cpp:52-56`) lives in a *mid-length-origin, base-at-+z* frame whose `+z` is the *opposite*
of our chosen forward. We do **not** flip the cone's internal tensor/CM math (it is correct and tested);
instead the resolver applies one explicitly-specified, unit-tested frame adapter for the cone (§2e). This is
the single most correctness-critical detail in the design.

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
   double   childStation01 {0.0};   // 0 = aft plane, 1 = fore plane, on the CHILD
   double   gap{0.0};               // metres; meaning per SeatKind (standoff or insertion depth)
   SeatKind seat{SeatKind::Abut};
   Quaternion childRot{Quaternion::Identity()};   // 6-DOF seam; identity today (see §8)
};

} // namespace model::part
```

An arbitrary station (e.g. a rail button mid-tube) is just a fraction in `[0,1]`; there is no enum to extend.
`SeatKind` stays closed because a rigid axisymmetric stack genuinely expresses only three radial
relationships.

### 2b. Resolved landmark (geometry-derived, never stored)

A part answers, for a fractional station, the live `{z, rOuter, rInner}` in its own `+z=forward` local frame:

```cpp
struct Station {
   double z{0.0};        // axial station, part-local +z-forward (m) = station01 * getLength()
   double rOuter{0.0};   // outer radius at z (m)
   double rInner{0.0};   // inner radius at z (m); 0 for solids
};

// On Part. Base implementation uses getLength() + the radius virtuals below, so symmetric
// parts need NO override. Clamps station01 to [0,1] (see §2d).
virtual Station stationAt(double station01) const;
```

### 2c. Extent profile for diagnostics

```cpp
// On Part. Closed-form per part type. Base returns 0 (geometrically inert).
virtual double radiusOuterAt(double zLocal) const { return 0.0; } // cone tapers; tube const; sphere bulges
virtual double radiusInnerAt(double zLocal) const { return 0.0; } // BodyTube bore; 0 for solids
virtual double axialLength()  const { return getLength(); }       // span is [0, axialLength()] in local +z
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

### 2e. The explicit cone CM-frame adapter (most important correctness detail)

`getCenterMassOffset()` (`Part.h:96`) returns the cone CM in the cone's **mid-length-origin, base-at-+z**
frame: `z_internal = L/2 − hbar`. The resolver works in the **aft-plane-origin, +z-forward** frame, where the
base is at `z=0` and the tip at `z=L`. The transform between them is fixed and stated once, here:

> For the cone, the CM's *forward-frame* axial coordinate is **`cmZ_fwd = L − hbar`** — i.e. `3L/4` forward of
> the base for a solid cone (`hbar = L/4`), `2L/3` for a shell (`hbar = L/3`). Equivalently, the CM sits
> `hbar` aft of the tip. This is unit-tested against a hand-computed value (§7, Phase 2).

For all symmetric parts (`getCenterMassOffset()` returns 0, CM at mid-length), `cmZ_fwd = L/2`. The resolver
computes each child's CM-in-its-own-frame as a single scalar `cmZ_fwd` via a small per-type adapter
`Part::cmStationLocal()`, and **the cone is the only part that overrides it.** No implicit cross-frame vector
addition survives.

### 2f. The link type that replaces `(Part, Vector3)`

`Part.h:326` changes by a single in-slot struct swap (single-vector storage is kept; parallel
`childParts`/`childLinks` arrays were rejected as a desync hazard across `clone()`/`removeChildById`):

```cpp
std::vector<std::pair<std::shared_ptr<Part>, StationLink>> childParts;
```

### 2g. What lives where

| Lives on `Part` | Lives in `Placement.h` (free / POD) |
|---|---|
| `stationAt`, `radiusOuterAt/InnerAt`, `axialLength`, `isSolid`, `cmStationLocal` (virtuals) | `Station`, `StationLink`, `SeatKind`, `Pose` |
| `childParts` (now `pair<ptr, StationLink>`) | `resolvePlacements(...)`, `sweepOverlaps(...)` |
| `addChildPart(child, StationLink)` + transitional `Vector3` overload | `OverlapDiagnostic`, `SolveResult` |

`getCenterMassOffset()` finally gets consumed for its documented purpose — **only** inside the cone's
`cmStationLocal()` adapter, never as the link.

---

## 3. The single resolver

One function is the sole authority for absolute placement. Both `computeCompositeAt` and `RocketMesh::walk`
call it.

```cpp
struct Pose {
   Vector3    origin {Vector3::Zero()};        // part-local-frame origin (aft plane, on axis), in ROOT frame
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
const double childOriginZ = p.z + signedGap(m) - c.z;   // child's aft-plane origin, in parent frame

// Coaxial today: x = y = 0. (Radial seat r is enforced as a CHECK, not a placement — see below.)
const Pose childInParent{ Vector3(0.0, 0.0, childOriginZ), m.childRot /* Identity in 3-DOF */ };
const Pose childInRoot   = parentPose.compose(childInParent);
```

The root is visited at `rootPose` (conventionally identity; the rocket's aft datum at the world origin, `+z`
forward).

**On radial placement (deliberate scope line):** the radial value `r` is used **only** for the seam/overlap
*check*; the pose solve sets `x = y = 0`. That is correct and sufficient for every axisymmetric part QtRocket
has today (tubes, cones, spheres, and fin sets — whose CM is on-axis by symmetry). True off-axis radial
placement (a single rail button, an asymmetric pod) is **deferred to the 6-DOF work**, because placing a part
at `r > 0` is only physically meaningful once `childRot` and a per-part body-frame offset are integrated —
doing it in 3-DOF would store a half-correct placement, exactly the error class this design exists to
eliminate.

### How CM becomes DERIVED (the core fix)

`computeCompositeAt(t)` is rewritten to consume the resolver's *geometric* placement and derive CM from it,
using the explicit per-part CM adapter (§2e):

```cpp
// For each child, from its resolved Pose and its OWN-frame CM station:
//   childCmInRoot = childPose.origin + childPose.orient * Vector3(0, 0, child.cmStationLocal())
//                   (+ child.getCenterMassOffset() radial components, zero for all current parts)
// Pass 1 (mass-weighted CM):  accumulate childMass * childCmInRoot
// Pass 2 (parallel-axis):     d = childCmInRoot - compositeCm; I += childI + childMass*(|d|^2 I - d d^T)
//                             AND (6-DOF) rotate child tensor: I' = R I R^T, R = childPose.orient
```

`cmStationLocal()` is the **only** place `getCenterMassOffset()` is consumed. Placement is geometric
(station-to-station); CM is its derived consequence. The two-pass mass/CM/parallel-axis structure at
`Part.cpp:179-201` is **preserved** — only its *input* (geometric stations from one resolver) changes.

`accumulateAeroAt` threads each child's resolved `childPose.origin.z()` (relative to the root datum) instead
of `axialStation + pos.z()`. `RocketMesh::walk` translates each geometric-centered primitive by the
resolver's `Pose.origin` (and rotates by `Pose.orient` when 6-DOF arrives). **Both consumers, one resolver,
identical numbers.**

### Caching — resolver split out of the mass gate

`computeCompositeAt` is rebuilt on the *mass*-delta gate (`Part.h:288-308`), which fires **every step during
a burn**. Placement must not. Placement gets its **own** structural-dirty-gated cache:

- New member `std::vector<Placed> resolvedCache;` rebuilt by `ensurePlacementCache()` **only when the tree is
  structurally dirty** (a part added/removed, or a geometry/length edit), gated by a *separate*
  `placementDirty` flag set in `addChildPart`/`removeChildById`/geometry setters — **not** by mass change.
- `computeCompositeAt(t)` reads `resolvedCache` (rigid, mass-independent) and only re-weights by `getMass(t)`.
  A burning motor re-runs the cheap mass/CM/parallel-axis arithmetic over **fixed geometry**, never the
  resolver and never the overlap sweep.
- The overlap sweep (§5) runs once per structural change, with the placement resolve — never per ODE step.

---

## 4. Authoring API & the zero-config default

```cpp
// Primary API — replaces addChildPart(child, Vector3):
virtual void addChildPart(std::shared_ptr<Part> child, StationLink link = {});

// Transitional shim (kept through migration so existing tuple call sites + *.qrd tests compile):
//   synthesizes a link reproducing the legacy CM-to-CM station; LOGS a deprecation note. See §7.
[[deprecated]] void addChildPart(std::shared_ptr<Part> child, Vector3 position);
```

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
                             .childStation01 = 1.0,     // coupler fore plane
                             .gap = 0.04,               // insertion depth
                             .seat = SeatKind::NestInBore});
```

**Optional snap-operator verbs** (produce a `StationLink`, never store a pose):

```cpp
StationLink abut       (double gap = 0.0);                 // {0, 1, gap, Abut}   child fore -> parent aft
StationLink nestInBore (double depth);                     // {1, 1, depth, NestInBore}
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
  the axial placement is exact. `getSpan`/`getSweep`/`getTipChord` remain available for the future sectored
  check.

**Layer 3 — DOF accounting (scoped).** In 3-DOF a single axial station-pair fully pins `z` because coaxiality
is implicit; we record a one-axial-constraint invariant. Once 6-DOF is enabled, a single `StationLink` cannot
express "concentric to a third part" (that needs the deferred joint graph, §10), so Layer 3 in v1 is a
**detector that emits a warning** when a child has only one link under free orientation; the *resolution* (a
companion concentricity constraint) is deferred.

**The gate:** if `SolveResult.ok == false`, **both** `computeCompositeAt` (sim throws/logs) and `RocketMesh`
(renders the offender in an error colour + surfaces the diagnostic) refuse the solve. The coupler-through-nose
case is a hard, located failure — never a silent render. The `SolveResult` is produced once per structural
resolve (§3 caching) and cached, so the gate costs nothing per ODE step.

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
            <link seat="NestInBore" parentStation="1.0" childStation="1.0" gap="0.04"/>
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
- **Version gate:** the writer emits `0.2`; the loader's existing `major == "0"` tolerance
  (`DesignSerializer.cpp:186`) already accepts both `0.1` (legacy `<offset>`) and `0.2` (`<link>`). No loader
  gate change needed.

---

## 7. Migration plan (phased, build stays green, sign-correct)

The migration is **analytic and physics-invariant**, gated by a unit test built **first**.

**Phase 0 — invariance gate FIRST.** Before any type changes: add a test that loads every
`tests/data/designs/*.qrd` through the *current* reader and snapshots `getCompositeCm(0)`,
`getCompositeI(0)`, and the resolved axial station of every part in the legacy `+z=forward` CM-to-CM
convention. These snapshots are the ground truth the migration must reproduce — compared against the legacy
reader's own output, not a re-derivation that could share a bug.

**Phase 1 — types & resolver land, both APIs coexist.** Add `Placement.h`, the
`stationAt`/`radius*At`/`cmStationLocal` virtuals (base + 4 concrete parts), the `StationLink` pair storage,
the resolver + sweep, and the `[[deprecated]] addChildPart(child, Vector3)` shim. Rewrite
`computeCompositeAt`/`accumulateAeroAt`/`RocketMesh::walk` to consume the resolver. Build green; the shim
reproduces legacy behaviour:

```
// All in +z = forward. legacyOffset.z is child CM minus parent CM (CM-to-CM).
// Recover the child's aft-plane origin in the parent frame:
childOriginZ_fwd = parentCmStationLocal - childCmStationLocal + legacyOffset.z
//   where *CmStationLocal = cmStationLocal() in the +z=forward local frame (cone uses §2e adapter)
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
sub-test pins the **nose CM** to a hand-computed value (solid cone: CM at `3L/4` forward of the base = `0.225`
for `L = 0.30`) — the guard that the cone frame adapter (§2e) is correct.

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
  oriented*. Deferred deliberately (see §3 rationale).
- A future body-frame integrator multiplies `rootPose` by the rocket attitude quaternion; the same resolver
  yields every part's world pose for aero/thrust. No schema or resolver-signature change.

---

## 9. Worked example — `xl75_multi`'s four relationships, in `+z = forward`

Root = NoseCone "MultiNose" (`L = 0.30`, `baseRadius = 0.0395`) with its **aft plane (base rim) at world
`z = 0`**, tip at world `z = +0.30`. Stations: aft `{z=0, rOuter=0.0395}`, tip `{z=0.30, rOuter≈0}`;
`radiusOuterAt(z) = 0.0395·(1 − z/0.30)`. Solid → `innerCapacityAt(z) = radiusOuterAt(z)` (the skin). Nose CM
(solid) at `cmStationLocal = 3L/4 = 0.225` forward of the base (world `z = 0.225`).

**(1) body's fore plane ABUTS nose's aft plane** — `addChildPart(body)` (defaults, **zero authored
numbers**): `StationLink{parentStation01=0, childStation01=1, gap=0, seat=Abut}`.
`p = nose.stationAt(0) = {z=0, rOuter=0.0395}`; `c = body.stationAt(1) = {z=0.90, rOuter=0.0395}`.
`childOriginZ = 0 + 0 − 0.90 = −0.90`. Body spans world `z ∈ [−0.90, 0]` (it extends *aft*, `-z` — physically
correct: the body hangs below the nose). Layer 1: `|0.0395 − 0.0395| = 0` → clean seam.

**(2) fin root lies on the body's outer surface near the aft end** — `seat=OnSurface`:
`StationLink{parentStation01=0.06, childStation01=0, gap=0, seat=OnSurface}`.
`p = body.stationAt(0.06) = {z=0.054 local → world −0.846, rOuter=0.0395}`. Layer 1: fin `bodyRadius 0.0395 ==
tube OD 0.0395` → OK. The fin root (chord `0.10`) seats axially with its aft end near the body aft. (Radial
seat is the §3-deferred 6-DOF concern; the fin's mass is axisymmetric, so its 3-DOF placement is exact.)

**(3) coupler NESTS inside the fore end of the body** (OD `0.0376` == body ID `0.0376`) — `seat=NestInBore`:
`StationLink{parentStation01=1.0, childStation01=1.0, gap=0.04, seat=NestInBore}`.
`p = body.stationAt(1.0) = {world z=0, rInner=0.0376}` (the body's fore bore mouth, at the nose seam).
`signedGap(NestInBore) = −0.04`. `c = coupler.stationAt(1.0) = {z=0.08}`. `childOriginZ = 0 + (−0.04) − 0.08 =
−0.12` (parent frame). The coupler spans world `z ∈ [−0.04, +0.04]` measured from the bore mouth: its fore
plane sits `0.04` forward, *projecting past the body fore rim toward the nose*. Layer 1: coupler OD `0.0376 ≤
body ID 0.0376` → fits radially.

**(4) the coupler must NOT poke through the tapering nose — how it's PREVENTED.** The coupler's fore `0.04`
lies at world `z ∈ [0, +0.04]` — *forward of the body fore rim, inside the nose region*. Layer 2 envelope
sweep:
- Over `z ∈ [−0.04, 0]`: host is the body bore, `innerCapacity = rInner = 0.0376` → coupler `0.0376 ≤ 0.0376`
  → OK.
- Over `z ∈ [0, +0.04]`: forward of the body fore rim — the host found by the sorted-interval query is the
  **nose cone** (solid). `nose.radiusOuterAt(0.026 local) = 0.0395·(1 − 0.026/0.30) = 0.0361`; at `z=0.04`,
  `0.0395·(1 − 0.133) = 0.0342`. The solid-host rule compares coupler OD `0.0376` against the skin: `0.0376 >
  0.0342` → `OverlapDiagnostic{offender=coupler, host=nose, zWorld=0.04, penetration=0.0034, "coupler OD
  0.0376 exceeds nose skin radius 0.0342 at z=0.04 forward of the body rim (poke-through 0.0034 m)"}`,
  `solve.ok = false`. Both sim and viz refuse.

This is the exact bug the legacy file produced silently: `<offset z="0.48999...">` (CM-to-CM, `+z=forward`)
slid the coupler *forward* through the cone, and both the simulator (`pos.z()` accumulation) and the renderer
(`station + offset`) drew it there — diverging from each other by the cone CM offset *on top of* the shared
overlap error. In the new model the relationship is "nest 40 mm into the body's fore bore", `NestInBore`
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
  graph if real multi-host designs demand it.
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

- `model/parts/Part.h` — storage (`pair<ptr, StationLink>`), virtuals (`stationAt`, `radiusOuterAt/InnerAt`,
  `axialLength`, `isSolid`, `cmStationLocal`), API, new `placementDirty` flag + `resolvedCache`.
- `model/parts/Placement.h` (new) — `Station`, `StationLink`, `SeatKind`, `Pose`, `resolvePlacements`,
  `sweepOverlaps`, `OverlapDiagnostic`, `SolveResult`.
- `model/parts/Part.cpp` — `computeCompositeAt` (consume resolver), `accumulateAeroAt` (thread resolved z),
  `ensurePlacementCache`, and the tuple-destructure sites (`179, 197, 221, 231, 244, 261, 272`) updated to the
  pair.
- `model/parts/ConicalNoseCone.cpp` — `radiusOuterAt`, `cmStationLocal` (the explicit §2e frame adapter),
  `isSolid`.
- `model/parts/BodyTube.{h,cpp}`, `FinSet.{h,cpp}`, `HollowSphere.{h,cpp}` — `radiusOuterAt/InnerAt`,
  `isSolid` (FinSet body-disc-only + scoped-out interference).
- `visualizer/RocketMesh.cpp` — `walk` consumes the resolver's `Pose` (consumer swap, **no axis flip**).
- `model/DesignSerializer.cpp` — `writeOffset` → `writeLink`, `buildPart` reads `<link>`, version `0.2`
  (loader gate already tolerant).
- **Blast radius beyond `Part.cpp`:** `getChildParts()`'s public return type changes from `tuple` to `pair`,
  so every external iterator updates — confirmed call sites: `visualizer/RocketMesh.cpp:466-469`,
  `model/DesignSerializer.cpp:73`, plus CLI/GUI tree-walkers and tests that iterate `getChildParts()`. The
  `[[deprecated]]` shim covers only the *setter*; accessor consumers are migrated explicitly (a
  `std::get<0>` → `.first` / structured-binding rename — mechanical, enumerated, not hidden).
