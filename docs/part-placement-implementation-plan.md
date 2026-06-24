# Part Placement — Station-Pair Backbone with Extent-Based Diagnostics

**Design specification and incremental implementation plan**

Status: design, pre-implementation. Branch `PartPlacement`. The live tree is *pre-refactor*; this
document describes the target and an ordered path from the current code to it.

This document is self-contained. A new engineer should need only this file plus the QtRocket source
tree to (a) understand the design and (b) execute it. It restates the design faithfully from the
authoritative whitepaper (`docs/PartPlacementDesignLatex/`), grounds every code citation in the
current working tree (verified by `grep`, not trusted from drifted spec line numbers), and then gives
a concrete, ordered, incremental implementation plan in which every step is independently buildable,
keeps the build green, has a definition-of-done, and names the exact gating test(s). A comprehensive
enumerated test plan (unit + integration + migration), an expected-numbers quick reference, a
glossary, an invariants checklist, and a live-tree citation audit close the document.

> **One decision the whitepaper does not pin (resolved).** `FinSet::getLength()` is not specified by
> the whitepaper. **Decision (confirmed 2026-06-22): `getLength() = rootChord`** — the chord along `z`
> at the body surface (`FinSet.h:44/62`), the reading consistent with the worked example treating the
> `0.10 m` root chord as the fin's axial extent. It is threaded consistently through the uniform CM
> rule (`cmLocalZ = x_c - L`, `L = rootChord`). The alternative `rootChord + sweep` was considered and
> rejected (it would shift the fin's local CM and diverges from the worked-example arithmetic).

---

## Table of contents

- **Part I — The design, restated**
  - 1. The problem this replaces
  - 2. Design philosophy and the longitudinal convention
  - 3. Core data model — the exact C++ types
  - 4. The single resolver
  - 5. Authoring API and the zero-config default
  - 6. Overlap and constraint diagnostics
  - 7. Serialization to `.qrd`
  - 8. The 6-DOF path — shaped in at zero schema cost
  - 9. Deliberately deferred / not adopted
  - 10. Files this design touches
- **Part II — The incremental implementation plan** (Steps 0–13)
- **Part III — Comprehensive test suite** (T1–T8) and the expected-numbers quick reference
- **Appendix A — Glossary**
- **Appendix B — Live-tree citation audit**
- **Appendix C — Invariants & guarantees (checklist)**
- **Appendix D — Build/test commands**

---

## Part I — The design, restated

### 1. The problem this replaces

A QtRocket rocket is a composite tree of `Part` objects (`model::part::Part`, base of
`ConicalNoseCone`, `BodyTube`, `FinSet`, `HollowSphere` under `model/parts/`). Each node stores its
direct children together with a single relative-position vector. Today, verified at
`model/parts/Part.h:326`:

```cpp
std::vector<std::tuple<std::shared_ptr<Part>, Vector3>> childParts;
```

The doc comment on that member is precise: the `Vector3` is the relative position of the child's
center of mass with respect to the center of mass of this part. Parts are linked **center-of-mass to
center-of-mass (CM-to-CM)**. That single offset is the only spatial relationship the model records;
every downstream quantity — the composite mass properties consumed by the integrator and the geometry
drawn by the visualizer — is derived from it.

**Why CM-to-CM is fragile.** A CM-to-CM link is only trustworthy if a part's CM coincides with its
geometric center. That holds for every longitudinally symmetric part and **fails for the nose cone**,
the part most rockets begin with. A solid right cone's centroid is `L/4` from the base; a thin
conical shell's is `L/3` from the base. QtRocket encodes exactly this in `ConicalNoseCone.cpp:52`:

```cpp
Vector3 ConicalNoseCone::coneCmOffset(double L, bool solid)
{
   const double hbar = solid ? (L / 4.0) : (L / 3.0);
   return Vector3{0.0, 0.0, L / 2.0 - hbar};   // CM is hbar forward of the base, base at +L/2
}
```

So the cone CM is displaced from its mid-length geometric center by `L/2 - hbar` = `L/4` (solid) or
`L/6` (shell). An author who wants a nose base rim to meet a body fore rim must hand-compute the
offset between two centers of mass that sit at different fractions of their lengths. The link records
a *derived* quantity where the author actually knows a *geometric fact* (two rims touch). Every such
offset is a manual calculation, and every manual calculation is a place to be wrong.

> **Key insight.** A CM-to-CM link is only as trustworthy as the assumption that CM coincides with
> the geometric center. That assumption holds for every symmetric part and fails for exactly the part
> most rockets begin with — the nose cone.

**The silent divergence between simulator and visualizer.** The two consumers of the link do not even
agree on what the offset *means*:

- The **simulator** treats it as documented. `Part::computeCompositeAt` (`Part.cpp:169`) accumulates
  each child's contribution at `pos + cc.cm` — the stored offset *plus* the child subtree's own CM
  (`Part.cpp:183`) — then shifts every tensor to the composite CM via the parallel-axis theorem. A
  strictly CM-to-CM computation.
- The **visualizer** assumes geometric centering. `RocketMesh::walk` (the lambda at
  `visualizer/RocketMesh.cpp:427`) builds each primitive *geometrically centered* — `buildCone` places
  the cone symmetrically about mid-length (`RocketMesh.cpp:87`, base at `-L/2`, tip at `+L/2`) — and
  then translates it by an accumulated `station` (`RocketMesh.cpp:466-472`):

  ```cpp
  for(const auto& childPair : part.getChildParts()) {
     const std::shared_ptr<model::part::Part>& child = std::get<0>(childPair);
     const Vector3& offset = std::get<1>(childPair);   // the stored CM-to-CM Vector3
     if(child) self(self, *child, station + offset);    // applied as geometric-center-to-center
  }
  ```

The renderer takes a value *defined* as a CM-to-CM displacement and *applies* it as a
geometric-center translation. For any symmetric part the two interpretations coincide. For a nose
cone they diverge by precisely the cone's `(geometric center − CM)` offset (`L/4` solid), so the
simulator and the viewer silently reconstruct two different rockets from the same file.

**This is real.** The fixture `tests/data/designs/xl75_multi.qrd:19` places a coupler at
`<offset z="0.48999999999999999"/>` relative to its body. Read as CM-to-CM by the simulator and as a
geometric translation by the viewer — and with the nose's own CM-vs-center displacement folded in — the
link produces a coupler that interpenetrates the nose cone. No check fires; the design loads,
simulates, and renders without complaint.

**The fix, in one paragraph.** Inter-part relationships are stated as physical *intent* — "this rim
seats against that rim," "this tube nests inside that bore" — expressed as a pair of fractional axial
stations plus a seat kind, never as an opaque offset. A **single resolver** walks the ownership tree
and produces an absolute, purely *geometric* placement for every part, and *both* the simulator
(`computeCompositeAt`, `accumulateAeroAt`) and the visualizer (`RocketMesh::walk`) consume that one
placement. With exactly one source of absolute position, the two consumers cannot disagree. Center of
mass ceases to be an input and becomes a *derived* consequence of geometry: the resolver places parts
by their edges, and the existing two-pass mass-weighted CM + parallel-axis machinery is re-driven from
those geometric stations. No force-path or integrator change is required, and a path to full 6-DOF
placement is shaped in at zero schema cost.

---

### 2. Design philosophy and the longitudinal convention

#### 2.1 The three non-negotiable rules

1. **Placement is *derived*, recomputed by exactly one resolver.** Absolute coordinates are never
   stored, and the resolved pose is never stored. One function turns intent into geometry; both the
   simulator and the visualizer read its output.
2. **The stored link is physical *intent*** — a feature-to-feature relationship between a station on
   the parent and a station on the child — never an opaque offset. CM falls out as a consequence of
   geometry, not as an authored input.
3. **The common case costs zero authored numbers.** A child attached with no explicit link abuts its
   parent; the trivial nose → body → fins stack requires no coordinates.

Rule 1 is the discipline the codebase already follows for every other derived quantity: the Barrowman
aero contribution is documented as a pure function of geometry, not stored; the CM-vs-middle offset
is marked "NOT CURRENTLY CONSUMED" and recomputed not cached (`Part.h:310`); and composite
mass/CM/inertia are rebuilt lazily behind a dirty flag. Placement is simply the last quantity brought
under the same rule.

> **Rejected — storing the resolved transform.** Resolve once, store the transform, read it back. It
> reintroduces a staleness class that is the *inverse* of today's bug: edit a part's length and its
> stored transform is silently wrong, instead of (today) edit an offset and the geometry is silently
> wrong. Either direction leaves two sources of truth. Keep one source — intent — and re-derive
> geometry on every read, with caching (§4.5) keeping recompute off the per-step hot path.

#### 2.2 The settled longitudinal convention: `+z = forward`

The local origin of **every** part lies on the longitudinal axis at its **fore (forward) plane**, so a
part occupies `z ∈ [-length, 0]`: its fore plane is the origin (`z = 0`), its aft plane is at
`z = -length`. Putting the origin at the *fore* plane (not the aft) is what makes the whole-rocket
datum a recognizable point: the resolver plants the root — the nosecone — at its own fore plane (the
**nose tip**) at the world origin; because every child composes off its parent's origin, the entire
assembly is referenced to that one point. The derived CM therefore comes back **relative to the nose
tip** — a fixed, recognizable geometric datum, not relative to any part's center of mass — and the
rocket extends aft into `-z`.

This is the *cheapest correct* choice because both the renderer and the existing files already use it:

- The visualizer is already `+z = forward`: `buildCone` places the base at `z = -L/2` and the tip at
  `z = +L/2` (`RocketMesh.cpp:87`), the base-cap normal points `-z` ("outward = aft"); `buildTube`
  names its `+z` end "forward" and its `-z` end "aft" (`RocketMesh.cpp:150-156`); the sphere runs from
  a `+z` pole to a `-z` pole. Every primitive the renderer emits treats `+z` as forward.
- Legacy `.qrd` files are already `+z = forward`: in `xl75_multi`, the body sits at negative `z`
  (extending aft) and the coupler at positive `z` relative to the body (forward, toward the nose).

> **Rejected — `+z = aft`.** It would force a whole-axis inversion of the renderer (every cap normal,
> sphere pole, primitive endpoint) *and* a sign-negating migration of every file. Under `+z = forward`,
> the renderer change is a pure consumer swap with no sign inversion, and the file migration is a
> re-expression of the same numbers with no axial flip.

**The per-part-type local frame.** With `+z = forward` fixed, each concrete type's frame is fully
determined. The fore plane is always the origin; the part extends aft to `-length`.

| Part type        | Fore (`z = 0`)        | Aft (`z = -length`)       | Outer radius profile                       |
| ---------------- | --------------------- | ------------------------- | ------------------------------------------ |
| `ConicalNoseCone`| tip, `r = 0`          | base rim, `r = baseRadius`| `radiusOuterAt(z) = baseRadius·(-z/length)`|
| `BodyTube`       | fore plane            | aft plane                 | constant `outerRadius`; bore `innerRadius` |
| `FinSet`         | fore plane            | aft plane (`L = rootChord`)| body-disc `bodyRadius` only (see §6.3)     |
| `HollowSphere`   | `+z` pole             | `-z` pole (`L = 2·rOuter`)| bulging silhouette                         |

#### 2.3 One frame for every part — including the cone

Every part — `ConicalNoseCone` no exception — lives in the single frame above. Achieving that requires
**correcting a latent defect** in the current cone. `coneCmOffset` (`ConicalNoseCone.cpp:52`) reports
the CM in a *reversed* internal frame: cone in `[-L/2, +L/2]` with the base at `+L/2`, so its `+z`
points aft, opposite the convention. This is the only place in the codebase whose `+z` disagrees with
the forward convention.

The fix is to *remove* the reversal, not adapt around it. `coneCmOffset` is corrected to report the
centroid in the same `+z = forward` frame as every other part: the solid cone's CM sits
`hbar = L/4` forward of the aft/base plane (`L/3` for a shell). Expressed relative to the part's
mid-length, as the `cm` member already is, that is `hbar - L/2 = -L/4` (solid). **The cone's
centroidal inertia tensor is unchanged**: it is axisymmetric and invariant under `z → -z`
(`I_xx = I_yy`, and `I_xx = ∫(y²+z²) dm` is unaffected by `z → -z`), so only the CM's sign was wrong.

`FinSet` carries the *same kind* of latent reference defect. `finSetCmOffset` (`FinSet.cpp:55`)
reports the axial mass centroid `x_c` measured **from one end** (the root leading edge), not from
mid-length. It is corrected the same way: report `x_c - L/2`. Its centroidal tensor is likewise
unchanged.

> **No special parts.** With both corrected, the CM of *every* part is read by one uniform rule (§3.6):
> the mid-plane (`z = -L/2`, half a length aft of the fore origin) plus the mid-referenced
> `getCenterMassOffset()` (`Part.h:96`), all in the one shared frame. No `cmStationLocal` adapter, no
> per-type frame override, no cross-frame vector addition in the placement path. This closes the
> historical simulator/visualizer divergence at its root rather than papering over it with an adapter.

The physics tests written around the defects are updated in lock-step (Phase 1 / Step 7). Concretely,
verified against the live tree:

- `NoseConeTests.cpp:74` asserts `off.z() == L/4` (solid) → becomes `-L/4`; `:109` and `:163` assert
  `L/6` (shell) → `-L/6`. The inertia-oracle assertions (`NoseConeTests.cpp:79-119`, the `cmFromApex`
  and transverse-term checks) pass **unchanged** — proof that only the CM sign moved.
- The frame-independent aero anchor "cone CP is `2/3 L` from the tip" (`NoseConeTests.cpp:144-150`,
  the `tipStation` lambda) must still hold after the sign flip: its middle→tip term is re-expressed to
  the corrected frame and re-asserted `== 2/3 L` for both solid and shell.
- `FinSetTests.cpp:105` asserts `off.z() == xc` → becomes `xc - L/2`.
- `AeroTests.cpp:158` derives `coneBaseToCm = Lnose/2 - getCenterMassOffset().z()` and `:172`/`:220`
  read `getCompositeCm`. Under the tip datum the composite CG flips sign relative to the legacy root-own-CM
  datum, so `AeroTests.cpp:220` (currently `getCompositeCm(0).z() > 0`) is re-expressed to assert the
  CG is *aft of the tip* (`z < 0`).

---

### 3. Core data model — the exact C++ types

All new value types and free functions live in a **new** header `model/parts/Placement.h`, namespace
`model::part`. None of these types exist today; the only geometry-adjacent base virtual presently on
`Part` is `getReferenceArea()` (`Part.h:176`, default `0.0`). The virtuals `stationAt`,
`radiusOuterAt`, `radiusInnerAt`, `axialLength`, `isSolid`, and a promoted `getLength()` are *not* on
`Part` today; the discussion below describes the target and flags what must be added.

#### 3.1 `SeatKind` — the closed seat enum

```cpp
namespace model::part {

// How a child seats against its parent. Governs (a) the radial compatibility CHECK
// and (b) the SIGN of the gap (insertion direction).
enum class SeatKind : std::uint8_t {
   Abut,        // rim-to-rim, same radius; gap is a forward standoff (signed +z)
   NestInBore,  // child OD seats inside parent ID; gap is insertion DEPTH (child moves AFT, -z)
   OnSurface    // child seats radially on parent's outer wall; gap is an axial standoff
};

} // namespace model::part
```

`SeatKind` stays **deliberately closed**. A rigid axisymmetric stack expresses exactly three radial
relationships: rim-to-rim of equal radius (`Abut`), one OD inside another's ID (`NestInBore`), and a
part on another's outer wall (`OnSurface`). The enum has two jobs: (a) choose the radial compatibility
check at the seam — equal radii for `Abut`, offender OD against host capacity for `NestInBore`, wall
match for `OnSurface` — and (b) fix the sign of the gap. There is no fourth relationship for the parts
QtRocket models, so leaving room to extend would only invite half-defined cases.

#### 3.2 `StationLink` — the station-pair link (stored intent)

```cpp
// Physical intent for one parent->child attachment. Stations are FRACTIONS of live length,
// resolved against getLength() every read, so editing a length moves the seam.
struct StationLink {
   double   parentStation01{0.0};   // 0 = aft plane, 1 = fore plane, on the PARENT
   double   childStation01 {1.0};   // 0 = aft plane, 1 = fore plane, on the CHILD (default: fore)
   double   gap{0.0};               // metres; meaning per SeatKind (standoff or depth)
   SeatKind seat{SeatKind::Abut};
   Quaternion childRot{Quaternion::Identity()}; // 6-DOF seam; identity today
};
```

`parentStation01`/`childStation01` are dimensionless fractions in `[0,1]` with `0 = aft`, `1 = fore`.
`gap` is in meters; its meaning is selected by `seat`. `childRot` is the per-seam orientation reserved
for 6-DOF (§8); identity today, so present behavior is bit-identical to a pure-translation tree, and
activating 6-DOF costs no schema reshape.

**Gap semantics.** The resolver places a child by `childOriginZ = p.z + signedGap - c.z`, with
`signedGap = +gap` for `Abut`/`OnSurface` and `-gap` for `NestInBore`. The sign of `gap` is a
displacement of the child along `+z`, and its admissible range is part of the seat's contract:

| SeatKind     | `signedGap` | gap = 0          | gap > 0                         | gap < 0                              |
| ------------ | ----------- | ---------------- | ------------------------------- | ------------------------------------ |
| `Abut`       | `+gap`      | flush rim seam   | open standoff forward            | self-intersecting (out of contract)  |
| `NestInBore` | `-gap`      | child at mouth   | nested aft (depth ≥ 0)           | withdrawn from bore (out of contract)|
| `OnSurface`  | `+gap`      | flush on wall    | forward standoff along wall      | aft standoff along wall (valid)      |

`NestInBore`'s gap is a non-negative insertion **depth**. The degenerate signs (`Abut`/`NestInBore`
< 0) are *out-of-contract* inputs the authoring helpers (§5) are built not to produce — distinct from
the radial/envelope violations the diagnostics (§6) detect *between* legitimately-seated parts.

> **Rejected — a rich named-datum enum (`FORE`, `MID`, `AFT`, …) with an escape hatch.** It pays both
> costs at once: too rigid (a rail button partway up a tube has no named datum and forces the escape
> hatch immediately) and too rich (the named edges `FORE`/`MID`/`AFT` are just `1`, `0.5`, `0`). Two
> fractional stations in `[0,1]` subsume the named edges, the arbitrary mid-body station, and the
> escape hatch in one uniform representation with nothing left to extend.

#### 3.3 `Station` — the resolved landmark (derived, never stored)

```cpp
struct Station {
   double z{0.0};      // axial station, +z-forward (m) = (station01 - 1) * getLength()
                       //   station01: 0 = aft (z = -length), 1 = fore (z = 0, origin)
   double rOuter{0.0}; // outer radius at z (m)
   double rInner{0.0}; // inner radius at z (m); 0 for solids
};

// On Part. Base implementation uses getLength() + the radius virtuals; symmetric parts need NO
// override. Clamps station01 to [0,1] (degenerate guard).
virtual Station stationAt(double station01) const;
```

The base `stationAt` maps the fraction to `z = (station01 - 1) * getLength()` (so `z ∈ [-L, 0]`, with
station 1 at the fore origin and station 0 at the aft plane) and queries the outer/inner radius there
via the profile virtuals (§3.4). Because the whole computation goes through those virtuals, every
symmetric part inherits a correct `stationAt` with no override. A part overrides only if its
station-to-radius mapping is not the default profile.

> **`getLength()` is promoted to a base virtual.** There is *no* `getLength()` on `Part` today. It
> exists as a non-virtual accessor on `ConicalNoseCone` (`ConicalNoseCone.h:53`) and `BodyTube`
> (`BodyTube.h:51`); `FinSet` and `HollowSphere` have none. The refactor promotes `getLength()` to a
> base virtual so the shared `stationAt`/`axialLength` can call it polymorphically. The existing
> per-part accessors become overrides; `FinSet` gains one (returning its `rootChord`, `FinSet.h:62` —
> see the inference note in the preamble); `HollowSphere` supplies one returning its diameter
> `2 * outerRadius`. This is added work, not a pre-existing facility.

#### 3.4 The extent profile and the solid-host rule

```cpp
// On Part. Closed-form per part type. Base returns a geometrically inert part.
virtual double radiusOuterAt(double zLocal) const { return 0.0; } // cone tapers; tube const
virtual double radiusInnerAt(double zLocal) const { return 0.0; } // BodyTube bore; 0 for solids
virtual double axialLength()  const { return getLength(); }       // span is [-axialLength(), 0]
```

These are distinct from `stationAt` because overlap detection samples the profile at many stations
across a span (§6), so the per-`z` radius functions must be callable independently. `radiusOuterAt`
returns the tapering skin for a cone, the constant wall for a tube, the bulging silhouette for a
sphere; `radiusInnerAt` returns the bore for a hollow part and zero for a solid; `axialLength`
defaults to the promoted `getLength()` and is the natural override point for `HollowSphere`.

The `HollowSphere` outer silhouette is the one non-trivial profile. With the sphere built from a `+z`
pole at `z = 0` to a `-z` pole at `z = -2·rOuter` (center at `z = -rOuter`), the outer radius at a
local station is

```cpp
// HollowSphere, center at z = -rOuter, span [-2·rOuter, 0]:
radiusOuterAt(z) = sqrt(max(0.0, rOuter*rOuter - (z + rOuter)*(z + rOuter)));
```

peaking at `rOuter` at the equator and going to 0 at both poles.

**The solid-host rule.** A naive "offender OD vs. host *inner* radius" test is wrong for a solid cone:
a solid cone has no bore, so `radiusInnerAt` returns zero everywhere and the naive test would flag
*every* intruder as poking through. Fix it with one predicate that selects the boundary by solidity:

> A host **occupies** radius `[0, hostCapacity(z)]`, where
> `hostCapacity(z) = radiusInnerAt(z)` for a **bored** part (the bore), and `radiusOuterAt(z)` for a
> **solid** part (the outer skin). An offender at station `z` with outer radius `r_off` **fits** iff
> `r_off ≤ hostCapacity(z) + tol`.

For a bored tube the offender must fit inside the bore; for a solid cone it must fit inside the outer
skin — which is exactly the poke-through test. Exposed as one non-virtual helper:

```cpp
double Part::innerCapacityAt(double zLocal) const;  // = hostCapacity(z); branches on isSolid()
```

so call sites in the sweep never branch on solidity themselves. The branch is driven by:

```cpp
virtual bool isSolid() const { return radiusInnerAt(0.0) <= 0.0; }
```

The base default infers solidity from the absence of a bore (correct for a solid cone). Parts with a
wall — `BodyTube` (annular wall, `BodyTube.h:67`) and `HollowSphere` (shell, `HollowSphere.h:89`) —
override `isSolid` to report their wall honestly, so `innerCapacityAt` routes through the bore for
them.

> **Note:** `ConicalNoseCone` already has a *non-virtual* `isSolid()` (`ConicalNoseCone.h:56`). The
> refactor makes the base `isSolid()` virtual; the cone's accessor becomes a `const` override
> returning its `solid` member (which agrees with the base-inferring default: a solid cone's bore is
> zero).

#### 3.5 Degenerate-geometry guards

The profile functions must be total: a malformed or zero-extent part is a load-time diagnostic, not
UB or a divide-by-zero.

- `stationAt` clamps `station01` to `[0,1]`. A link whose fraction falls outside is reported as a
  load-time warning rather than producing an out-of-range station.
- The cone's `radiusOuterAt` guards the taper divide: when `length ≤ 1e-9` it returns `baseRadius` (a
  degenerate zero-length disc/ring) instead of evaluating `baseRadius * (-z/length)` as `0/0`.
- A zero-length part has `axialLength() = 0`, so `stationAt(any) → z = 0`. Its overlap span collapses
  to a single station, which the sweep (§6) handles as a point sample. Documented behavior, not a
  crash.

#### 3.6 Center of mass in the local frame — the uniform rule

Because every part lives in the one fore-plane-origin, `+z = forward` frame (§2.3), CM is read the
same way for all. A part's mid-plane sits at `z = -L/2`, and `getCenterMassOffset()` (`Part.h:96`,
returning the private `cm` member) reports the CM relative to that mid-plane in the same frame. So the
local CM station is one uniform expression:

```
cmLocalZ = -L/2 + getCenterMassOffset().z()
```

evaluated identically for every part type. Because `getCenterMassOffset()` is relative to mid-length,
it is independent of the origin choice — only the `-L/2` term reflects the fore origin. No
`cmStationLocal` virtual, no cone special case. The values fall out directly:

| Part type        | `getCenterMassOffset().z()` | `cmLocalZ = -L/2 + offset.z()`          | `xl75_multi` (concrete)        |
| ---------------- | --------------------------- | --------------------------------------- | ------------------------------ |
| `BodyTube`       | `0` (geometric-center CM)   | `-L/2`                                   | body: `-0.45`; coupler: `-0.04`|
| `HollowSphere`   | `0` (geometric-center CM)   | `-L/2`                                   | —                              |
| `ConicalNoseCone`| `hbar - L/2` (`-L/4` solid) | `hbar - L` (`-3L/4` solid, `-2L/3` shell)| nose (`L = 0.30`): **`-0.225`**|
| `FinSet`         | `x_c - L/2`                 | `x_c - L`  (`L = rootChord`)             | fins: `x_c - 0.10`             |

The solid cone's `cmLocalZ = -0.225` means the CM sits `3L/4` aft of the tip, equivalently
`hbar = L/4` forward of the wide base, where the mass actually concentrates.

> **CM is uniform, not special-cased.** Every part's CM is derived from
> `-L/2 + getCenterMassOffset().z()` in one shared frame. The remedy for both the cone and the fin set
> is the *same*: report CM relative to mid-length in the shared frame, so the codebase carries no
> reversed frame and no non-mid reference. No cross-frame vector addition survives in the composition
> path — closing the simulator/visualizer divergence (§1) at its root, not papering over it.

#### 3.7 The storage swap

The single in-slot storage change on `Part`, at `Part.h:326`:

```cpp
// before: std::vector<std::tuple<std::shared_ptr<Part>, Vector3>> childParts;
std::vector<std::pair<std::shared_ptr<Part>, StationLink>> childParts;
```

Keeping a **single** vector is deliberate.

> **Rejected — parallel `childParts` and `childLinks` arrays.** Every operation that reshapes the
> child list — `clone()`, `removeChildById()`, `addChildPart()` — would have to keep two arrays
> index-aligned; any future edit touching one but not the other silently corrupts the mapping. A
> single vector of `pair<ptr, StationLink>` makes the pointer and its intent inseparable.

The swap is in-slot: storage stays one vector of pairs, the same accessor pattern reads it, and the
composition routines that destructure it (§4) change only their second binding. Because the second
element is no longer consumed as a coordinate but as intent fed to the resolver, the `<tuple>`
dependency the header carries solely for this member can be dropped once the swap lands.

#### 3.8 What lives where

| Lives on `Part` (behavior, polymorphic)                                              | Lives in `Placement.h` (value / free)                            |
| ----------------------------------------------------------------------------------- | ---------------------------------------------------------------- |
| `stationAt`, `radiusOuterAt`/`radiusInnerAt`, `axialLength`, `isSolid` (virtuals)   | `Station`, `StationLink`, `SeatKind`, `Pose`, `Placed`           |
| promoted `getLength()` virtual                                                      | `resolvePlacements(...)`, `sweepOverlaps(...)`                   |
| `childParts` (now `pair<ptr, StationLink>`); non-virtual `innerCapacityAt`          | `OverlapDiagnostic`, `SolveResult`                               |
| `addChildPart(child, StationLink)` + transitional `Vector3` overload                | snap verbs `abut`/`nestInBore`/`seatOnWall`                      |

Virtuals encode per-part geometry and must dispatch on dynamic type → on `Part`. The value types are
pure data and the resolver/sweep are pure functions over a whole tree → in a free header both the
model and the visualizer can include without dragging in the part hierarchy. The transitional
`Vector3` overload coexists with the `StationLink` primary during migration (§ Part II); the present
tree has exactly one overload, `addChildPart(std::shared_ptr<Part>, Vector3)` at `Part.h:239`.

---

### 4. The single resolver

The data model stores only intent. Turning a tree of intents into absolute placements — one transform
per part in a common root frame — belongs to one pure function. There is exactly **one authority for
absolute placement**, and both the simulator's composite-mass code and the visualizer's mesh walker
consume *its* output rather than computing their own. The divergence of §1 — in which `Part.cpp`
threaded a CM-to-CM axial offset while `RocketMesh.cpp` threaded a geometric-center offset — is removed
by construction, because neither consumer is permitted to invent its own placement.

#### 4.1 `Pose` and `Placed`

```cpp
struct Pose {
   Vector3    origin {Vector3::Zero()};        // fore-plane origin, on axis, in ROOT frame
   Quaternion orient {Quaternion::Identity()}; // (x,y,z,w); identity in 3-DOF

   Pose compose(const Pose& childInThis) const {
      return Pose{ origin + orient * childInThis.origin,
                   (orient * childInThis.orient).normalized() };
   }
};

struct Placed { const Part* part; Pose pose; }; // deterministic DFS (attachment) order
```

`compose` rotates the child translation by the parent orientation *before* adding it — the exact
rigid-transform composition law. In 3-DOF every `orient` is identity, so `compose` degenerates to
vector addition and the resolved tree is bit-identical to the legacy pure-translation walk. The same
line is correct once orientations are non-trivial (§8); carrying the quaternion from day one costs one
identity multiply per child today and buys a zero-schema path to 6-DOF. A `Placed` is a non-owning
pointer to a part plus its resolved pose, returned in deterministic depth-first attachment order —
giving the sweep a stable ordering and making the resolved sequence reproducible across runs.

#### 4.2 Signature and per-child algorithm

```cpp
// THE resolver. DFS over the ownership tree from rootPose. Pure geometry: no CM, no time, no mass.
std::vector<Placed> resolvePlacements(const part::Part& root, const Pose& rootPose);
```

`rootPose` is conventionally identity — the root's fore plane (the nose tip) at the world origin,
`+z = forward`, rocket extending into `-z`. For each child of an already-placed parent, carrying
`StationLink m`:

```cpp
const Station p = parent.stationAt(m.parentStation01); // parent frame
const Station c = child .stationAt(m.childStation01);   // child frame

// signedGap encodes the seat direction in +z = FORWARD:
//   Abut       => +m.gap   (forward standoff; flush at gap = 0)
//   OnSurface  => +m.gap   (axial standoff along the parent wall)
//   NestInBore => -m.gap   (m.gap = insertion depth >= 0; child seats AFT, -z, into bore)
const double signedGap = (m.seat == SeatKind::NestInBore) ? -m.gap : +m.gap;

// Child's fore-plane origin along the parent axis: line the two stations up, then displace.
const double childOriginZ = p.z + signedGap - c.z;

// Coaxial today: x = y = 0. (Radial r is a CHECK, not a placement -- see 4.3.)
const Pose childInParent{ Vector3(0.0, 0.0, childOriginZ), m.childRot /* Identity in 3-DOF */ };
const Pose childInRoot = parentPose.compose(childInParent);
```

`childOriginZ = p.z + signedGap - c.z` reads as physical intent: the parent station sits at `p.z`; the
child is positioned so its chosen station `c.z` lands there (hence `- c.z`, because the pose locates
the child's *fore plane*, not its chosen station, which sits `c.z` from it); the seat-signed gap then
separates or inserts. The seat kind makes the sign correct without authored negatives: a `NestInBore`
of depth `0.04` and an `Abut` standoff of `0.04` are both written as a plain positive number.

Control flow: plant the root at `rootPose`; for each child in DFS order, sample both stations, run the
Layer-1 radial seam check (§6.2), compute `childOriginZ`, build the local pose, compose into the root
frame, recurse.

#### 4.3 Why radial placement is a check, not a coordinate

The per-child step sets `x = y = 0`: every child is coaxial with its parent. The radial value `r`
(`p.rOuter`, `p.rInner`, `c.rOuter`) is used *only* for the seam/overlap *check* (§6), never to
displace the pose off-axis. This is exactly correct for every part QtRocket models: tubes, cones,
spheres are axisymmetric, and a fin set's mass is on-axis by symmetry, so their physically correct
3-DOF placement *is* coaxial.

> **Rejected — off-axis seating in 3-DOF.** Placing a part at `x = r` without rotating it into a
> body-frame orientation stores a *half-correct* placement: mass moved off the axis but the part not
> oriented to sit against its surface, and no per-part body-frame CM offset applied — the exact class
> of silently-wrong stored placement this design eliminates, relocated from the axial axis to the
> radial. True off-axis seating is deferred to 6-DOF (§8), where `childRot` + a body-frame CM offset
> make an `OnSurface` placement at `x = r` correctly oriented. Until then `r` earns its keep as a
> diagnostic.

#### 4.4 How CM becomes derived

The legacy composite routine consumed the stored CM-to-CM `Vector3` directly: in `computeCompositeAt`
the child CM in the parent frame was `pos + cc.cm` (the two-pass loop at `Part.cpp:179`). The refactor
severs that coupling. Placement is now *geometric* (station-to-station, from the resolver), and CM is
its derived consequence.

**Crucially, the two-pass mass-weighted CM + parallel-axis structure is preserved unchanged in
shape.** Pass 1 still accumulates a mass-weighted sum for the composite CM; pass 2 still shifts every
child tensor to that CM via `parallelAxisTerm` (the helper `f(d) = (d·d) I₃ - d dᵀ` at `Part.cpp:20`,
used twice). What changes is solely the *input*: each child's CM in the root frame is now derived from
its resolved pose and its own-frame CM station:

```cpp
const double  cmLocalZ      = -child.getLength() / 2.0 + child.getCenterMassOffset().z();
const Vector3 childCmInRoot = childPose.origin + childPose.orient * Vector3(0.0, 0.0, cmLocalZ);
   // (+ child.getCenterMassOffset() radial parts, zero for every current part)

// Pass 1 (mass-weighted CM):  weighted += childMass(t) * childCmInRoot
// Pass 2 (parallel-axis):     d = childCmInRoot - compositeCm
//                             I += childI + childMass * parallelAxisTerm(d)
//                             AND (6-DOF) I' = R * I * R^T, R = childPose.orient   (§8)
```

This is uniform precisely because the design admits no special parts (§2.3). With no reversed frame
left, no vector arithmetic crosses a frame boundary unwatched — the same single expression derives the
CM of the cone, the tubes, the sphere, and the fin set alike.

Because the resolver plants the root at its fore origin (the nose tip) at the world origin, every
resolved station, and therefore `getCompositeCm()`, is expressed **relative to the nose tip**. For a
conventional nose-led rocket the whole-body CG comes back as `(0, 0, -d)` with `d > 0` — the CG sits
`d` metres aft of the tip. (The migration invariance gate accounts for this deliberate datum change;
§ Phase 0 / Step 2.)

The aero path is updated in the same spirit. `accumulateAeroAt` previously threaded
`axialStation + pos.z()` in its recursive descent (`Part.cpp:224`). It now threads each child's
resolved `childPose.origin.z()` — the same root-frame station every other consumer sees.
`RocketMesh::walk`, which recursed with `station + offset` (`RocketMesh.cpp:472`) reading the legacy
tuple (`RocketMesh.cpp:466-469`), instead translates each geometric-centered primitive by the
resolver's `Pose.origin` (and, when 6-DOF arrives, rotates by `Pose.orient`).

> **Aero CP datum + a correction the migration makes (updated 2026-06-24).** The datum change moves both
> `cp()` and `cg()` to the nose-tip datum. `cg`, mass, and inertia-about-CM are bit-invariant across the
> refactor. The **static margin `cp() − cg()` is NOT** bit-invariant, however: implementing its test
> surfaced that the *legacy* `cp` was CM-contaminated (the legacy aero walk leaked each part's own CM into
> `cnAlphaXcp/cnAlpha`), so the migrated, CM-cancelling `cp` *corrects* the static margin by ~0.1–2% on
> every fixture. The migrated `cp` is the physically correct, CM-independent value (a CP is a function of
> external shape only). 3-DOF flight is unaffected — `cp` is unused until 6-DOF. So the implementer must
> hold bit-stable: **mass, inertia-about-CM, CG (after the datum shift), resolved stations** — but `cp`
> and the static margin are *corrected*, not preserved. The correction is pinned by
> `NoseConeTest.CompositeCpIsCmIndependentSolidVsShell`.

> **Both consumers, one resolver, identical numbers.** The simulator's composite/aero code and the
> visualizer's mesh walk obtain every part's absolute placement from the *same* call to
> `resolvePlacements`. They no longer each compute an axial station from the stored offset, so they
> cannot disagree: the number the integrator uses to place a part's mass is, by construction, the
> number the renderer uses to draw it. The divergence of §1 is structurally impossible.

#### 4.5 Caching — two gates, two cadences

The composite cache today is gated on **mass change**. `ensureCompositeCache` (decl `Part.h:288`)
rebuilds when the structure is dirty *or* the composite mass has moved since the last build, keyed on
`builtAtCompositeMass` (`Part.h:308`). Correct for mass: while a motor burns, the composite mass
differs every step, so the mass-delta gate fires every step — exactly when CM and inertia need
re-weighting.

But geometry does **not** change during a burn. Station landmarks, radii, seat gaps, and every
resolved `Pose` are rigid functions of part lengths and seat intents; invariant under mass loss.
Re-running the full DFS (with quaternion compositions and the overlap sweep) every ODE step would be
pure waste, and during the heavy full-ladder flight sweeps it would dominate.

The design therefore splits placement into its **own** cache, gated on **structural** change:

> **Two gates, two cadences.** `resolvedCache` (a `std::vector<Placed>`) is rebuilt by
> `ensurePlacementCache` *only* when `placementDirty` is set — by `addChildPart`, `removeChildById`, or
> a geometry/length edit — and *never* by mass change. `computeCompositeAt(t)` reads the
> mass-independent `resolvedCache` and re-weights only by `getMass(t)`. A burning motor re-runs the
> cheap mass/CM/parallel-axis arithmetic over *fixed geometry*, while the resolver and the overlap
> sweep run once per structural change. The mass-delta gate fires every step; the placement gate fires
> only when the rocket is actually re-built.

The present tree has a single dirty flag `needsRecomputing` (`Part.h:322`) propagated upward by
`markAsNeedsRecomputing` (`Part.h:294`). The refactor adds a **separate** `placementDirty` flag and a
`resolvedCache` member. `placementDirty` must propagate **up** the ownership chain exactly like
`needsRecomputing` (a child added deep in the tree changes the resolved tree of every ancestor), and a
structural edit sets *both* flags.

> **Precise gate semantics — easy to get backwards.** A pure mass/inertia edit must *not* set
> `placementDirty`. The existing `setMass` (`Part.h:84`) and `setI` (`Part.h:88`) call
> `markAsNeedsRecomputing()` only; under the refactor they continue to set `needsRecomputing` but
> **not** `placementDirty`, because geometry is unchanged. Only structural/geometry edits set both.
> This is the operational counterpart of "derive, don't store": resolved poses are derived (never
> serialized, never authored) but also memoized against the correct invalidation key.

---

### 5. Authoring API and the zero-config default

#### 5.1 The primary call and the transitional shim

Today the only attachment entry point is `addChildPart(std::shared_ptr<Part> child, Vector3 position)`
(`Part.h:239`), whose `Vector3` is the CM-to-CM offset. The refactor replaces it with a link-valued
overload and keeps the old one alive, deprecated, only through the migration window:

```cpp
// Primary API. A default-constructed StationLink means "abut aft", so the trivial stack
// call carries no authored geometry at all.
virtual void addChildPart(std::shared_ptr<Part> child, StationLink link = {});

// Transitional shim, retained only through migration so tuple call sites and every legacy *.qrd
// fixture still compile. It synthesizes a StationLink reproducing the legacy CM-to-CM station and
// logs a deprecation note; removed once the corpus is cut over (Phase 3 / Step 12).
[[deprecated]] void addChildPart(std::shared_ptr<Part> child, Vector3 position);
```

The new overload is a strict superset of the old: a default-constructed `StationLink` encodes the most
common relationship, so migration is additive. The shim is `[[deprecated]]`, not deleted — a hard
deletion would break the build the instant the type changes; the attribute keeps the tree green while
emitting a compile-time warning at each unported call site, turning migration into a compiler-maintained
punch list. The shim's body is **not** a behavioral synonym for the new call — it reconstructs the
equivalent `StationLink` from the legacy CM-to-CM offset using the same uniform local-CM derivation as
the resolver (the recovery formula in Step 8).

#### 5.2 The zero-config default

A default-constructed `StationLink{}` carries `parentStation01 = 0` (parent aft plane),
`childStation01 = 1` (child fore plane), `gap = 0`, `seat = SeatKind::Abut`. In `+z = forward`: **the
child's fore plane meets the parent's aft plane** — a newly attached child stacks aft of its parent.

> **The default is the build order.** A bare `parent->addChildPart(child)` abuts the child's fore
> plane to the parent's aft plane — exactly the natural nose → body → … assembly order. The stack grows
> aft from the nose tip without the author writing a single coordinate.

This is correct precisely because the convention was settled once: `parentStation01 = 0` is *always*
the aft plane, for every part type, no per-type special case. The default is a direct reading of the
data model, not a heuristic to remember.

The three `xl75_multi` relationships as authoring calls:

```cpp
// (1) Body fore plane abuts nose aft plane -- ZERO authored numbers.
nose->addChildPart(body);

// (2) Fin root seats on the body's outer wall near the aft end.
body->addChildPart(fins, {.parentStation01 = 0.06, .childStation01 = 0.0,
                          .seat = SeatKind::OnSurface});

// (3) Coupler nests into the body's fore bore (insertion depth 40 mm).
body->addChildPart(coupler, {.parentStation01 = 1.0, .childStation01 = 0.0,
                             .gap = 0.04, .seat = SeatKind::NestInBore});
```

Designated initializers carry their own documentation; an omitted field falls back to its zero-config
value, so the `OnSurface` fin call need not restate `gap = 0` and the abut call states nothing at all.

#### 5.3 Snap-operator verbs

For authors who prefer named operations, a small set of free verbs each *produces* a `StationLink` by
value — none stores a pose, mutates a part, or fixes a coordinate:

```cpp
StationLink abut       (double gap = 0.0);        // {0, 1, gap, Abut}  child fore -> parent aft
StationLink nestInBore (double depth);            // {1, 0, depth, NestInBore}  child aft inserted
StationLink seatOnWall (double parentStation01);  // {parentStation01, 0, 0, OnSurface}

body->addChildPart(coupler, nestInBore(0.04));    // identical in effect to the brace form
```

> **Verbs name intent; they do not resolve it.** A snap verb is a pure factory for a `StationLink`. It
> reads no length, computes no `z`, returns the same kind of stored intent a brace initializer would.
> All resolution from intent to absolute placement happens in the one resolver. Because the verbs
> touch no part, calling one before its operands are sized, or reusing its result across designs,
> cannot bake in a stale coordinate. They add no expressive power over the brace form; they exist so an
> author may write a relationship as a verb when that reads more clearly.

---

### 6. Overlap and constraint diagnostics

Deriving CM from intent removes one class of error (sim/viewer can no longer disagree). It does not
remove the other: a relationship can be geometrically *consistent* yet physically *impossible* — a
coupler told to nest 40 mm into a 30 mm bore, a tube seated against a rim of the wrong radius, a part
nested near a tube's fore end projecting past it into the nose. The resolver places all of these (the
arithmetic is well-defined); the result is a self-intersecting solid that is meaningless as a rocket.
The diagnostics layer refuses such designs with a *located* reason.

Three escalating layers (cheapest first) feed a single verdict, computed **once per structural
resolve** and cached alongside the placement cache (§4.5), so the gate that protects the per-step ODE
loop costs nothing per step.

#### 6.1 The verdict structures (in `Placement.h`)

```cpp
struct OverlapDiagnostic {
   Part::Id    offender;
   Part::Id    host;          // the part it intrudes into (may be a non-tree neighbour, e.g. the nose)
   double      zWorld;        // located station of worst violation (root frame, +z fwd)
   double      penetration;   // metres the offender radius exceeds host capacity
   std::string message;
};

struct SolveResult {
   bool ok{true};
   std::vector<OverlapDiagnostic> diagnostics;
};
```

`offender`/`host` are `Part::Id` (`std::uint64_t`, `Part.h:56`), the stable per-instance identifier
from `getId()` (`Part.h:191`). Every instance, including a `clone()` copy, carries a distinct id
(re-minted on copy via `makePartId()`, `Part.cpp:29`/`:37`/`:60`) that never changes once assigned, so
attribution is unambiguous even when two parts share a human-facing name (names need not be unique,
`Part.h:193`). Identifying by id rather than by name or pointer is deliberate: a name-keyed diagnostic
could point at the wrong part; raw pointers would dangle if the tree were edited between resolve and
report.

#### 6.2 Layer 1 — radial seam check, on every resolve

Runs as each child is placed, the moment its `StationLink` is resolved against its parent: a single
radius comparison between the two mated stations, dispatched on `SeatKind`. Using `xl75_multi`:

- **`Abut`**: the two outer radii must match, `|p.rOuter - c.rOuter| ≤ tol`. Nose base rim `0.0395`
  equals body fore rim `0.0395` → clean (`|0.0395 - 0.0395| = 0`). ✓
- **`OnSurface`**: the child's outer radius must equal the parent's outer radius at the seat station —
  the fin's `bodyRadius` `0.0395` against the tube OD `0.0395`. ✓
- **`NestInBore`**: the child's outer radius must fit inside the parent's bore,
  `c.rOuter ≤ p.rInner + tol`. Coupler OD `0.0376 ≤` body ID `0.0376` → fits. ✓

Layer 1 is local and `O(1)` per attachment. It catches mating mismatched rims and nesting an
over-wide coupler at the seam, before any global query, at essentially no cost. What it cannot see is
interference between parts not directly mated — that is Layer 2.

#### 6.3 Layer 2 — envelope sweep, after the whole tree resolves

A concrete one-dimensional spatial query over the fully resolved tree. Once `resolvePlacements` has
produced its `std::vector<Placed>` in deterministic DFS order, the sweep (`sweepOverlaps`) builds a
world-frame axial interval `[z_aft, z_fore]` for every part with a non-trivial envelope and asks, at
every feature breakpoint, whether any part's outer radius exceeds the radial capacity available to it.

**1. Build and sort intervals.** From each `Placed` pose, the local span `[0, axialLength]` maps to a
world interval `[z_aft, z_fore]`. Sort by `z_aft` in **`O(N log N)`**. Sorting once lets every host
lookup be a sweep over an active set rather than a quadratic scan — it matters because the sweep is the
most expensive layer and must remain comfortably sub-quadratic.

**2. Sample at feature breakpoints and find hosts.** Each offender's world span is sampled not at
uniform steps but at *feature breakpoints*: its own two endpoints, plus every other interval endpoint
falling within its span. This is **exact**, not approximate, because each part's radius profile `r(z)`
is closed-form and monotone (cone tapers linearly, tube constant, sphere bulges), so a thin protrusion
cannot slip between samples. At each sample `z_world`, candidate hosts are the active set (intervals
covering that station), advanced through the sorted list. The test at each candidate:

```
offender.radiusOuterAt(z_local) > host.innerCapacityAt(z_local) + tol
```

applying the **solid-host rule** (§3.4). `innerCapacityAt` returns the bore for a bored host and the
outer skin for a solid, so the single predicate is correct for both.

**3. Deterministic tie-break.** When more than one interval covers a sample station, the host is the
covering part with the **smallest `Part::Id`**, excluding the offender itself **and the offender's own
ancestor chain through the seat it legitimately occupies**. Because ids are assigned at construction
and re-minted deterministically in DFS load order, this tie-break is reproducible across runs and
reloads — a property required if a reported violation is to be debuggable.

> **Why the sweep walks into a non-tree neighbour.** The host need not be the offender's parent or any
> ancestor. The worked coupler is nested at the body's fore end with a `0.04 m` depth so its fore plane
> projects forward *past* the body fore rim. Over `z ∈ [-0.30, -0.26]` the offender lies forward of the
> body and inside the nose region; the sorted-interval query finds the host to be the **nose cone** — a
> non-tree neighbour the coupler is never linked to. The solid-host rule evaluates the nose's tapering
> skin at each station; coupler OD `0.0376` exceeds nose skin `0.0342` at `z = -0.26`, and the forward
> poke-through is flagged. The global sweep sees the collision precisely because it does not restrict
> itself to the ownership tree.

**FinSet, handled honestly.** A `FinSet` is not axisymmetric, so it has no scalar outer radius the
sweep could meaningfully consume. For *axial* occupancy it contributes only its body-disc radius —
`getBodyRadius()` (`FinSet.h:67`), the same disc it reports as its reference area (`FinSet.h:72`) — so
it hosts nothing inside it and is not falsely modeled as a `bodyRadius + span` disc that would collide
with every adjacent tube. Radial fin-to-fin and fin-to-tube interference (the `span` extent occupying
azimuthal sectors) is **explicitly scoped out of v1** and documented as unchecked.

> **Rejected — a full `bodyRadius + span` disc for fins.** Reporting a fin set's outer radius as
> `getMaxRadius() = bodyRadius + span` (`FinSet.h:73`) would let the existing sweep "check" fins for
> free, but dishonestly: the tip extent occupies only a few azimuthal sectors, not a full disc, and
> modeled as a solid disc it would false-positive against every body tube. Honest sectored interference
> needs a fin clocking/azimuth model QtRocket lacks, so v1 declines to fake it. Layer 1 still verifies
> fin-root radial compatibility, and the fin set's axial placement is exact. The geometry for a future
> sectored check — `getSpan()`, `getSweep()`, `getTipChord()` (`FinSet.h:63-65`) — remains available.

#### 6.4 Layer 3 — DOF accounting

In 3-DOF, a single axial `StationLink` fully pins a child, because coaxiality is implicit (every part
shares the body-frame orientation and sits on the axis, §8). One axial constraint is sufficient, and
the layer records that invariant. Once 6-DOF is enabled, a single `StationLink` cannot express
"concentric to a third part": under free orientation a one-link child has an unpinned rotational DOF.
The full resolution (a companion concentricity constraint) requires the deferred joint graph (§9) and
is out of scope for v1. What v1 *does* provide is the **detector**: Layer 3 emits a warning when a
child has only one link under free orientation, so the missing constraint is surfaced rather than
silently producing an under-determined placement. Cheap, and honest about what it can and cannot fix.

#### 6.5 The gate

> **`SolveResult.ok == false` stops both consumers.** When the diagnostics produce
> `SolveResult.ok == false`, *both* downstream consumers refuse the solve. `computeCompositeAt`
> (`Part.cpp:169`), the routine the simulator drives every ODE step through `ensureCompositeCache`
> (`Part.h:288`), throws and logs rather than returning an inertia computed over a self-intersecting
> solid. `RocketMesh::walk` renders the offender in an error colour and surfaces the diagnostic
> message instead of drawing the impossible geometry. The coupler-through-nose case is a hard, located
> failure in *both* views — never the silent render of the legacy `xl75_multi` fixture.

The gate is correct only because it is cheap. If diagnostics ran per ODE step they would dominate
integration cost; if they ran lazily inside `computeCompositeAt` they would re-fire on every
mass-delta rebuild during a burn (the mass-delta gate fires every step while the motor mass changes).
Neither happens. The `SolveResult` is produced exactly once per structural resolve — when a part is
added/removed or a geometry edit changes the tree — and cached alongside the placement cache. Each
consumer reads the cached boolean. Guarding the inner loop is a single boolean test.

---

### 7. Serialization to `.qrd`

The on-disk format must make the same shift: stop recording resolved geometry, start recording intent.
Today a child's spatial relationship is `<offset x y z/>` (`DesignSerializer.cpp:53`,
`writePart` at `:64`), the serialized CM-to-CM `Vector3`. That triple inherits every pathology of §1:
a resolved coordinate that conflates CM with placement and bakes part lengths into the file so a later
geometry edit silently invalidates it.

#### 7.1 The `<link>` element, kept inline

`<offset>` is replaced, per child, by an inline `<link>` carrying the four authored quantities:
`seat`, `parentStation`, `childStation`, `gap`. Attached to the child `<part>` it governs, exactly
where `<offset>` sat.

> **The link is stored inline with its child, not in a sibling block.** A sibling `<joints>` block
> keyed by part name is a hazard: names need not be unique (`getName()` is non-identifying,
> `Part.h:193`; identity is the unserialized `getId()`). A name-keyed block is ambiguous the moment a
> design has two same-named parts (a permitted, common state). Anchoring each `<link>` to its owning
> child element sidesteps name resolution entirely — the ownership tree in the XML *is* the keying.

Target `0.2` encoding of `xl75_multi` (after migration and fixing the poke-through):

```xml
<QtRocketDesign version="0.2">         <!-- minor bump; loader still accepts 0.1 -->
  <part type="NoseCone" name="MultiNose">
    <params baseRadius="0.0395" length="0.30" wallThickness="0" density="1700" solid="true"/>
    <!-- root: no <link> -->
    <children>
      <part type="BodyTube" name="MultiBody">
        <params innerRadius="0.0376" outerRadius="0.0395" length="0.90" density="1700"/>
        <link seat="Abut" parentStation="0.0" childStation="1.0" gap="0"/>  <!-- = default; omittable -->
        <children>
          <part type="BodyTube" name="MultiCoupler">
            <params innerRadius="0.036" outerRadius="0.0376" length="0.08" density="1700"/>
            <link seat="NestInBore" parentStation="1.0" childStation="0.0" gap="0.04"/>
          </part>
          <part type="FinSet" name="MultiFins">
            <params rootChord="0.10" tipChord="0.04" span="0.06" sweep="0.04" thickness="0.003"
                    bodyRadius="0.0395" finCount="6" density="1700"/>
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

#### 7.2 Writer, reader, version

- **Writer.** `writeOffset` (`DesignSerializer.cpp:53`) becomes `writeLink(const StationLink&)`,
  emitting `seat`/`parentStation`/`childStation`/`gap`. `writePart` (`DesignSerializer.cpp:64`) adds a
  `<link>` sub-element instead of `<offset>`. The `SeatKind` enumerator maps to/from its attribute
  string through a single shared lookup table, so writer and reader agree on the spelling
  (`Abut`/`NestInBore`/`OnSurface`) by construction rather than by two independent switch statements.
  - *A default-equal link is omittable.* Because the zero-config default is a meaningful physical
    relationship, a link whose fields all equal the default carries no information and may be elided;
    the reader reconstructs the same default. This keeps the common stack as terse on disk as in the
    API, and makes the serialized form proportional to the design's complexity, not its part count.
- **Reader.** `buildPart` (`DesignSerializer.cpp:117`) today reconstructs a `Vector3` from
  `offset.<xmlattr>.x/y/z` (defaulting each to `0.0`) and calls the legacy `addChildPart(child,
  Vector3)`. Under the new model it reads the link attributes — `parentStation`, `childStation`,
  `seat`, `gap` — defaulting them to `0.0`, `1.0`, `Abut`, `0`, then constructs a `StationLink{...}`
  and calls `addChildPart(child, StationLink{...})`. Those defaults are exactly the zero-config link,
  so a minimal `<part>` carrying *no* `<link>` still attaches by abutting its parent. The reader
  inherits the existing fail-closed discipline: a `seat` string mapping to no enumerator is a load
  error, raised on the same path as today's `makePart` rejection of an unknown type
  (`DesignSerializer.cpp:127`), so a malformed file is refused, not silently coerced to `Abut`.
- **Version.** The format moves `0.1` → `0.2`. The loader gate (`DesignSerializer.cpp:186`) splits the
  version on the dot and rejects only an unrecognized **major** (`major != "0"`), so it admits any
  minor within major 0. **No loader gate change is needed.** The reader distinguishes the two element
  shapes structurally (by which element is present on the child), not by the version string. The
  current writer hard-codes `"0.1"` (`DesignSerializer.cpp:156`) and `writeOffset` still emits the
  coordinate triple; the `0.2`/`<link>` encoding is the target.

> **Resolved coordinates are never serialized.** The file stores only authored intent — station
> fractions, a gap, a seat kind. The absolute `Pose` of each part is recomputed by the resolver on
> load. A `.qrd` can be edited (lengthen a tube, swap a cone) and reloaded, and the seams move to
> follow the new geometry because nothing pins them to the old. Load is also the natural validation
> point: recomputing placement on load is exactly when the overlap sweep runs against a freshly read
> tree.

---

### 8. The 6-DOF path — shaped in at zero schema cost

The 3-DOF model is a *deliberately soft* restriction. The data model was shaped so the move to 6-DOF
costs **no schema change, no signature change, and no second resolver**. Nothing here is implemented or
exercised in the 3-DOF cut; it is the forward-compatibility argument for the choices already made.

The motivating constraint is in the source: "all parts share the same body-frame orientation, so child
position offsets are pure translations and tensors combine by addition (no rotation) … relative part
rotation is not modeled" (`Part.h:43-45`). The 6-DOF shaping converts this from a structural assumption
baked across the tree into a single localized limitation — one identity-valued quaternion and one
composition line — liftable in place.

- **Orientation already lives in the value types.** `Pose::orient` (§4.1) and `StationLink::childRot`
  (§3.2) both hold `Quaternion::Identity()` in 3-DOF, so behavior is bit-identical to a
  pure-translation tree. `Pose::compose` already rotates the child translation by the parent
  orientation: with identity everywhere, `orient * childInThis.origin` is the input unchanged, so
  `compose` degenerates to vector addition. `childRot` is a *value* in the already-existing struct, not
  a new field — the storage layout does not reshape. Canted fins and side boosters (parts that seat at a
  fixed relative angle) are expressed by a non-identity `childRot`.
- **The one new composition line.** The only structural change is in the composite derivation, not the
  resolver. The two-pass routine is preserved verbatim; 6-DOF adds exactly one operation before a
  child's tensor is shifted to the composite CM: rotate it out of the child body frame into the root
  frame.

  ```cpp
  // In computeCompositeAt's pass 2, per child, with R = childPose.orient:
  const Matrix3 R    = childPose.orient.toRotationMatrix();
  const Matrix3 Irot = R * childI * R.transpose();   // I' = R I R^T  (NEW for 6-DOF)
  const Vector3 d    = childCmInRoot - compositeCm;
  I += Irot + childMass * parallelAxisTerm(d);        // parallel-axis shift, unchanged
  ```

  `I' = R I Rᵀ` is the standard congruence re-expressing a centroidal tensor under a change of body
  orientation. It must precede the parallel-axis shift because `parallelAxisTerm` operates on `d` in
  the root frame; the child tensor must be in that same frame first. In the 3-DOF cut `R = I₃`, so
  `Irot == childI` bit-for-bit and the routine reduces to today's addition. The "no rotation"
  assumption is now localized to this one line.
- **Off-axis radial seating arrives with rotation, not before** (§4.3). It is physically meaningful
  only once `childRot` (so the off-axis part is correctly *oriented*) and a per-part body-frame CM
  offset are present together. The second ingredient is the latent purpose of `getCenterMassOffset()`
  (`Part.h:96`), documented as the natural home for that offset once asymmetric parts or 6-DOF force
  application need it (`Part.h:310`). In 3-DOF only its axial component is consumed (via
  `-L/2 + getCenterMassOffset().z()`); under 6-DOF its radial components, rotated by `childPose.orient`,
  place a genuinely off-axis part's mass correctly.
- **A body-frame integrator reuses the same resolver.** A future attitude-integrating integrator
  supplies the rocket's world orientation by multiplying the `rootPose` passed to
  `resolvePlacements(root, rootPose)` by the integrated attitude quaternion. Because `compose` already
  propagates that root orientation down the entire tree, the same resolver yields every part's full
  world pose. No schema change, no resolver-signature change, no second code path. The 6-DOF work
  reduces to three localized edits: stop forcing `childRot` to identity at authoring/load time, rotate
  the child tensor by the one line above, and feed the integrated attitude into `rootPose`.

---

### 9. Deliberately deferred / not adopted

| Choice                                              | Verdict                          | Why |
| --------------------------------------------------- | -------------------------------- | --- |
| `+z = aft` convention                               | **rejected (sign-inverted)**     | Inverts the axis vs. the renderer and every legacy file; `+z = forward` makes both a re-expression. |
| A stored resolved transform as ground truth         | **rejected (reintroduces staleness)** | Relocates the staleness bug to the output side; edit a length and the stored transform is silently wrong. |
| A rich named-datum enum with an escape hatch        | **rejected (over-engineering)**  | `FORE/MID/AFT` are `1/0.5/0`; two fractions + closed `SeatKind` subsume them with nothing to extend. |
| Parallel `childParts` / `childLinks` vectors        | **rejected (desync hazard)**     | Two arrays must be kept index-aligned across `clone`/`removeChildById`/`addChildPart`. |
| "Every link mandatory"                              | **rejected (punishes the trivial stack)** | The defaulted `Abut` link keeps `nose->addChildPart(body)` a valid, fully specified attachment. |
| Scalar `radiusOuterAt` for `FinSet`                 | **rejected (dishonest)**         | A `bodyRadius + span` disc collides falsely with every tube; honest under-reporting (body disc) instead. |
| Off-axis (`r > 0`) placement in 3-DOF               | **deferred to 6-DOF**            | Seating mass at `x = r` with no orientation is a half-correct stored placement. |
| Full joint graph with sibling `<joints>` block      | **deferred (correct long-term)** | Over-engineering for a rigid inline stack; sibling block keyed by name is ambiguous. |

> **Tree vs. graph — the most consequential "not yet."** A general design models attachments as a
> graph of joint entities serialized in a sibling `<joints>` block keyed independently of the part
> tree. This is the correct long-term chassis: clusters, struts, off-axis pods, and the 6-DOF
> companion-concentricity constraint of Layer 3 all require a part to relate to more than its single
> tree parent. It is deferred for two reasons. First, for a rigid inline stack it is over-engineering:
> every relationship QtRocket authors today is a single parent-to-child seam, captured exactly by the
> inline `std::pair<std::shared_ptr<Part>, StationLink>`. Here **"single" qualifies the *incoming*
> edge, not the fan-out**: `childParts` is a *vector*, so a parent may own many children — branching is
> fully supported — but each child has exactly one parent and one mate, which is the defining property
> of a **tree**. What the deferred cases need is not more *children* but a second *incoming* edge — a
> part bound to more than one host — which closes a loop no tree can hold; that is the qualitative jump
> from a tree to a graph. Second, a sibling block keyed by name is an active hazard given non-unique
> names (`Part.h:193`); keeping the link inline sidesteps it. The inline pair is forward-compatible:
> when real multi-host designs appear, the single `StationLink` per child can widen into a list of
> links — or a separate joint table — without disturbing the resolver's interface or the serialization
> of the common case.

> **Deferred is not dismissed.** The joint graph and off-axis seating are staged, not rejected. The
> data model is shaped so adopting them later is additive: `Pose` already carries a `Quaternion`,
> `compose()` already rotates child translations correctly (the "no rotation" limit at `Part.h:43` is a
> property of today's identity `childRot`, not of the resolver), and the single-link-per-child storage
> can widen to a graph. Each deferral leaves a defined extension point, not a wall.

---

### 10. Files this design touches

The change is **narrow**: a single in-slot storage swap, one new header, and a fixed set of consumer
rewrites. No change to the force path, the integrator, or the state vector. Line numbers verified
against the current tree (branch `PartPlacement`, namespace `model::part`); see Appendix B for the full
audit.

| File | Change |
| ---- | ------ |
| `model/parts/Placement.h` | **New.** `Station`, `StationLink`, `SeatKind`, `Pose`, `Placed`, `resolvePlacements`, `sweepOverlaps`, `OverlapDiagnostic`, `SolveResult`. Plus snap verbs `abut`/`nestInBore`/`seatOnWall`. |
| `model/parts/Part.h` | `childParts` (`:326`) → `vector<pair<shared_ptr<Part>, StationLink>>`. Add geometry virtuals `stationAt`, `radiusOuterAt`, `radiusInnerAt`, `axialLength`, `isSolid`, the promoted `getLength()`, and the non-virtual `innerCapacityAt`. Add the `StationLink` `addChildPart` overload + keep the `[[deprecated]]` `Vector3` shim (`:239`). Change `getChildParts` return type (`:212`). Add `placementDirty` flag and `resolvedCache` member; add `markPlacementDirty` (propagating up like `markAsNeedsRecomputing`, `:294`). Drop `<tuple>` once the swap lands. |
| `model/parts/Part.cpp` | `computeCompositeAt` (`:169`) rewritten to consume the resolver `Pose` and derive CM via `-L/2 + getCenterMassOffset().z()`, preserving the two-pass mass-weighted CM + parallel-axis structure (`parallelAxisTerm` at `:20`). `accumulateAeroAt` (`:214`) threads each child's resolved `Pose.origin.z()` instead of `axialStation + pos.z()` (`:224`). Add `ensurePlacementCache`. Migrate every tuple destructure (see below). |
| `model/parts/ConicalNoseCone.{h,cpp}` | Add `radiusOuterAt` (linear taper `baseRadius * (-z/L)`, with the `L ≤ 1e-9` guard), `radiusInnerAt` (0), `getLength` override, `isSolid` override. **Correct** `coneCmOffset` (`:52`): `L/2 - hbar` → `hbar - L/2`. Tensor unchanged. |
| `model/parts/BodyTube.{h,cpp}` | Add `radiusOuterAt` (constant `outerRadius`), `radiusInnerAt` (constant `innerRadius`, `:67`), `getLength` override (`:51`), `isSolid` override (`innerRadius > 0`). CM already `-L/2`. |
| `model/parts/FinSet.{h,cpp}` | Add `radiusOuterAt` = `getBodyRadius()` (body disc only, `:67`), `radiusInnerAt` (0), `getLength` override (`rootChord`, `:62`), `isSolid` (true). **Correct** `finSetCmOffset` (`:55`): `x_c` → `x_c - L/2`. |
| `model/parts/HollowSphere.{h,cpp}` | Add `getLength`/`axialLength` override (`2 * outerRadius`), `radiusOuterAt` (bulging silhouette `sqrt(rOuter² - (z + rOuter)²)` about its center), `radiusInnerAt` (shell bore, 0 outside the inner band), `isSolid` override (`innerRadius > 0`, `:89`). |
| `visualizer/RocketMesh.cpp` | `walk` (`:427`) consumes the resolver `Pose` instead of `station + offset` (`:472`); destructure at `:466-469` updated to `.first`/`.second`. Pure consumer swap, **no axis flip** (already `+z = forward`, `:150-156`). Error-colour render on `SolveResult.ok == false`. |
| `model/DesignSerializer.cpp` | `writeOffset` (`:53`) → `writeLink`; `writePart` (`:64`) emits `<link>`; `buildPart` (`:117`) reads `<link>`; version `"0.1"` (`:156`) → `"0.2"`. Loader gate (`:186`) unchanged. |
| `model/RocketModel.cpp` | `getChildParts` consumers at `:19` (`findMotorInTree`) and `:177-179` (`addPart`) updated to `.first`/`.second`. `addPart`'s `Vector3 offset` parameter routes through the shim during migration, then is reshaped to a `StationLink` (Step 12). |
| `cli/Repl.cpp` | Tree-walker `getChildParts` consumer at `:223` updated. |
| `visualizer/VisualizerWindow.cpp` | `getChildParts` consumer at `:65` updated. |
| `model/tests/NoseConeTests.cpp`, `FinSetTests.cpp`, `AeroTests.cpp` | Updated to the corrected CM references and the tip datum (§2.3, §7). |
| `model/tests/PartTests.cpp`, `tests/DesignPersistenceTests.cpp` | Updated `getChildParts` destructures (`std::get<0/1>` → `.first/.second`) and CM-datum expectations. |
| `tests/data/designs/*.qrd` | Re-saved to `0.2` with `<link>` (Step 12). Plus a **new** frozen `xl75_multi_pokethrough.qrd` (see Step 12 / T4). |

**The tuple-to-pair destructure sites in `Part.cpp`** (verified — the complete set, *broader* than the
spec's `179, 197, 221, 231, 244, 261, 272`): `clone` (`:117`, including the `emplace_back` at `:121`),
`getCompositeMass` (`:131`), `computeCompositeAt` pass 1 (`:179`), `accumulateAeroAt` (`:221`),
`maxFrontalReferenceArea` (`:231`), `findById` (`:244`), and both loops in `removeChildById` — the
`std::get<0>` direct-child scan (`:263`) and the recursive descent (`:272`). Each must be re-spelled to
bind a `pair<shared_ptr<Part>, StationLink>` (and, where it matters, to read the resolved geometric
station rather than the stored offset).

> **Two corrections to the spec's enumeration.** (1) Line `197` is *not* a `childParts` site: it
> destructures the local `std::vector<std::pair<Vector3, CompositeProperties>>` accumulator (`kids`) in
> `computeCompositeAt` pass 2, unaffected by the storage swap. (2) The spec omits two real sites —
> `clone` (`:117`) and `getCompositeMass` (`:131`). An implementer working from the spec's list alone
> would leave `clone` and `getCompositeMass` broken.

**Blast radius — the public `getChildParts()` return type.** The destructure sites above are private to
`Part`. The change with reach beyond the class is the accessor (`Part.h:212`), which today returns
`const std::vector<std::tuple<...>>&`; changing it to the `pair` vector breaks every external iterator
at compile time.

> **The shim does not cover the accessor.** The `[[deprecated]]` `addChildPart(child, Vector3)` overload
> keeps legacy *setter* call sites compiling through migration. It does **not** shield the *accessor*:
> `getChildParts`'s element type is part of the public contract, and there is no way to return both
> shapes from one signature. Accessor consumers must be migrated explicitly, in lockstep with the
> storage swap, rather than deferred behind the shim.

The confirmed external consumers (verified by grep, broader than the spec's two) — each a mechanical
`std::get<0>` → `.first`, `std::get<1>` → `.second` (or structured-binding) rewrite, **but the *tests*
that assert on the stored `Vector3` must change their assertions, not just rename the access**:

| Consumer | Sites | Kind of change |
| -------- | ----- | -------------- |
| `model/DesignSerializer.cpp` | `:73` (child-writing loop), `:139`, `:141` | mechanical rename (folds into the `writeLink` change) |
| `model/RocketModel.cpp` | `:19` (`findMotorInTree`), `:177`, `:179` (`addPart`) | mechanical rename |
| `cli/Repl.cpp` | `:223` (tree-walker) | mechanical rename |
| `visualizer/RocketMesh.cpp` | `:466` (`std::get<0>`/`std::get<1>` at `:468-469`) | consumer swap to `Pose` |
| `visualizer/VisualizerWindow.cpp` | `:65` | mechanical rename |
| `model/tests/PartTests.cpp` | `:410`, `:433`, `:492`, `:513`, `:515-518` (incl. the `std::get<0>` at `:516`/`:518` and the `PartCompositionAccess` `childAt` friend) | rename + assert on `StationLink` fields |
| `tests/DesignPersistenceTests.cpp` | `:70-83` (incl. the `std::get<1>(...).z() == -0.13` offset assertion at `:83`), `:209`, `:237-239` | **rewrite assertions** from the stored `Vector3` to `StationLink` fields / resolved stations |

---

## Part II — The incremental implementation plan

The migration is structured around one load-bearing claim: the new geometry-driven placement model is
**physics-invariant** on every existing design. The CM, the composite inertia tensor, and every
resolved axial station the legacy CM-to-CM reader produces must be reproduced **bit-for-bit** by the
migrated reader (modulo the one deliberate CG datum shift to the tip). The plan is analytic, not
empirical — no flight re-runs, no tolerance fudging — and gated by a test written and made to pass
**before** any production type changes.

The four migration *phases* of the whitepaper map onto the steps below: **Phase 0** (the invariance
gate, written first) is Step 2; **Phase 1** (types and resolver land, both APIs coexist, the two CM
corrections, the shim) is Steps 1, 3–8; **Phase 2** (run the gate against the migrated reader, hand-pin
the cone) is Step 9; **Phase 3** (cut over the fixtures, surface+fix the poke-through, retire the shim)
is Step 12.

The ordering is deliberately faithful to the whitepaper's "gate-first, correct-within-Phase-1"
discipline: the Phase-0 snapshot is the incumbent's literal output and is **never disturbed**. The two
CM-reference corrections land in Phase 1 (Step 7), and the single resulting datum shift is accounted
for only in the **Phase-2 comparison** (Step 9) — the gate re-expresses the legacy snapshot into the tip
datum by the one known offset rather than the baseline being regenerated. Mass and inertia-about-CM
stay bit-exact throughout.

Each step below is **independently buildable, keeps the build green, has a definition-of-done, and
names its gating test(s)**. After every step, run the fast loop:

```
cmake --build --preset debug-clang
ctest --test-dir build -R 'qtrocket_*' -LE heavy
```

and run `-L heavy` at the milestones noted. Steps marked **[invariance]** are the physics-invariance
spine; do them in order.

### Step 0 — Baseline green

- **Do:** configure and build with the debug-clang preset; run the full suite.
- **DoD:** `ctest --test-dir build -R 'qtrocket_*'` passes on `PartPlacement` before any edit.
- **Gates:** all `qtrocket_*` suites green. This is the reference for "build stays green."

### Step 1 — `Placement.h` value types only (no behavior wired) **[invariance prerequisite]**

- **Do:** create `model/parts/Placement.h` with `SeatKind`, `StationLink`, `Station`, `Pose`,
  `Placed`, `OverlapDiagnostic`, `SolveResult`, and the snap verbs. Add unit tests in a new
  `model/tests/PlacementTypesTests.cpp` registered with the model test binary.
- **Why first:** pure additions; nothing includes the header yet, so the build cannot break.
- **DoD:** the header compiles; `Pose::compose` of identity-with-translation equals vector addition;
  `compose` associativity holds; default `StationLink{}` is `{0, 1, 0, Abut, Identity}`; the snap verbs
  return the expected aggregates.
- **Gating tests:** `PlacementTypesTests.PoseComposeIsTranslationUnderIdentity`,
  `PlacementTypesTests.DefaultLinkIsAbutAft`, `PlacementTypesTests.SnapVerbsReturnExpectedLinks`.

### Step 2 — Phase 0: the invariance gate, written first **[invariance]**

- **Do:** add `tests/PlacementInvarianceTests.cpp` (registered in `tests/CMakeLists.txt`, locating the
  corpus via `QTROCKET_DATA_DIR`/`tests/data/designs`). Load all 24 fixtures (`micro13`→`xl75`)
  through the **current** reader and snapshot, per design, three quantities: `getCompositeMass(0)`,
  `getCompositeI(0)` (both datum-independent), and `getCompositeCm(0)` plus the resolved axial station
  of every part in the existing convention. Write the snapshot to a versioned data file (e.g.
  `tests/data/placement-invariance-baseline.txt`) committed to the tree.
- **Discipline:** the snapshot is taken from the legacy reader's own output, **not re-derived** from
  intended geometry (a re-derivation could share whatever bug the legacy path contains). *Invariance is
  defined by the incumbent, not the spec.* This snapshot is the immutable ground truth; it is never
  regenerated.
- **Datum control:** `getCompositeMass` and inertia *about the CM* are compared bit-for-bit directly.
  `getCompositeCm` is datum-relative and this design *deliberately changes its datum* (legacy: relative
  to the root's own CM; new: relative to the root's fore plane = nose tip). The gate compares the CG
  after re-expressing the legacy snapshot into the tip datum by the one fixed, known offset — the root's
  own CM station `cmLocalZ_root = -L/2 + getCenterMassOffset().z()` (e.g. `-0.225` for a solid
  `L = 0.30` nose). The shim guarantees the per-part CM-to-CM *differences* match, so once that single
  datum shift is applied the CG matches as well. **The baseline itself is never datum-shifted; the
  shift is applied only in the comparison (Step 9).**
- **DoD:** the gate test exists, runs against the *current* code, and passes by definition (it compares
  the snapshot to itself). It is now the ground truth the migration is held against.
- **Gating test:** `qtrocket_integration_tests` includes `PlacementInvariance.*` and is green.

### Step 3 — Promote `getLength()` to a base virtual **[invariance-safe]**

- **Do:** add `virtual double getLength() const;` to `Part`. Make the existing
  `ConicalNoseCone::getLength` (`ConicalNoseCone.h:53`) and `BodyTube::getLength` (`BodyTube.h:51`)
  `override`. Add `FinSet::getLength` (returns `rootChord`, `FinSet.h:62`) and `HollowSphere::getLength`
  (returns `2 * outerRadius`).
- **Why now:** `stationAt`/`axialLength` depend on it; isolating it keeps the next step small. No
  behavior changes — `getLength` was never consumed by the composition math.
- **DoD:** all four concrete types report a length; existing tests unchanged and green.
- **Gating tests:** full `qtrocket_*` suite stays green (no new assertions; this is structural).

### Step 4 — Add geometry/profile virtuals (additive, unconsumed) **[invariance-safe]**

- **Do:** add to `Part`: `stationAt` (base, in terms of `getLength()` + the radius virtuals, clamping
  `station01` to `[0,1]`), `radiusOuterAt`/`radiusInnerAt` (base returns 0), `axialLength` (base
  `getLength()`), `isSolid` (base `radiusInnerAt(0) <= 0`), and the non-virtual `innerCapacityAt`.
  Override the radius/profile virtuals on the four concrete types per §3.4 and §10 (cone taper with the
  degenerate guard; tube constant walls; fin set body disc; sphere silhouette). Make the base
  `isSolid` virtual; `ConicalNoseCone::isSolid` (`ConicalNoseCone.h:56`) becomes a `const` override.
- **Why now:** still additive — nothing consumes these yet, so the build cannot break and the
  invariance gate cannot move.
- **DoD:** the profile functions return the expected closed-form values at known stations (T1);
  degenerate guards return finite values.
- **Gating tests:** `model_tests` gains `GeometryProfileTests.*` (cone taper, tube walls, fin disc,
  sphere silhouette, solid-host capacity, degenerate zero-length); full `qtrocket_*` stays green.

### Step 5 — The resolver and overlap sweep (free functions, unconsumed) **[invariance-safe]**

- **Do:** implement `resolvePlacements(const Part&, const Pose&)` and `sweepOverlaps(...)` in
  `Placement.cpp` (or header-inline) using the geometry virtuals from Step 4.
- **Pragmatic ordering note:** because the storage is still a tuple until Step 6, exercise the resolver
  against trees built *programmatically* with `StationLink`s passed to a temporary helper, not against
  the legacy `childParts`. This keeps Step 5 independent of the storage swap.
- **DoD:** the resolver places the worked `xl75_multi` stack at the expected world origins (nose
  `[-0.30, 0]`, body `[-1.20, -0.30]`, coupler origin `-0.26` spanning `[-0.34, -0.26]`); the sweep
  flags the coupler-vs-nose poke-through with `penetration == 0.0034` at `zWorld = -0.26`.
- **Gating tests:** `ResolverTests.*`, `SweepTests.*` in `model_tests` (T2, T4). Full suite green.

### Step 6 — The storage swap + tuple→pair destructures + accessor migration **[invariance]**

- **Do:** change `childParts` (`Part.h:326`) to `vector<pair<shared_ptr<Part>, StationLink>>` and
  `getChildParts` return type (`Part.h:212`). Add the `StationLink` `addChildPart` overload and the
  `[[deprecated]]` `Vector3` shim (whose body is wired in Step 8). Migrate **every** destructure: the
  internal `Part.cpp` sites (`:117,121,131,179,221,231,244,263,272`) and the external accessor
  consumers (`DesignSerializer.cpp`, `RocketModel.cpp`, `cli/Repl.cpp`, `RocketMesh.cpp`,
  `VisualizerWindow.cpp`, and the tests — see the blast-radius table in §10). Until Step 8 wires the
  shim body, the shim may temporarily forward to a trivial default link so the build links.
- **Why now:** the swap and its mechanical fallout must land together or the build does not compile.
- **DoD:** the whole tree compiles and links; `[[deprecated]]` warnings appear only at intended legacy
  call sites; no tuple `std::get` over `childParts` remains.
- **Gating tests:** `model_tests`, `cli_tests`, `design_matrix_tests` compile and run; the *structural*
  tests (`PartTests.Clone*`, child-list ops) pass. Numeric CM/aero invariance is deferred to Step 9;
  some numeric assertions may be temporarily red until Step 8 — keep them in a separate test filter and
  re-green by Step 9.

> **Build-green hazard and its remedy.** This is the one real ordering trap: the resolver needs a
> `StationLink` but the storage is still a tuple until this step, and the shim that bridges them is not
> wired until Step 8 — so between Step 6 and Step 8 the numeric assertions are transiently red. If
> mid-step numeric redness is unacceptable, **land Step 6, Step 7, and Step 8 together as one atomic
> commit** (the shim body is small); the DoD for the combined commit is the Step 8 DoD. Otherwise keep
> the red numeric assertions in a named filter that is re-greened at Step 9.

### Step 7 — Correct the two CM-reference defects + update their tests **[invariance]**

- **Do:** fix `ConicalNoseCone::coneCmOffset` (`:52`): `L/2 - hbar` → `hbar - L/2`. Fix
  `FinSet::finSetCmOffset` (`:55`): `x_c` → `x_c - L/2`. Leave both centroidal tensors unchanged.
  Update the tests written around the old references in lock-step: `NoseConeTests.cpp:74` (`L/4` →
  `-L/4`), `:109`/`:163` (`L/6` → `-L/6`), the aero anchor `tipStation` lambda (`:144-150`,
  re-expressed so it still asserts `2/3 L`), `FinSetTests.cpp:105` (`xc` → `xc - L/2`), and the
  `AeroTests.cpp` `coneBaseToCm`/`getCompositeCm` expectations (`:158`/`:172`/`:220`).
- **DoD:** `cone.getCenterMassOffset().z() == -L/4` (solid), `-L/6` (shell);
  `fins.getCenterMassOffset().z() == xc - L/2`; the cone's *tensor* assertions
  (`NoseConeTests.cpp:79-119`) still pass unchanged.
- **Gating tests:** `NoseConeTests.*`, `FinSetTests.*`, `AeroTests.*` all green.

### Step 8 — Wire the resolver into the consumers + the shim recovery body **[invariance]**

- **Do:** rewrite `computeCompositeAt` (`Part.cpp:169`) to consume the resolved `Pose` and derive CM
  via `-L/2 + getCenterMassOffset().z()`, preserving the two-pass structure (`parallelAxisTerm`
  unchanged). Rewrite `accumulateAeroAt` (`:214`) to thread `Pose.origin.z()`. Add `ensurePlacementCache`
  driven by `placementDirty`; `computeCompositeAt(t)` reads `resolvedCache` and re-weights by
  `getMass(t)`. Implement the `[[deprecated]]` shim body — the CM-to-CM recovery:

  ```cpp
  // All quantities +z = forward. legacyOffset.z = child CM minus parent CM.
  // 1. Recover the child fore-plane origin in the parent frame. *CmLocalZ = -L/2 + getCenterMassOffset().z()
  //    -- the SAME uniform expression the forward derivation uses, so it inverts cleanly per part.
  childOriginZ_fwd = parentCmLocalZ - childCmLocalZ + legacyOffset.z;
  // 2. Pick a station pair, back the (unsigned) gap out of the resolved geometry.
  gap_unsigned = childOriginZ_fwd + childStation.z - parentStation.z;
  // 3. Infer the seat from the PART PAIR -- BEFORE any sign:
  //      tube fore-rim vs tube aft-rim => Abut;  small-OD inside large-ID => NestInBore; fin on wall => OnSurface
  seat = inferSeat(parent, child);
  // 4. Apply the seat sign LAST. NestInBore inserts aft (-z); others stand off forward (+z).
  gap = (seat == SeatKind::NestInBore) ? -gap_unsigned : +gap_unsigned;
  ```

  Wire `RocketMesh::walk` to consume `Pose.origin`.
- **Two correctness invariants (both easy to get backwards):** (a) `+z` is forward *throughout* — the
  legacy `xl75_multi` coupler `z = +0.49` is read directly as forward, no mirror, no whole-rocket
  inversion. (b) The seat is inferred from the part pair *before* the gap sign is applied, because
  `NestInBore` is the one seat that flips the sign; inferring the seat from the already-signed gap
  would be circular and mis-place every nested coupler.
- **DoD:** all 24 fixtures load through the (still-`<offset>`) reader, flow through the shim, and
  reproduce the Phase-0 snapshot: bit-identical `getCompositeMass(0)` and `getCompositeI(0)`, and
  matching `getCompositeCm(0)` + resolved stations after the one tip-datum shift.
- **Gating tests:** `PlacementInvariance.MassAndInertiaBitIdentical`,
  `PlacementInvariance.CgMatchesUnderTipDatumShift`, `PlacementInvariance.ResolvedStationsMatch`. (A
  planned `StaticMarginBitInvariant` was dropped: it revealed the migration *corrects* a CM-contaminated
  legacy `cp` rather than preserving it — see the Part III as-built note; the correctness is pinned by
  `NoseConeTest.CompositeCpIsCmIndependentSolidVsShell`.) Plus the full
  `model_tests`/`integration_tests`/`cli_tests` green.

### Step 9 — Phase 2: run the gate + the hand-checked CM sub-tests **[invariance]**

- **Do:** run the invariance gate from Step 2 against the migrated reader. Add the dedicated sub-tests
  that pin the off-center parts' CM to **hand-computed** values, independent of both code paths:
  `xl75_multi` solid nose `L = 0.30` → `cmLocalZ = hbar - L = -0.225` (`3L/4` aft of the tip); the
  `xl75_multi` fin set → `cmLocalZ = x_c - L`.
- **Why the off-center parts get their own sub-tests:** a symmetric part has CM at mid-length
  (`cmLocalZ = -L/2`), so a sign slip cannot hide there. The cone and fin set are the only current
  parts whose CM is off-center *and* whose helper formerly carried a wrong reference, so they are the
  parts where a stale-sign mistake could survive the aggregate check yet still be wrong. The
  hand-computed `-0.225` isolates exactly that risk for the cone.
- **Equality within the ULP-drift tolerance** *(amended — see note)* — the original plan called for
  exact `==`, on the premise that the migration is a re-expression of the *same* arithmetic so any
  deviation is a real defect. That premise does not hold in IEEE-754: the resolver re-associates the
  floating-point sums (different composite-walk order, plus the datum shift adds new add/subtracts),
  and FP addition is not associative, so the result is mathematically identical but drifts from the
  legacy bits. The gate therefore compares mass/inertia and `getCompositeCm`-after-datum-shift with a
  relative tolerance `kTol = 1e-9` (`tests/PlacementInvarianceTests.cpp`). The measured worst drift
  across all 24 fixtures is `2.2204460492503131e-16` (**exactly 1 ULP**), so the tolerance is ~7 orders
  of magnitude above the noise floor and far below any real placement bug (which moves a value by
  orders of magnitude more). `ReportWorstDrift` prints the live worst drift every run, so a regression
  that widens it toward the tolerance stays visible even while the assertion passes. Hand-computed
  scalars like `-0.225` use a tight `1e-12`.
  > **Decision (2026-06-23):** keep the `1e-9` tolerance and amend this DoD rather than pursue true
  > bit-identity (which would require restructuring the new arithmetic to match the legacy operation
  > order exactly — fragile, and partly infeasible because the datum shift introduces new ops).
- **DoD:** the gate passes on all 24 fixtures within the documented `1e-9` tolerance (worst observed
  drift = 1 ULP); the cone CM pins to `-0.225` at `1e-12`; the fin CM pins to `x_c - L` at `1e-12`.
- **Gating tests:** `PlacementInvariance.*` (all), `PlacementInvariance.ConeNoseCmHandPinnedMinus0p225`,
  `PlacementInvariance.FinSetCmHandPinned`.

### Step 10 — Serializer: write/read `<link>`, version `0.2` **[invariance-safe]**

- **Do:** replace `writeOffset` with `writeLink` (`DesignSerializer.cpp:53`), have `writePart` emit
  `<link>` (`:64`), `buildPart` read `<link>` (`:117`) with defaults `{0.0, 1.0, Abut, 0}` and the
  shared seat lookup table, bump the writer version to `"0.2"` (`:156`), and elide a default-equal
  link. Keep `<offset>` reading alive (the reader distinguishes by which element is present). Add the
  fail-closed unknown-`seat` error on the `makePart` rejection path (`:127`).
- **DoD:** a design saved at `0.2` round-trips through load; a `0.1` `<offset>` file still loads via
  the shim; an unknown `seat` string is rejected with a clear error; a `0.2` file with no `<link>`
  attaches by abut.
- **Gating tests:** `DesignPersistenceTests.*` (round-trip, deeply nested, unsupported-major-rejected),
  plus new `SerializerLinkTests.LinkRoundTrips`, `SerializerLinkTests.UnknownSeatRejected`,
  `SerializerLinkTests.OmittedLinkAbuts`, `SerializerLinkTests.LegacyOffsetStillLoads`.

### Step 11 — Diagnostics gate wired into both consumers **[invariance-safe]**

- **Do:** compute the `SolveResult` once per structural resolve and cache it alongside `resolvedCache`.
  Make `computeCompositeAt` throw/log on `SolveResult.ok == false`; make `RocketMesh::walk` render the
  offender in an error colour and surface the message.
- **DoD:** a programmatically over-nested coupler design produces `ok == false` with a located
  diagnostic; `getCompositeI(0)` on it throws; a clean design is unaffected and still re-weights every
  step from the cached geometry.
- **Gating tests:** `DiagnosticsGateTests.OverNestedCouplerStopsSolve`,
  `DiagnosticsGateTests.CleanDesignSolves`, `DiagnosticsGateTests.GateDoesNotReFirePerStepDuringBurn`.

  > **Note (2026-06-23) — a sweep false positive surfaced and was fixed here, to keep the [invariance-safe]
  > guarantee.** The plan assumed the shim-loaded corpus was clean except `xl75_multi` (whose poke-through
  > appears only at the Step-12 `NestInBore` cutover — under the shim it is clean). Empirically, two OTHER
  > corpus fixtures, `mid24_multi` and `large38_multi`, resolved `ok == false` today — but on a **false
  > positive**, not a real overlap: a fin set mounted on the body's outer wall, whose root chord overhangs
  > the body aft plane, axially overlaps a sibling AFT coupler of the **same outer radius**; the Layer-2
  > sweep compared the fin's body disc (at `bodyRadius`) against that coupler's **bore**, as if an
  > externally-mounted part had to fit inside it. They are radially separated by the airframe wall — no
  > collision. Wiring a throwing gate without fixing this would have broken `PlacementInvariance` on those
  > two. **Fix:** `sweepOverlaps` now skips the bore-capacity test when the offender's outer radius reaches
  > the (hollow) host's outer skin (`rOff >= hostOuter`) — it is sitting ON/OUTSIDE the host, not nested
  > within it. A **solid** host keeps the poke-through test unchanged (there `cap == hostOuter`, so the
  > guard never trips and `xl75`'s coupler-through-nose still fires). Regressed by
  > `SweepTests.OnSurfaceFinOverCoRadialAftCouplerIsClean`; the fix touches only the `ok`/diagnostics
  > verdict, never mass/inertia/CM, so it is invariance-safe by construction. With it, all 24 fixtures
  > resolve `ok == true` under the shim, so Step 12's "every production `0.2` fixture resolves clean"
  > expectation is unchanged (mid24/large38 were never real poke-throughs; only `xl75_multi` is).

### Step 12 — Phase 3: cut over the fixtures, surface+fix the poke-through, retire the shim **[invariance]**

- **Do:** re-save the corpus to `0.2` (`<offset>` → `<link>`). For `xl75_multi`, the coupler migrates to
  a `NestInBore` link and the Layer-2 sweep **fires**: coupler OD `0.0376` exceeds nose skin `0.0342`
  forward of the body fore rim → `solve.ok == false`. **Fix the production fixture** (reduce insertion
  depth or shorten the coupler) until it resolves clean — do not suppress the diagnostic. Re-run the
  invariance gate against the fresh `0.2` files to confirm the corrected designs are stable under
  reload. Then remove the `[[deprecated]]` `Vector3` `addChildPart` overload (`Part.h:239`), collapsing
  the authoring surface onto the single `StationLink` API; reshape `RocketModel::addPart`'s `Vector3`
  parameter to a `StationLink`. The accessor was already migrated in Step 6 (the shim never covered it).
- **Keep a frozen offender fixture.** Before fixing `xl75_multi.qrd`, copy its un-fixed `NestInBore`
  form into a **separate** `tests/data/designs/xl75_multi_pokethrough.qrd` so the Layer-2 poke-through
  regression keeps exercising the path after the production fixture resolves clean. Without it, once
  `xl75_multi` is fixed nothing else would drive the poke-through diagnostic and the regression would
  silently vanish.
- **DoD:** every production `0.2` fixture resolves `ok == true`; the corrected `xl75_multi` no longer
  pokes through; `xl75_multi_pokethrough.qrd` still resolves `ok == false` with the located diagnostic;
  the deprecated overload is gone; the build is green with no `[[deprecated]]` warnings.
- **Gating tests:** `PlacementInvariance.*` re-run on the `0.2` corpus; `WorkedExampleTests.*` (T4);
  `PokeThroughRegressionTests.UnfixedFixtureStillFires`; full `qtrocket_*` including `-L heavy`.

  > **Note (2026-06-24) — as built.** Two plan premises did not hold and one decision was taken:
  > - **No production poke-through to fix.** Empirically `xl75_multi` is **clean** under the shim and
  >   stays clean after a faithful `0.2` cutover (the corpus re-saves with **zero drift** in mass/CG/
  >   inertia — the cutover is purely `<offset>`→`<link>`). The legacy fixture's recovered links keep the
  >   coupler clear of the nose; the poke-through exists only in the whitepaper's gap=0.04 geometry. So
  >   `xl75_multi_pokethrough.qrd` is **hand-authored** from that whitepaper design (the
  >   `ResolverSweepTests.CouplerPokesThroughNose` geometry), not "copied from an un-fixed xl75_multi."
  >   `WorkedExampleTests` + `PokeThroughRegressionTests` drive it end-to-end through the reader.
  > - **Clean break to 0.2-only (user decision).** The serializer no longer recovers `<offset>`: a legacy
  >   `0.1` file is **rejected with a clear error** (`LegacyOffsetFileIsRejected`). The `Vector3`
  >   `addChildPart` overload, `RocketModel::addPart`'s `Vector3` parameter, and `recoverLink`/`inferSeat`
  >   are all removed; the production authoring surface is `StationLink`-only.
  > - **Test migration.** ~60 call sites moved to `StationLink`. Composition/aero/motor tests that pin
  >   values to a geometric-center-to-center placement use a small **test-only** helper
  >   `model::part::test::cmToCm` (`model/tests/PlacementTestSupport.h`) that re-expresses that intent as
  >   an explicit CM-station `StationLink` (behaviour-preserving, no production shim); clone/structural
  >   tests use `abut(z)`. The CLI `addpart` now places by an abut gap (`z`); the runtime motor attaches
  >   via a CM-station link so its CG contribution is unchanged.
  > - The pokethrough fixture is excluded from the design-matrix enumeration (`designFiles()` skips
  >   `*_pokethrough.qrd`) since it deliberately fails the gate and is not flyable.

### Step 13 — Heavy sweep + final regression

- **Do:** run `ctest --test-dir build -R 'qtrocket_*'` (both `-LE heavy` and `-L heavy`).
- **DoD:** the full ladder and atmosphere sweeps pass; flight numerics are unchanged from baseline
  (the placement refactor is physics-invariant on clean designs; only the deliberately-corrected
  `xl75_multi` differs, and only because its geometry was fixed).
- **Gating tests:** `qtrocket_design_matrix_tests`, `qtrocket_design_matrix_heavy_tests`,
  `qtrocket_integration_tests`, and the whole suite green.

---

## Part III — Comprehensive test suite

Numeric values below are the whitepaper's, verified against the fixtures. **Tolerances:** exact `==`
where the migration is a pure re-expression (mass, inertia about CM, the datum-shifted CG, the static
margin); `1e-12` for hand-computed scalars; `1e-4`/`1e-5` where existing tests already use
approximations (`FinSetTests.cpp:110`, the cone disk-integral oracle `NoseConeTests.cpp:88-92`).

> **As-built (2026-06-24).** T1–T8 were authored incrementally as the Part II gating tests, not as a
> separate phase. A read-only audit reconciled every plan row against the tree; the suite was already
> ~85% present (most rows live under as-built names that differ from the idealized names below — match
> on the *assertion*, not the literal test name). The audit's residual gaps were then closed:
>
> - **Filled (5):** `T5 LinkRoundTrips` (`DesignPersistenceTests.cpp` — the only *direct* check that
>   non-default `NestInBore`/`OnSurface` link fields survive save→load); `T6 CorpusStableAfterCutover`
>   (`PlacementInvarianceTests.cpp` — proves the migrated 0.2 writer is reload-idempotent on the corpus);
>   `T4 Layer1AbutMismatchFlags` (the Abut rim-mismatch path, distinct from the existing NestInBore
>   over-wide test); `T3 TubeAndSphereCmAtMid` (`BodyTubeTests.cpp` — direct symmetric-part CM rule for
>   the tube and the sphere); `T4 TieBreakSmallestId` (`ResolverSweepTests.cpp` — a crafted three-rod
>   multi-cover geometry proving the sweep picks the smallest-id covering host deterministically;
>   mutation-verified to fail if the rule is flipped).
> - **`T6 StaticMarginBitInvariant` — the premise was FALSE; the migration *corrected* `cp` (2026-06-24).**
>   Writing this test surfaced that the static margin `cp() − cg()` is **not** invariant: it changed by
>   ~0.1–2% on **all 24** fixtures. Investigation (algebraic + an empirical solid-vs-shell-cone probe)
>   proved this is a **correction, not a regression**. The legacy aero walk leaked each part's own CM into
>   the CP (`cnAlphaXcp/cnAlpha`), so the legacy `cp` was CM-contaminated; the migrated formula cancels
>   the CM, giving the textbook CM-independent CP (a cone's CP is 2/3 L from its tip regardless of mass).
>   `mass / inertia / CG / resolved stations` remain bit-invariant; 3-DOF flight is unaffected (`cp` is
>   unused until 6-DOF). **Resolution:** the false-premise invariance test was removed; the correctness it
>   should have guarded is now locked by `NoseConeTest.CompositeCpIsCmIndependentSolidVsShell` (solid and
>   shell cones of identical shape ⇒ identical composite `cp` = −2/3 L). The §Part II aero-invariance note,
>   the Step-8 gating list, the §Guarantees summary, and the `Part.cpp` `getCompositeAero` comment were all
>   corrected to state `cp`/static-margin is corrected, **not** legacy-invariant.
> - **`T4 Layer2DiagnosticIsLocated` — realized against the actual message.** The generic envelope sweep
>   reasons over intervals and capacities, not seat relationships, so it cannot (and does not) emit the
>   worked-example narrative "0.04 m forward of the body rim" / "skin". The production message
>   (`Placement.cpp`) is the located form `part <id> (OD …) intrudes into part <id> (capacity …) by …
>   at z=…`; the test asserts that located content (offender/host ids, OD/capacity, penetration, z).
>   The narrative phrasing in the row below is descriptive intent, not a serialized contract.
> - **Obsolete (4) — retired by the Step-12 clean break** (the legacy 0.1 `<offset>` shim was removed):
>   `T5 LegacyOffsetStillLoads` and `T6 ShimSeatInferredBeforeSign` / `ShimNoMirror` / `ShimAbutRecovers`.
>   All four are replaced by `DesignRoundTrip.LegacyOffsetFileIsRejected` (a 0.1 file is now rejected with
>   a clear error, not silently re-placed). Annotated inline below.
> - **Deferred (intentional, tracked):** `T6 Phase0SnapshotIsSelfConsistent` (low value —
>   the baseline is frozen and already diffed bit-for-bit); `T8 TensorRotationIsIdentityUnderIdentityR`
>   (the `R·I·Rᵀ` line is deliberately omitted in 3-DOF, so there is nothing to guard until 6-DOF — a
>   co-located `TODO` at the omitted line in `Part.cpp` ties the test-to-write to the code-to-write).
>   The `GateStopsBothConsumers` visualizer-colour sub-claim is likewise untested (GL render path);
>   the composite-gate throw half is covered by `DiagnosticsGateTests`.

### T1 — Geometry profile unit tests (`model_tests`, Step 4)

| Test | Assertion |
| ---- | --------- |
| `ConeTaperLinear` | `cone.radiusOuterAt(z) == baseRadius * (-z/L)`; at `z = -L` → `baseRadius`, at `z = 0` → 0. For `xl75` nose (`baseRadius 0.0395`, `L 0.30`): `radiusOuterAt(-0.30) == 0.0395`, `radiusOuterAt(-0.26) == 0.0395 * 0.26/0.30 == 0.0342`. |
| `ConeTaperDegenerateGuard` | `length ≤ 1e-9` → `radiusOuterAt(any) == baseRadius` (no `0/0`). |
| `ConeIsSolidAndHasNoBore` | `cone.isSolid() == true`; `cone.radiusInnerAt(z) == 0`; `innerCapacityAt(z) == radiusOuterAt(z)` (solid-host rule). |
| `TubeConstantWalls` | `tube.radiusOuterAt(z) == outerRadius`, `radiusInnerAt(z) == innerRadius` over `[-L, 0]`; `isSolid() == false`; `innerCapacityAt(z) == innerRadius`. For coupler: `radiusInnerAt == 0.036`, `radiusOuterAt == 0.0376`. |
| `TubeSolidRodIsSolid` | `innerRadius == 0` → `isSolid() == true`, `innerCapacityAt == outerRadius`. |
| `FinSetReportsBodyDiscOnly` | `fins.radiusOuterAt(z) == getBodyRadius() == 0.0395`, **not** `getMaxRadius() == bodyRadius + span`. `axialLength() == rootChord == 0.10`. |
| `SphereSilhouetteAndDiameter` | `sphere.axialLength() == 2*outerRadius`; `radiusOuterAt = sqrt(rOuter² - (z+rOuter)²)` peaks at the equator = `outerRadius`, → 0 at the poles; `isSolid()` reflects `innerRadius`. |
| `StationAtClampsAndMaps` | `stationAt(0).z == -L`, `stationAt(1).z == 0`, `stationAt(0.5).z == -L/2`; `stationAt(-0.1)` clamps to `stationAt(0)`, `stationAt(1.1)` to `stationAt(1)` (load-time warning). |
| `ZeroLengthCollapsesToPoint` | `axialLength() == 0` → `stationAt(any).z == 0`; sweep treats it as a point sample, no crash. |

### T2 — Resolver unit tests (`model_tests`, Step 5)

| Test | Assertion |
| ---- | --------- |
| `RootPlantedAtRootPose` | `resolvePlacements(root, identity)` yields the root `Pose.origin == (0,0,0)` (nose tip at world origin); rocket extends into `-z`. |
| `AbutDefaultStacksAft` | body abutted to nose (default link) → body `Pose.origin.z == -0.30`; body spans world `[-1.20, -0.30]`. (`childOriginZ = p.z + 0 - c.z = -0.30 + 0 - 0`.) |
| `NestInBoreSignsGapAft` | coupler `NestInBore`, `parentStation01 = 1.0`, `childStation01 = 0.0`, `gap = 0.04`: `signedGap = -0.04`; in body frame `childOriginZ = 0 + (-0.04) - (-0.08) = +0.04`; composed with body world fore origin `-0.30` → coupler origin world `-0.26`; spans `[-0.34, -0.26]`. |
| `OnSurfaceFinStation` | fins `OnSurface`, `parentStation01 = 0.06`: body-local `z = (0.06-1)*0.90 = -0.846`, world `≈ -1.146`; fin root seats near body aft. |
| `ComposeIsTranslationInThreeDof` | every resolved `Pose.orient == Identity`; resolved origins equal the legacy `station + offset` accumulation for symmetric stacks. |
| `DeterministicDfsOrder` | `Placed` sequence is stable across repeated calls and reloads. |
| `LengthEditMovesSeam` | lengthening the body re-resolves the abutting fin/coupler stations (fractional stations track live length); no stored coordinate goes stale. |
| `CmDerivedFromPose` | for the worked stack, `getCompositeCm(0)` is `(0, 0, -d)`, `d > 0` (CG aft of the tip), in the tip datum. |

### T3 — CM / composite uniform-rule tests (`model_tests`, Steps 7–9)

| Test | Assertion |
| ---- | --------- |
| `SolidConeCmLocalZ` | solid cone `L = 0.30`: `getCenterMassOffset().z() == -L/4 == -0.075`; `cmLocalZ = -L/2 + (-L/4) = hbar - L = -0.225` (hand-pinned, `1e-12`). |
| `ShellConeCmLocalZ` | shell cone: `getCenterMassOffset().z() == -L/6`; `cmLocalZ == hbar - L == -2L/3`. |
| `ConeTensorUnchangedUnderFix` | cone centroidal tensor identical before/after the sign fix (matches `NoseConeTests.cpp:79-119`: `cmFromApex == 0.75 L`, the transverse term unchanged). |
| `ConeAeroAnchorStillTwoThirds` | the frame-independent CP anchor `tipStation == 2/3 L` (`NoseConeTests.cpp:144-150`) still holds for solid and shell after the sign flip. |
| `TubeAndSphereCmAtMid` | `BodyTube`/`HollowSphere`: `getCenterMassOffset().z() == 0` → `cmLocalZ == -L/2`. |
| `FinSetCmLocalZ` | `getCenterMassOffset().z() == x_c - L/2` → `cmLocalZ = x_c - L` (`L = rootChord`). |
| `CompositeCmIsTipRelative` | nose-led rocket `getCompositeCm(0) == (0, 0, -d)`, `d > 0`. |
| `BurningMotorReweightsFixedGeometry` | over a burn, `resolvedCache` is built once (placement gate fires once) while the mass-delta gate re-weights every step; `getCompositeI(t)` shrinks then flattens. |

### T4 — Overlap sweep + worked-example tests (`model_tests` + `integration_tests`, Steps 5, 11, 12)

The `xl75_multi` end-to-end, with exact whitepaper numbers:

| Test | Assertion |
| ---- | --------- |
| `Layer1AbutSeamClean` | nose base rim `0.0395` == body fore rim `0.0395`; `|0.0395 - 0.0395| == 0` → clean. |
| `Layer1AbutMismatchFlags` | rims differing by `> tol` → one `OverlapDiagnostic`, `ok == false`. |
| `Layer1OnSurfaceFinMatch` | fin `bodyRadius 0.0395` == tube OD `0.0395` → clean. |
| `Layer1NestInBoreFits` | coupler OD `0.0376 ≤` body ID `0.0376` → fits (boundary equality within `tol`). |
| `Layer1NestInBoreOverWide` | coupler OD `> body ID + tol` → diagnostic, `ok == false`. |
| `Layer2CouplerBoreSpanClean` | over `z ∈ [-0.34, -0.30]` host is the body bore (`rInner 0.0376`); coupler `0.0376 ≤ 0.0376` → clean. |
| `Layer2CouplerPokesThroughNose` | over `z ∈ [-0.30, -0.26]` the only covering envelope is the solid nose; solid-host rule compares coupler OD vs cone skin: at `z = -0.30`, skin `0.0395` (fits); at the worst station `z = -0.26`, skin `0.0395 * 0.26/0.30 = 0.0342`, coupler `0.0376 > 0.0342` → `penetration == 0.0034` (`1e-12`). |
| `Layer2HostIsNonTreeNeighbour` | the flagged `host` id is the **nose**, not the coupler's parent (the body). |
| `Layer2DiagnosticIsLocated` | `OverlapDiagnostic{offender = coupler, host = nose, zWorld == -0.26, penetration ≈ 0.0034}`; message names the OD, the skin radius, the station, and "0.04 m forward of the body rim". |
| `GateStopsBothConsumers` | `SolveResult.ok == false` → `computeCompositeAt`/`getCompositeI(0)` throws; visualizer renders the offender in error colour. |
| `FixedFixtureResolvesClean` | after reducing the depth/length (Step 12), `xl75_multi` resolves `ok == true`. |
| `OnSurfaceFinNotFalseDisc` | the fin set contributes only its body disc to the sweep; it does **not** false-positive against the adjacent body tube (regression for the rejected `bodyRadius + span` disc). |
| `TieBreakSmallestId` | **DONE (2026-06-24, `ResolverSweepTests.cpp`).** When multiple intervals cover a sample station, the host is the smallest-`Id` covering part excluding the offender and its ancestor chain; reproducible across re-resolves. Crafted three-rod multi-cover geometry; mutation-verified (flipping the rule to largest-id fails the test). |

### T5 — Serialization / `<link>` tests (`integration_tests`, Step 10)

| Test | Assertion |
| ---- | --------- |
| `LinkRoundTrips` | save → load a `0.2` design with `Abut`/`NestInBore`/`OnSurface` links; the reloaded `StationLink`s equal the originals (stations, gap, seat). |
| `OmittedDefaultLinkAbuts` | a `0.2` `<part>` with no `<link>` attaches `{0, 1, 0, Abut}`. |
| `DefaultLinkIsElidedOnWrite` | a default-equal link is not written. |
| `UnknownSeatRejected` | `seat="Wedge"` → load error on the `makePart`-rejection path (fail-closed). |
| `LegacyOffsetStillLoads` | **OBSOLETE (Step-12 clean break — shim removed).** Replaced by `LegacyOffsetFileIsRejected`: a `0.1` `<offset>` file is now rejected with a clear error, not re-placed. |
| `VersionGateUnchanged` | `0.2` passes the `major == "0"` gate (`DesignSerializer.cpp:186`); an unknown major is still rejected. |
| `RoundTripPreservesMassCgStructure` | extends `DesignPersistenceTests` to `0.2`: mass, CG, child count, multi-child order preserved. |

### T6 — Migration / invariance tests (`integration_tests`, Steps 2, 8, 9, 12)

| Test | Assertion |
| ---- | --------- |
| `Phase0SnapshotIsSelfConsistent` | the Phase-0 baseline loads and re-snapshots identically against the current reader (Step 2). |
| `MassAndInertiaBitIdentical` | for all 24 fixtures, migrated `getCompositeMass(0)` and `getCompositeI(0)` equal the Phase-0 snapshot with exact `==`. |
| `CgMatchesUnderTipDatumShift` | migrated `getCompositeCm(0)` equals the Phase-0 CG re-expressed into the tip datum by `cmLocalZ_root` (no extra tolerance). |
| `ResolvedStationsMatch` | every part's resolved axial station equals the legacy station under the same single datum shift. |
| `StaticMarginBitInvariant` | **PREMISE FALSE — removed (2026-06-24).** The static margin is **not** legacy-invariant: the migration *corrected* a CM-contaminated legacy `cp` (changed ~0.1–2% on all 24 fixtures). The correctness it should have guarded — `cp` is CM-independent (shape-only) — is now pinned by `NoseConeTest.CompositeCpIsCmIndependentSolidVsShell`. See the as-built note above. |
| `ConeNoseCmHandPinnedMinus0p225` | `xl75_multi` solid nose `cmLocalZ == -0.225` (hand-computed, independent of both code paths, `1e-12`). |
| `FinSetCmHandPinned` | `xl75_multi` fin set `cmLocalZ == x_c - L` (hand-computed). |
| `ShimSeatInferredBeforeSign` | **OBSOLETE (Step-12 clean break — shim removed).** Replaced by `LegacyOffsetFileIsRejected`. |
| `ShimNoMirror` | **OBSOLETE (Step-12 clean break — shim removed).** Replaced by `LegacyOffsetFileIsRejected`. |
| `ShimAbutRecovers` | **OBSOLETE (Step-12 clean break — shim removed).** Replaced by `LegacyOffsetFileIsRejected`. |
| `CorpusStableAfterCutover` | re-running the invariance gate on the fresh `0.2` corpus (Step 12) confirms corrected designs are stable under reload. |

### T7 — Structural / regression tests (existing suites, Steps 6, 13)

- `PartTests.Clone*` — `clone()` deep-copies the `pair<ptr, StationLink>` element, re-mints ids,
  preserves the link (`Part.cpp:117-123`).
- `PartTests` child-list ops — `addChildPart`/`removeChildById`/`findById` over the pair storage
  (`Part.cpp:254-280`); `placementDirty` propagates up like `needsRecomputing`; `setMass`/`setI` set
  only `needsRecomputing`, **not** `placementDirty`.
- `cli_tests` — `Repl` tree-walker (`cli/Repl.cpp:223`) prints the migrated child list.
- `PhysicsIntegrationTests.*` — flight numerics unchanged on clean designs (vacuum/atmosphere/drag;
  recorded inertia shrinks during a burn then flattens).
- `design_matrix` (both `-LE heavy` smoke and `-L heavy` full ladder) — every motor class flies, proving
  the geometry-driven CM/inertia produces bit-identical flights to the legacy path on every class.

### T8 — 6-DOF shaping tests (forward-compat, optional in v1)

- `PoseComposeRotatesChildTranslation` — with a non-identity `orient`, `compose` rotates then
  translates (the `compose` law); identity reduces to addition.
- `TensorRotationIsIdentityUnderIdentityR` — the `I' = R I Rᵀ` line with `R = I₃` yields `Irot == childI`
  bit-for-bit (guards that enabling 6-DOF later is a no-op in 3-DOF).

### Expected-numbers quick reference (whitepaper)

Every load-bearing value, anchored to the section that defines it and the test that checks it.

| Quantity | Value | Section | Test |
|---|---|---|---|
| Solid-cone local CM station `cmLocalZ` (`L = 0.30`) | **-0.225** (`hbar - L`, `3L/4` aft of tip) | §3.6 | T3 `SolidConeCmLocalZ`, T6 `ConeNoseCmHandPinned…` |
| Solid-cone `getCenterMassOffset().z()` (`L = 0.30`) | **-0.075** (`hbar - L/2`) | §2.3, §3.6 | T3 `SolidConeCmLocalZ` |
| Shell-cone `getCenterMassOffset().z()` | **-L/6** | §2.3, §3.6 | T3 `ShellConeCmLocalZ` |
| Cone centroidal tensor under correction | unchanged (`z`-flip invariant) | §2.3 | T3 `ConeTensorUnchangedUnderFix` |
| Cone CP anchor from tip | `2/3 L` (frame-independent) | §2.3 | T3 `ConeAeroAnchorStillTwoThirds` |
| FinSet `cmLocalZ` | **`x_c - L`** (`L = rootChord`) | §3.6 | T3 `FinSetCmLocalZ`, T6 `FinSetCmHandPinned` |
| `xl75_multi` nose span | `[-0.30, 0]` | §6 worked | T2 `AbutDefaultStacksAft` |
| `xl75_multi` body span | `[-1.20, -0.30]` | §6 worked | T2 `AbutDefaultStacksAft` |
| `xl75_multi` coupler span | `[-0.34, -0.26]` | §6 worked | T2 `NestInBoreSignsGapAft` |
| Coupler `NestInBore` depth / origin | depth `0.04`, origin world `z = -0.26` | §6 worked | T2 `NestInBoreSignsGapAft` |
| Fin `OnSurface` station | body-local `-0.846`, world `≈ -1.146` | §6 worked | T2 `OnSurfaceFinStation` |
| Poke-through penetration | **0.0034 m** at `z = -0.26` | §6.3 | T4 `Layer2CouplerPokesThroughNose` |
| Nose skin at `z = -0.26` vs coupler OD | **0.0342** vs **0.0376** | §6.3 | T4 `Layer2CouplerPokesThroughNose` |
| Clean Abut seam | `0.0395 == 0.0395` | §6.2 | T4 `Layer1AbutSeamClean` |
| Coupler vs body bore | OD `0.0376` ≤ ID `0.0376` | §6.2 | T4 `Layer1NestInBoreFits` |
| Legacy coupler offset (recovered as forward) | `z = +0.49`, no mirror | §8 (Step 8) | T6 `ShimNoMirror` |
| Envelope sweep complexity | `O(N log N)` sort + active-set sweep | §6.3 | (perf/sanity) |
| Composite CG datum | nose tip; nose-led CG `(0,0,-d)`, `d > 0` | §4.4 | T3 `CompositeCmIsTipRelative` |

---

## Appendix A — Glossary

Domain terms as used throughout, in alphabetical order. Concrete constructs cite the live working tree;
types introduced by this design are marked *new*.

- **Abut** — `SeatKind`: a rim-to-rim seat between two equal-radius parts (a tube fore rim meeting a
  tube aft rim). The radial check requires matching outer radii within tolerance; the gap is a forward
  standoff along `+z`, flush at 0. The default seat, so the trivial stack costs no authored numbers.
- **CM-to-CM legacy link** — the current attachment representation: each child stores a `Vector3`
  giving its CM relative to the parent's CM (`Part.h:326`). Because a part's CM is not in general at
  mid-length (a solid cone's is `L/4` from the base), this couples placement to a derived quantity and
  forces authors to hand-compute offsets. Replaced by the geometric `StationLink`.
- **Composite CM / composite inertia** — the CM and inertia tensor of a part plus all descendants,
  cached on each part (`compositeCm` at `Part.h:320`, `compositeInertiaTensor` at `Part.h:300`) and
  recomputed by the two-pass `computeCompositeAt` (`Part.cpp:169`): pass 1 builds the mass-weighted CM,
  pass 2 shifts every child tensor to it via the parallel-axis theorem. The composite tensor is full
  mass-weighted (kg·m²); a part's own `inertiaTensor` is per-unit-mass (m²).
- **Cone reversed-frame defect** — a pre-existing mistake in `coneCmOffset` (`ConicalNoseCone.cpp:52`):
  it reports the cone CM in a mid-length-origin, base-at-`+z` frame, whose `+z` is opposite the chosen
  forward direction. This design corrects it (`L/2 - hbar` → `hbar - L/2`) so the cone reports in the
  shared `+z = forward` frame; the centroidal tensor is unchanged (axisymmetric, `z`-flip invariant).
- **Envelope sweep** — Layer 2 of the diagnostics (`sweepOverlaps`, *new*): after the whole tree
  resolves, build world axial intervals for every part, sort by aft station (`O(N log N)`), then sample
  each offender at feature breakpoints and test its outer radius against each candidate host's capacity
  using the solid-host rule. Runs once per structural change, never per integration step.
- **`getCenterMassOffset`** — the accessor returning a part's CM relative to the component middle
  (`Part.h:96`), zero for symmetric parts and nonzero for a cone or fin set. Documented but unconsumed
  today; this design consumes it through the uniform rule `-L/2 + getCenterMassOffset().z()`.
- **`hostCapacity` / solid-host rule** — the radius a host occupies at a station: `radiusInnerAt(z)`
  for a bored part (the bore) and `radiusOuterAt(z)` for a solid part (the outer skin). An offender of
  outer radius `r_off` fits iff `r_off ≤ hostCapacity(z) + tol`. Resolves the ambiguity of a solid
  part, which has no bore and would otherwise flag everything; makes the coupler-versus-cone test the
  correct poke-through check. Exposed as the *new* non-virtual `innerCapacityAt`.
- **`NestInBore`** — `SeatKind`: the child's outer radius seats inside the parent's bore. The radial
  check requires child OD within parent ID; the gap is a non-negative insertion depth that drives the
  child aft (`-z`), so the signed gap is negated relative to `Abut`.
- **`OnSurface`** — `SeatKind`: the child seats radially on the parent's outer wall at a chosen station
  (a fin root on a body tube). The check compares the child's radius against the parent's outer radius
  at the station; the gap is an axial standoff along the wall.
- **`OverlapDiagnostic`** — a *new* POD recording one located interference: the offender and host
  `Part::Id`s, the world station of the worst violation, the penetration depth in metres, and a message.
  `Part::Id` is `std::uint64_t` (`Part.h:56`).
- **Parallel-axis theorem (`parallelAxisTerm`)** — the displacement tensor `f(d) = (d·d) I₃ - d dᵀ`
  that, scaled by a body's mass and added to its centroidal tensor, shifts that tensor to a parallel
  axis offset by `d` (`Part.cpp:20`). The composite pass applies it to every child; this design
  preserves the math and changes only the input `d` from CM-to-CM offsets to resolver-derived stations.
- **`Part` composite tree** — the ownership tree rooted at `Part` (`model/parts/Part.h`, namespace
  `model::part`). Each node is both a single component and the composite of itself with its children;
  children are stored in `childParts` (`Part.h:326`) and adopted by move via `addChildPart`
  (`Part.h:239`). `clone()` deep-copies and re-mints ids.
- **`placementDirty` vs. the mass-delta gate** — two distinct cache gates. The existing mass-delta gate,
  `ensureCompositeCache` keyed on `builtAtCompositeMass` (`Part.h:288`/`:308`), fires every integration
  step during a burn because the composite mass changes each step. The *new* `placementDirty` flag
  gates the resolver cache on structural change only. Today only `needsRecomputing` (`Part.h:322`)
  exists, propagated upward by `markAsNeedsRecomputing` (`Part.h:294`); the separate `placementDirty`
  and `resolvedCache` are added by this design. `setMass`/`setI` (`Part.h:84`/`:88`) set
  `needsRecomputing` but **not** `placementDirty`.
- **`Placed`** — a *new* value pairing a resolved `Part` pointer with its `Pose`. The resolver returns
  `std::vector<Placed>` in deterministic depth-first attachment order.
- **`Pose`** — a *new* value type carrying an origin (the part-local fore-plane origin in the root
  frame) and an orientation quaternion (identity in 3-DOF). Its `compose` operator chains a child pose
  through its parent, rotating the child translation, so the same routine serves 6-DOF unchanged.
- **`Propagatable`** — the model↔sim bridge interface implemented by `RocketModel`, exposing
  `getForces`, `getTorques`, `getMass(t)`, and `terminateCondition`. Unaffected by this design, which
  changes only how composite quantities are derived, not how the integrator consumes them.
- **Resolver (`resolvePlacements`)** — the *new* single authority for absolute placement: a depth-first
  walk over the ownership tree from a root pose, producing a `Placed` for every part by pure geometry
  (no CM, no time, no mass). Both the simulator (`computeCompositeAt`, `accumulateAeroAt`) and the
  visualizer (`RocketMesh::walk`) consume its output, so the two cannot disagree.
- **SeatKind** — a *new* closed enum of three radial seat relationships (`Abut`, `NestInBore`,
  `OnSurface`) governing the radial compatibility check and the sign of the gap. Closed because a rigid
  axisymmetric stack expresses only these three.
- **`SolveResult`** — a *new* aggregate holding an `ok` flag and the list of `OverlapDiagnostic`s from a
  resolve. When `ok` is false, both consumers refuse the solve; produced once per structural resolve
  and cached, so the gate costs nothing per step.
- **Solid-host rule** — see *`hostCapacity`*.
- **Station landmark** — the *new* `Station` value `{z, rOuter, rInner}` returned by `stationAt` for a
  fractional station: the live axial coordinate and radii in the part's `+z = forward` local frame,
  derived from geometry and never stored.
- **Station / fractional station (`station01`)** — a position along a part's longitudinal axis as a
  fraction in `[0,1]`, where 0 is the aft plane and 1 the fore plane. The absolute coordinate is
  `z = (station01 - 1) * getLength()` (so `z ∈ [-L, 0]`), recomputed on each read so editing a length
  moves the seam. `getLength()` is promoted to a base `Part` virtual by this design; today it exists
  only on `ConicalNoseCone` and `BodyTube`.
- **StationLink** — the *new* stored attachment value replacing the CM-to-CM `Vector3`: a parent
  fractional station, a child fractional station, an axial gap, a `SeatKind`, and a per-seam child
  rotation (identity in 3-DOF). It encodes physical intent rather than a derived coordinate, so the
  storage type becomes `std::vector<std::pair<std::shared_ptr<Part>, StationLink>>`.
- **3-DOF vs. 6-DOF** — the simulator currently integrates only the three translational DOF; orientation
  is carried in `StateData` but not yet integrated. `Part` documents the rigid no-relative-rotation
  assumption (`Part.h:43-45`). This design shapes in the 6-DOF path at zero schema cost — `Pose` carries
  an orientation, `StationLink` carries a per-seam rotation, and one composition line rotates each child
  tensor — while remaining bit-identical to today in 3-DOF.
- **`+z = forward` convention** — the settled longitudinal convention: `+z` points toward the nose tip,
  every part's local origin sits on the axis at its fore plane, and a part occupies `z ∈ [-length, 0]`.
  The fore origin makes the root's origin — the nose tip — the whole-assembly datum, so the derived CM
  is reported relative to the tip. The `+z` direction matches the visualizer and the existing `.qrd`
  files, so the renderer change is a consumer swap and the file migration a re-expression, both with no
  axial sign flip. `+z = aft` was rejected for requiring exactly those inversions.

---

## Appendix B — Live-tree citation audit

Every load-bearing `file:line` verified against branch `PartPlacement` (namespace `model::part`). The
whitepaper's own prose cites some pre-drift line numbers (e.g. `ConicalNoseCone.cpp:49`); the values
below are the **live** ones and supersede them.

**`model/parts/Part.h`**
- `:43-45` — the "all parts share the same body-frame orientation … no rotation" assumption (the 6-DOF
  limit, localized to one line by §8).
- `:56` — `using Id = std::uint64_t` (`Part::Id`).
- `:84` `setMass`, `:88` `setI` — both call `markAsNeedsRecomputing()` only (set `needsRecomputing`, NOT
  `placementDirty`).
- `:96` — `getCenterMassOffset()` returns the `cm` member; `:310` — `cm` "NOT CURRENTLY CONSUMED", the
  home for the 6-DOF/asymmetric offset.
- `:176` — `getReferenceArea()` (the only existing base geometry virtual, default `0.0`).
- `:191` `getId()`, `:193` name "need NOT be unique".
- `:212` — `getChildParts()` returns the tuple-vector const ref (the public accessor to migrate).
- `:239` — sole `addChildPart(std::shared_ptr<Part>, Vector3)`.
- `:288` `ensureCompositeCache`, `:294` `markAsNeedsRecomputing` (propagates up), `:300`
  `compositeInertiaTensor`, `:308` `builtAtCompositeMass`, `:320` `compositeCm`, `:322`
  `needsRecomputing`.
- `:326` — `childParts` is `std::vector<std::tuple<std::shared_ptr<Part>, Vector3>>` (the swap target).

**`model/parts/Part.cpp`**
- `:20` `parallelAxisTerm` (`:200` is its second composite-loop use); `:29`/`:37`/`:60` `makePartId`
  mints a fresh id per ctor (and on clone).
- `:169` two-pass `computeCompositeAt`; `:183` `pos + cc.cm`; `:214` `accumulateAeroAt`, `:224`
  `axialStation + pos.z()`.
- Tuple destructure sites (complete, verified): `:117`/`:121` (`clone`), `:131` (`getCompositeMass`),
  `:179` (`computeCompositeAt` pass 1), `:221` (`accumulateAeroAt`), `:231` (`maxFrontalReferenceArea`),
  `:244` (`findById`), `:263`/`:272` (`removeChildById`).
- `:197` — the *local* `kids` accumulator (`vector<pair<Vector3, CompositeProperties>>`), NOT a
  `childParts` site (the spec's enumeration is wrong here).

**Concrete parts**
- `ConicalNoseCone.cpp:52` — `coneCmOffset` (the reversed-frame defect to correct); `ConicalNoseCone.h:53`
  non-virtual `getLength()`, `:56` non-virtual `isSolid()`.
- `BodyTube.h:51` non-virtual `getLength()`, `:67` wall radii.
- `FinSet.cpp:55` — `finSetCmOffset` (the non-mid-reference defect to correct); `FinSet.h:62`
  `getRootChord` (= the fin's `getLength`, an inference — see preamble), `:63-65`
  `getTipChord`/`getSpan`/`getSweep`, `:67` `getBodyRadius`, `:72` `getReferenceArea` (body disc),
  `:73` `getMaxRadius` (= `bodyRadius + span`, the rejected fin-extent disc).
- `HollowSphere.h:52` radii / no length member, `:89` shell members; HollowSphere has zero-CM, so
  `cmLocalZ = -L/2`.

**Serializer & visualizer**
- `DesignSerializer.cpp:53` `writeOffset`, `:64` `writePart`, `:73` child-writing loop (external
  `getChildParts` consumer), `:117` `buildPart`, `:127` fail-closed `makePart`, `:156` version `"0.1"`,
  `:186` the `major != "0"` loader gate, `:139`/`:141` (external `getChildParts` consumers).
- `visualizer/RocketMesh.cpp:87` `buildCone` (`+z` forward), `:150-156` `buildTube` (`+z` forward),
  `:427` `walk`, `:466` `getChildParts` loop, `:468-469` `std::get<0>`/`std::get<1>`, `:472`
  `station + offset`.

**External `getChildParts` / accessor consumers**
- `model/RocketModel.cpp:19` (`findMotorInTree`), `:177`/`:179` (`addPart`).
- `cli/Repl.cpp:223` (tree-walker).
- `visualizer/VisualizerWindow.cpp:65`.
- `model/tests/PartTests.cpp:410`/`:433`/`:492`/`:513`/`:515-518` (incl. the `std::get<0>` at `:516`/`:518`
  and the `PartCompositionAccess::childAt` friend accessor).
- `tests/DesignPersistenceTests.cpp:70-83` (incl. the stored-`Vector3` assertion `std::get<1>(...).z()
  == -0.13` at `:83` — this must change to a `StationLink`/resolved-station assertion, not just a
  rename), `:209`, `:237-239`.

**Test expectations to update for the corrected frame**
- `NoseConeTests.cpp:74` (`L/4` → `-L/4`), `:109`/`:163` (`L/6` → `-L/6`), `:144-150` (the `tipStation`
  aero anchor, re-expressed to still assert `2/3 L`); the inertia oracle `:79-119` stays unchanged.
- `FinSetTests.cpp:105` (`xc` → `xc - L/2`).
- `AeroTests.cpp:158` (`coneBaseToCm`), `:172`/`:220` (composite-CG expectations under the tip datum:
  `:220` flips from `> 0` to the CG being aft of the tip).

> **Note on `Motor`.** `Motor` is a real `Part` subclass in the live tree, but the resolver and
> diagnostics model only the four *geometry* part types the whitepaper enumerates (`ConicalNoseCone`,
> `BodyTube`, `FinSet`, `HollowSphere`); `Motor` is not given geometry virtuals or a station profile in
> v1.

---

## Appendix C — Invariants & guarantees (checklist)

- **Single source of truth.** Only *intent* (`StationLink`) is stored; the absolute `Pose` is never
  stored, never serialized. (§2.1, §7)
- **One resolver, two consumers, identical numbers.** Simulator and visualizer obtain every part's
  placement from the same `resolvePlacements` call; they cannot disagree. (§4.4)
- **CM is derived, uniformly.** `cmLocalZ = -L/2 + getCenterMassOffset().z()` for every part; no
  per-type adapter, no reversed frame, no non-mid reference. (§3.6)
- **Two cache gates, two cadences.** `placementDirty` (structural) gates the resolver/sweep;
  `builtAtCompositeMass` (mass-delta) gates the CM/inertia rebuild. Geometry resolves once per edit;
  mass re-weights every step. `setMass`/`setI` do not set `placementDirty`. (§4.5)
- **Diagnostics gate both consumers cheaply.** `SolveResult.ok == false` stops both; computed once per
  structural resolve, cached; guarding the inner loop is one boolean. (§6.5)
- **Physics-invariant migration (with one correction).** Bit-identical mass and inertia-about-CM; CG
  matches after one known datum shift to the nose tip; resolved stations bit-stable — gated by a test
  written first whose baseline is never disturbed. The static margin `cp() − cg()` is the one quantity
  the migration *corrects* rather than preserves (the legacy `cp` was CM-contaminated by ~0.1–2%); 3-DOF
  flight is unaffected (`cp` unused until 6-DOF). (§Part II, T6 as-built note)
- **Zero schema cost for 6-DOF.** `Pose::orient` and `StationLink::childRot` carried from day one;
  enabling rotation adds one composition line (`I' = R I Rᵀ`), no type/schema/signature change. (§8)
- **Degenerate geometry is total.** Clamped stations, guarded taper divide, point-sample zero-length
  parts — load-time diagnostics, never UB. (§3.5)
- **Narrow blast radius.** One storage swap, one new header, enumerated consumer rewrites; no force
  path, integrator, or state-vector change. (§10)
- **Tree, not graph, in v1.** Each child has exactly one parent and one mate ("single" = one *incoming*
  edge); fan-out is unrestricted (`childParts` is a vector). Multi-host designs are the deferred jump to
  a joint graph. (§9)

---

## Appendix D — Build/test commands

```bash
cmake --preset debug-clang
cmake --build --preset debug-clang
ctest --test-dir build -R 'qtrocket_*' -LE heavy    # fast loop, run after every step
ctest --test-dir build -R 'qtrocket_*'              # full, run at milestones (Steps 9, 12)
ctest --test-dir build -R 'qtrocket_*' -L heavy     # full ladder, run once at Step 13

# Targeted binaries:
./build/model/tests/model_tests          # profile, resolver, sweep, CM, cone/fin tests
./build/tests/integration_tests          # invariance gate, worked example, serialization
./build/tests/design_matrix_tests        # flight matrix smoke
```

The authoritative design source — including the figures, the full derivations, and the rendered
57-page PDF — is the whitepaper at `docs/PartPlacementDesignLatex/`. This document is the
engineer-facing distillation plus the executable, build-green, test-gated plan.
