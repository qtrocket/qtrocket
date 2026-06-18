# P2 — Concrete Part Types (`ConicalNoseCone`, `BodyTube`, `FinSet`) + `sim::Aero`: Build-Ready Spec

**Status:** ready to implement · **TODO item:** P2 · **Date:** 2026-06-18
**Scope:** add three closed-form rocket components as leaves of the existing `model::part::Part` composite tree — a straight `ConicalNoseCone` (solid + thin shell), a `BodyTube`, and a multi-fin `FinSet` (trapezoidal) — with honest mass / CM / per-unit-mass inertia, geometry-derived reference area (manual override retained), and the Barrowman geometry (lengths, radii, fin root/tip/span/sweep/count) designed in **now** for P5. Also: replace the dead `sim::Aero` skeleton with a per-part Barrowman value type + a composition walk that is **built and unit-tested but NOT yet consumed in the force path** (so 3-DOF trajectories stay bit-identical, mirroring how P1 built I(t) before anything consumed it).

This spec merges the strongest, **physics-verified** ideas from four candidate plans:
- **Plan 5 (API & extensibility architect)** — verified-correct cone constants, the rotate-and-sum single-fin-lamina FinSet derivation, the "every future part is just another leaf" extensibility framing, and the shared-reference-area composition invariant.
- **Plan 3 (Barrowman-aero-first)** — the `AeroComponent` representation storing the **CNalpha-weighted moment** so composite CP is literal field addition and zero-lift bodies vanish automatically.
- **Plan 4 (Minimal-incremental)** — the reference-area-as-**single max-radius frontal disc** rule (not a sum), with a manual-override flag.
- **Plan 8 (Aero-class specialist)** — the exact closed-form FinSet **spin** moment `Izz = J`, the diagnosis of *why* `sim::Aero` is commented out (unused stored fields trip `-Werror`), and the `cpValid` guard for a body-only rocket (CP undefined when total CNalpha = 0).

Physics that the verification stage flagged WRONG is corrected here (see §5.3: plan 8's FinSet transverse `Kz=(cr²+ct²)/24` is rectangle-only; the corrected trapezoid form is used; and we additionally validate the whole fin tensor against an independent numeric oracle rather than trusting any closed form).

---

## 1. Objective & scope

Today `RocketModel::topPart` is a placeholder `HollowSphere("Body", 0.04, 0.05, 2700)`. This item delivers the first real airframe components so a rocket can be assembled from a nose, body, fins, and the existing `Motor`.

**In scope**
- Three new leaf parts in `model::part`, each following the `HollowSphere`/`Motor` recipe exactly (static compute helpers feeding the `Part` base initializer; ctor-body `std::invalid_argument` validation; geometry getters; `~T() override = default`; protected defaulted copy ctor + `cloneShallow()`; registration in `Parts.h` + `model/CMakeLists.txt` + `model/tests/CMakeLists.txt`):
  - `ConicalNoseCone` — straight right circular cone, **solid** (default / first-class) or **thin conical shell**.
  - `BodyTube` — uniform hollow circular cylinder (reuses `InertiaTensors::Tube`).
  - `FinSet` — `N≥3` identical symmetric trapezoidal flat-plate fins, modeled as **one analytic leaf** (NOT `N` child parts — see §2; `N<3` warns and is not modeled for now).
- New `InertiaTensors` static helpers: `SolidCone(R,L)`, `ConicalShell(R,L)`, `TrapezoidalFinSet(N,cr,ct,s,sweep,thk,rb)` (all per-unit-mass, m², z longitudinal).
- Geometry-derived **reference area** with manual override preserved (§4).
- Replace the empty `sim::Aero` class with a per-part Barrowman value type + a `Part::getAero(...)` hook + a composite walk — **built and tested only**, consumed nowhere in P2 (§3.F, §6).
- Tests: `NoseConeTests.cpp`, `BodyTubeTests.cpp`, `FinSetTests.cpp`, `AeroTests.cpp`, plus an `InertiaTensorsTests.cpp` for the raw helpers, all with **independent** closed-form / numeric-integration oracles.

**Out of scope (deferred, breadcrumbs left in code)**
- **P5 Barrowman pipeline**: actually consuming `getCompositeAero()` in `getForces` (Cd build-up) and showing CG/CP/static margin live. P2 only defines and unit-tests the seam.
- **P6 6-DOF torques** and the moment coefficients (Cl/Cm/Cn) — deliberately omitted from the Aero struct.
- **P3 GUI editor / XML serialization** — the geometry fields are chosen so each part owns its own scalars + a (future) type tag; no serialization code here.
- Ogive/parabolic noses, transitions, airfoiled/canted/filleted fins, thick-wall conical frustum, body crossflow lift, end caps/centering rings. All flagged as future leaves.

---

## 2. The constraint everything hinges on: the part tree never rotates child tensors

`Part` documents (Part.h:40-42) that **all parts share one body-frame orientation, so child `position` offsets are pure translations and tensors combine by addition (no rotation)**. The composite walk `computeCompositeAt(t)` (Part.cpp) only ever parallel-axis-*shifts* a child tensor; it cannot *rotate* one.

Therefore a `FinSet` of `N` fins arrayed at azimuths `2πk/N` **cannot** be built by attaching `N` child `Part`s at different angles — each fin's own tensor would need an `Rz(2πk/N)` rotation the tree will never apply, silently corrupting the off-diagonals. **A `FinSet` must bake the full N-fin rotate-and-sum into its own single per-unit-mass tensor and present itself as one leaf.** This is the single most important correctness decision in this item, and it is verified: for `N≥3` the azimuthal sum is transversely isotropic (`Ixx=Iyy`, off-diagonals exactly 0).

**`FinSet` therefore requires `N≥3`** — the supported, transversely-isotropic domain. `N<3` is genuinely anisotropic (`Ixx≠Iyy`) and is **not modeled for now**: the ctor logs a warning and the helper returns the `N≥3` isotropic tensor (only an approximation for `N<3`) rather than throwing (§5.3). Proper anisotropic `N<3` support — like canted fins and any rotated component — waits on **per-part tensor rotation in the composite walk**, which is now tracked as a **P6 roadmap item** ([TODO.md](../TODO.md) P6 "Per-part orientation / tensor rotation"); once that lands, a `FinSet` can become `N` real child fins and the single-leaf workaround can be retired.

Two secondary integration traps, both real:
- **The cone CM is NOT at mid-length** (`L/4` from base solid, `L/3` from base shell). `Part`'s `cm` member is not consumed by composition (Part.h:231) and child `addChildPart` positions are CM-to-CM, so the cone reports a centroidal per-unit-mass tensor and the *attach* position must account for the CM-vs-tip offset. Covered by a composite oracle test (§7).
- **SI meters, not mm.** `MotorModel` stores diameter/length in mm (the `Motor` divides by 1000). All three new parts take SI meters; validation ranges (radii ~0.01–0.1 m) make an mm/m slip throw or fail a test.

---

## 3. Detailed changes

### 3.A `model/InertiaTensors.h` — three new per-unit-mass helpers

All return diagonal `Matrix3`, units m², z = longitudinal/spin axis, **about the part's own CM**, matching the existing `SolidSphere`/`Tube` style. Every constant below was independently re-derived (symbolic + numeric) by the verification stage and confirmed exact.

```cpp
/**
 * @brief Solid right circular cone, PER UNIT MASS, about the cone's own CM (at L/4 from the base,
 *        3L/4 from the apex). z is the symmetry/longitudinal axis.
 *        Izz/m = (3/10) R^2 ; Ixx/m = Iyy/m = (3/20) R^2 + (3/80) L^2.
 * @param R base radius (meters)
 * @param L axial height tip-to-base (meters)
 */
static Matrix3 SolidCone(double R, double L)
{
    double zz = (3.0 / 10.0) * R * R;
    double xx = (3.0 / 20.0) * R * R + (3.0 / 80.0) * L * L;
    double yy = xx;
    return Matrix3{{xx, 0.0, 0.0}, {0.0, yy, 0.0}, {0.0, 0.0, zz}};
}

/**
 * @brief Thin conical lateral shell (open base, uniform areal density, t << R), PER UNIT MASS,
 *        about the shell's own CM (at L/3 from the base, 2L/3 from the apex).
 *        Izz/m = (1/2) R^2 ; Ixx/m = Iyy/m = (1/4) R^2 + (1/18) L^2.
 *        (Verified exact; do NOT treat the (1/18)L^2 term as uncertain.)
 */
static Matrix3 ConicalShell(double R, double L)
{
    double zz = (1.0 / 2.0) * R * R;
    double xx = (1.0 / 4.0) * R * R + (1.0 / 18.0) * L * L;
    double yy = xx;
    return Matrix3{{xx, 0.0, 0.0}, {0.0, yy, 0.0}, {0.0, 0.0, zz}};
}

/**
 * @brief N identical symmetric trapezoidal flat-plate fins arrayed about the z-axis, PER UNIT
 *        (total set) MASS, about the SET CM (which lies ON the z-axis for N >= 2). Assembled by
 *        rotate-and-sum of one fin's centroidal lamina tensor + a radial parallel-axis shift to the
 *        body axis; the long algebra is pinned by an independent numeric oracle (FinSetTests), so
 *        this helper is NOT trusted on faith.
 *
 *        Assumes N >= 3 (the supported, forced domain): the azimuthal sum is then transversely
 *        isotropic (Ixx = Iyy, off-diagonals == 0) and is returned diagonal. N < 3 is genuinely
 *        anisotropic and is NOT modeled for now -- this helper still returns the N >= 3 isotropic
 *        tensor (an approximation for N < 3) and FinSet's ctor logs a warning rather than throwing.
 *        True anisotropic N < 3 support waits on per-part tensor rotation in the Part tree (P6).
 *
 * @param N       fin count (>= 3 supported; N < 3 warns and uses the N >= 3 isotropic approximation)
 * @param cr      root chord (m, along z at the body surface)
 * @param ct      tip chord (m)
 * @param s       semi-span / fin height (m, radial, body surface to tip)
 * @param sweep   leading-edge sweep length Xt (m): axial distance the tip LE is aft of the root LE
 * @param thk     fin plate thickness (m)
 * @param rb      body radius the fins mount on (m): radial offset of the fin root
 */
static Matrix3 TrapezoidalFinSet(unsigned int N, double cr, double ct, double s,
                                 double sweep, double thk, double rb);
```

**Implementation of `TrapezoidalFinSet`** (out-of-line, in a new `model/InertiaTensors.cpp` — the file is currently header-only, so add it to `model/CMakeLists.txt`; the algebra is too long to inline cleanly and the test oracle is the source of truth). Method (verified by both symbolic algebra and Monte-Carlo mesh integration):

1. **One fin's centroidal lamina tensor.** Treat one fin as a thin trapezoidal plate lying in a meridian plane (the plane containing the z-axis), thickness `thk` in the circumferential (out-of-plane) direction. Local axes: `u` = spanwise/radial, `w` = chordwise/axial (≡ z). Verified single-fin per-unit-mass second moments about the fin's own area centroid:
   - spanwise: `Iuu = s^2 (cr^2 + 4 cr ct + ct^2) / (18 (cr+ct)^2)`  *(verified EXACT)*
   - chordwise: `Iww = (cr^4 + 2 cr^3 ct + 2 cr ct^3 + ct^4) / (18 (cr+ct)^2)` plus the sweep-induced spread — **use the verified corrected chordwise second moment; do NOT use `(cr^2+ct^2)/24`, which is rectangle-only** (see §5.3). For a swept fin add the area-weighted axial centroid shift; pin against the mesh.
   - product `Iuw` (nonzero for swept fins), and out-of-plane via the thin-plate perpendicular-axis relation plus the `thk^2/12` through-thickness term.
2. **Radial parallel-axis** each fin centroid out to body radius: lever `d = rb + yc`, `yc = (s/3)(cr+2ct)/(cr+ct)` (verified).
3. **Rotate-and-sum** `Σ_k Rz(2πk/N) · I_fin · Rz(2πk/N)^T` plus the radial PA term, then **divide by set mass** to honor the per-unit-mass contract.

**Verified closed form available as an internal cross-check (not the sole source of truth):** the set **spin** moment is exact and sweep-independent:
```
Izz/m = J + thk^2/12,  where  J = mean radial^2
      = (6 rb^2 (cr+ct) + 4 rb s (cr+2ct) + cr s^2 + 3 ct s^2) / (6 (cr+ct))
```
and, by the axisymmetry identity for `N≥3`, `Ixx/m = Iyy/m = J/2 + Kz` with the **corrected** chordwise term `Kz` (the trapezoid `Iww` above, including sweep), NOT `(cr^2+ct^2)/24`. The implementation may assert this closed form against the rotate-and-sum result in tests, but the **mesh oracle is the acceptance gate.**

`BodyTube` needs **no** new helper — it reuses `InertiaTensors::Tube(ri,ro,L)` verbatim (the canonical case that helper was written for).

### 3.B `model/parts/ConicalNoseCone.{h,cpp}` (new)

```cpp
// ConicalNoseCone.h  (namespace model::part)
class ConicalNoseCone : public Part
{
public:
   /// @param baseRadius    R (m), the open aft radius == the body tube it caps
   /// @param length        L (m), tip-to-base axial height
   /// @param wallThickness t (m), used only when solid == false (thin lateral shell)
   /// @param density       rho (kg/m^3)
   /// @param solid         true => uniform solid cone (default, "straight cone first"); false =>
   ///                      thin conical shell (open base) with wall t
   /// @param centerMass    extra CM offset w.r.t. the component middle (defaults to origin)
   /// @throws std::invalid_argument unless R>0 && L>0 && density>0 && (solid || (0<t<R))
   ConicalNoseCone(const std::string& name, double baseRadius, double length,
                   double wallThickness, double density, bool solid = true,
                   const Vector3& centerMass = {0.0, 0.0, 0.0});
   ~ConicalNoseCone() override = default;

   double getBaseRadius()    const { return baseRadius; }
   double getLength()        const { return length; }
   double getWallThickness() const { return wallThickness; }
   double getDensity()       const { return density; }
   bool   isSolid()          const { return solid; }
   double getReferenceArea() const { return std::numbers::pi * baseRadius * baseRadius; } ///< pi*R^2
   double getMaxRadius()     const { return baseRadius; } ///< for the rocket-wide max-disc ref area

   sim::AeroComponent getAero(double refArea) const override; ///< Barrowman; see §3.F

protected:
   ConicalNoseCone(const ConicalNoseCone&) = default;
   std::shared_ptr<Part> cloneShallow() const override
   { return std::shared_ptr<Part>(new ConicalNoseCone(*this)); }

private:
   static double  computeVolume(double R, double L, double t, bool solid);
   static double  computeMass(double R, double L, double t, double density, bool solid);
   static Matrix3 coneTensor(double R, double L, bool solid);   ///< SolidCone or ConicalShell
   static Vector3 coneCmOffset(double L, bool solid);           ///< {0,0, base->CM offset}

   double baseRadius, length, wallThickness, density;
   bool   solid;
};
```

```cpp
// ConicalNoseCone.cpp  (key bodies)
ConicalNoseCone::ConicalNoseCone(const std::string& name, double R, double L, double t,
                                 double density_, bool solid_, const Vector3& cm)
   : Part(name, coneTensor(R, L, solid_), computeMass(R, L, t, density_, solid_),
          coneCmOffset(L, solid_) + cm),
     baseRadius(R), length(L), wallThickness(t), density(density_), solid(solid_)
{
   if(!(R > 0.0 && L > 0.0 && density_ > 0.0 && (solid_ || (t > 0.0 && t < R))))
   {
      throw std::invalid_argument(
         "ConicalNoseCone requires R>0, L>0, density>0 and (solid or 0<wallThickness<R)");
   }
}

double ConicalNoseCone::computeVolume(double R, double L, double t, bool solid)
{
   if(solid) { return (1.0 / 3.0) * std::numbers::pi * R * R * L;        } // (1/3) pi R^2 L
   const double slant = std::sqrt(R * R + L * L);
   return std::numbers::pi * R * slant * t;                               // lateral area * t (thin)
}

double ConicalNoseCone::computeMass(double R, double L, double t, double density, bool solid)
{ return density * computeVolume(R, L, t, solid); }

Matrix3 ConicalNoseCone::coneTensor(double R, double L, bool solid)
{ return solid ? InertiaTensors::SolidCone(R, L) : InertiaTensors::ConicalShell(R, L); }

// The tensor is centroidal; report where that CM sits relative to the component middle so the
// assembly's addChildPart position is CM-to-CM. CM is hbar from the base: L/4 (solid), L/3 (shell).
Vector3 ConicalNoseCone::coneCmOffset(double L, bool solid)
{ const double hbar = solid ? (L / 4.0) : (L / 3.0); return Vector3{0.0, 0.0, L / 2.0 - hbar}; }
```

> **CM convention note (load-bearing).** With the cone occupying `[-L/2, +L/2]` in z (base at `+L/2`), the centroid sits `hbar` forward of the base, i.e. at `z = +L/2 - hbar` relative to the component middle. `coneCmOffset` returns exactly that, so when the cone is attached to a body tube via `addChildPart`, the caller positions it CM-to-CM. The cone is the only one of the three parts with a non-central CM; this is the top integration risk and is pinned by a composite oracle test (§7).

### 3.C `model/parts/BodyTube.{h,cpp}` (new)

```cpp
// BodyTube.h  (namespace model::part)
class BodyTube : public Part
{
public:
   /// @throws std::invalid_argument unless 0 <= ri < ro && length > 0 && density > 0
   BodyTube(const std::string& name, double innerRadius, double outerRadius, double length,
            double density, const Vector3& centerMass = {0.0, 0.0, 0.0});
   ~BodyTube() override = default;

   double getInnerRadius()   const { return innerRadius; }
   double getOuterRadius()   const { return outerRadius; }
   double getLength()        const { return length; }
   double getDensity()       const { return density; }
   double getWettedArea()    const { return 2.0 * std::numbers::pi * outerRadius * length; } ///< P5 skin friction
   double getReferenceArea() const { return std::numbers::pi * outerRadius * outerRadius; }  ///< pi*ro^2
   double getMaxRadius()     const { return outerRadius; }

   sim::AeroComponent getAero(double refArea) const override; ///< CNalpha = 0 (constant-diameter body)

protected:
   BodyTube(const BodyTube&) = default;
   std::shared_ptr<Part> cloneShallow() const override
   { return std::shared_ptr<Part>(new BodyTube(*this)); }

private:
   static double computeVolume(double ri, double ro, double L);
   static double computeMass(double ri, double ro, double L, double density);
   double innerRadius, outerRadius, length, density;
};
```

```cpp
// BodyTube.cpp  (key bodies)
BodyTube::BodyTube(const std::string& name, double ri, double ro, double L, double density_,
                   const Vector3& cm)
   : Part(name, InertiaTensors::Tube(ri, ro, L), computeMass(ri, ro, L, density_), cm),
     innerRadius(ri), outerRadius(ro), length(L), density(density_)
{
   if(!(ri >= 0.0 && ro > ri && L > 0.0 && density_ > 0.0))
   { throw std::invalid_argument("BodyTube requires 0 <= ri < ro, length > 0, density > 0"); }
}

double BodyTube::computeVolume(double ri, double ro, double L)
{ return std::numbers::pi * (ro * ro - ri * ri) * L; }

double BodyTube::computeMass(double ri, double ro, double L, double density)
{ return density * computeVolume(ri, ro, L); }
```
CM is `{0,0,0}` (geometric center, by symmetry — the default `cm`).

### 3.D `model/parts/FinSet.{h,cpp}` (new)

```cpp
// FinSet.h  (namespace model::part)
class FinSet : public Part
{
public:
   /// @param finCount   N (>= 3 supported/forced; N < 3 is non-axisymmetric, unsupported for now ->
   ///                   logs a warning and uses the N >= 3 isotropic approximation, does NOT throw)
   /// @param rootChord  cr (m), chord at the body surface, along z
   /// @param tipChord   ct (m), chord at the tip (>= 0; ct == 0 is an allowed triangular fin)
   /// @param span       s  (m), semi-span / radial fin height (body surface to tip)
   /// @param sweep      Xt (m), leading-edge sweep LENGTH (axial tip-LE offset aft of root LE) -- NOT an angle
   /// @param thickness  thk (m), flat-plate thickness
   /// @param bodyRadius rb (m), outer radius of the tube the fins mount on
   /// @param density    rho (kg/m^3)
   /// @throws std::invalid_argument unless N>=1 && cr>0 && ct>=0 && (cr+ct)>0 && s>0 && thk>0 && rb>=0 && rho>0
   FinSet(const std::string& name, unsigned int finCount, double rootChord, double tipChord,
          double span, double sweep, double thickness, double bodyRadius, double density,
          const Vector3& centerMass = {0.0, 0.0, 0.0});
   ~FinSet() override = default;

   unsigned int getFinCount()  const { return finCount; }
   double getRootChord()       const { return rootChord; }
   double getTipChord()        const { return tipChord; }
   double getSpan()            const { return span; }
   double getSweep()           const { return sweep; }
   double getThickness()       const { return thickness; }
   double getBodyRadius()      const { return bodyRadius; }
   double getDensity()         const { return density; }
   double getSingleFinArea()   const { return 0.5 * (rootChord + tipChord) * span; }
   double getTotalFinArea()    const { return finCount * getSingleFinArea(); }      ///< P5 fin CNalpha normalization
   double getWettedArea()      const { return 2.0 * getTotalFinArea(); }            ///< both faces, P5 skin friction
   double getReferenceArea()   const { return std::numbers::pi * bodyRadius * bodyRadius; } ///< body disc, for consistency
   double getMaxRadius()       const { return bodyRadius + span; }                  ///< fin tip radius (extent only)

   sim::AeroComponent getAero(double refArea) const override; ///< Barrowman fin-set term; see §3.F

protected:
   FinSet(const FinSet&) = default;
   std::shared_ptr<Part> cloneShallow() const override
   { return std::shared_ptr<Part>(new FinSet(*this)); }

private:
   static double  computeMass(unsigned int N, double cr, double ct, double s, double thk, double density);
   static Vector3 finSetCmOffset(double cr, double ct, double sweep); ///< {0,0, chordwise centroid} for N>=2
   double rootChord, tipChord, span, sweep, thickness, bodyRadius, density;
   unsigned int finCount;
};
```

```cpp
// FinSet.cpp  (key bodies)
FinSet::FinSet(const std::string& name, unsigned int N, double cr, double ct, double s,
               double xt, double thk, double rb, double density_, const Vector3& cm)
   : Part(name,
          InertiaTensors::TrapezoidalFinSet(N, cr, ct, s, xt, thk, rb),
          computeMass(N, cr, ct, s, thk, density_),
          finSetCmOffset(cr, ct, xt) + cm),
     rootChord(cr), tipChord(ct), span(s), sweep(xt), thickness(thk),
     bodyRadius(rb), density(density_), finCount(N)
{
   if(!(N >= 1 && cr > 0.0 && ct >= 0.0 && (cr + ct) > 0.0 && s > 0.0 && thk > 0.0
        && rb >= 0.0 && density_ > 0.0))
   { throw std::invalid_argument("FinSet: bad geometry (need N>=1, cr>0, ct>=0, s>0, thk>0, rb>=0, rho>0)"); }

   if(N < 3)
   {
      // FinSet forces the N >= 3 axisymmetric model: TrapezoidalFinSet returns the transversely
      // isotropic tensor, which is only an APPROXIMATION for N < 3 (truly anisotropic, Ixx != Iyy).
      // N < 3 is unsupported for now -- warn rather than throw (kept intentionally; a future hard
      // requirement may reject N < 3 outright). Proper anisotropic N < 3 support needs per-part
      // tensor rotation in the Part tree (P6 -- see TODO.md).
      utils::Logger::getInstance().warning(
         "FinSet: finCount < 3 is unsupported (using the N>=3 axisymmetric approximation)");
   }
}

double FinSet::computeMass(unsigned int N, double cr, double ct, double s, double thk, double density)
{ return N * density * (0.5 * (cr + ct) * s) * thk; }

// Axial (z) mass centroid of one trapezoid from the root LE; transverse cancels to the axis for
// N >= 2 by symmetry. NOTE: this MASS centroid differs from the aero CP (Xf, see getAero) -- they are
// different quantities with different formulas; do not conflate them (§5.3).
Vector3 FinSet::finSetCmOffset(double cr, double ct, double sweep)
{
   const double xcMass = (cr*cr + cr*ct + ct*ct + sweep*(cr + 2.0*ct)) / (3.0 * (cr + ct));
   // expressed relative to the component middle; convention mirrors the cone (caller attaches CM-to-CM)
   return Vector3{0.0, 0.0, xcMass};
}
```

> **Why one leaf, not N children:** see §2. Verified necessary — the tree cannot rotate child tensors.

### 3.E `model/parts/Parts.h` + `model/CMakeLists.txt` + `model/tests/CMakeLists.txt` — wiring

`Parts.h` (umbrella header) gains three includes next to `HollowSphere.h`/`Motor.h`:
```cpp
#include "model/parts/ConicalNoseCone.h"
#include "model/parts/BodyTube.h"
#include "model/parts/FinSet.h"
```
`model/CMakeLists.txt` `model` library sources gain (next to `parts/Motor.*`), plus the now-non-header-only `InertiaTensors.cpp`:
```cmake
   parts/ConicalNoseCone.cpp
   parts/ConicalNoseCone.h
   parts/BodyTube.cpp
   parts/BodyTube.h
   parts/FinSet.cpp
   parts/FinSet.h
   InertiaTensors.cpp   # was header-only; now hosts TrapezoidalFinSet
   InertiaTensors.h
```
`model/tests/CMakeLists.txt` `model_tests` sources gain `InertiaTensorsTests.cpp`, `NoseConeTests.cpp`, `BodyTubeTests.cpp`, `FinSetTests.cpp`, `AeroTests.cpp` (next to `PartTests.cpp`).

### 3.F `sim/Aero.{h,cpp}` + `model/parts/Part.{h,cpp}` — the Aero seam (built, NOT consumed)

**Decision: REPLACE the empty `sim::Aero` skeleton.** Its commented-out fields (`cp; Cx,Cy,Cz; Cl,Cm,Cn; baseCd,Cd`) are commented out precisely because, as **stored** members on an ownerless class, they tripped `-Werror` unused-field warnings; they also (a) presuppose a 6-DOF body/moment-coefficient model nothing produces in P2/P5, and (b) conflate per-part contributions with the assembled whole-rocket result. We replace the contents with a focused value type that each part **returns on demand** (no stored aero state, so no staleness and no `-Werror` issue), and compose it by a tree walk. The 6-DOF moment coefficients (Cl/Cm/Cn) are deferred to P6 with a one-line breadcrumb — they are not declared now.

```cpp
// sim/Aero.h  (replaces the class body; keep the file and sim/Aero.cpp in the build graph)
namespace sim
{

/**
 * @brief One part's Barrowman contribution, all referenced to a SHARED rocket reference area so the
 *        pieces are directly additive. Stores the CNalpha-WEIGHTED moment (cnAlphaXcp) rather than a
 *        raw x_cp, so composing CP is literal field addition and a zero-lift body (CNalpha == 0)
 *        drops out of the CP weighted-average automatically with no special case.
 *
 * NOTE(P6): roll/pitch/yaw moment coefficients (Cl/Cm/Cn) are intentionally NOT modeled here yet;
 * they are derived at P6 from CNalpha and the CP-CG lever, not stored per part.
 */
struct AeroComponent
{
   double cnAlpha{0.0};      ///< normal-force-coeff slope (per rad), referenced to the shared refArea
   double cnAlphaXcp{0.0};   ///< cnAlpha * x_cp (axial CP station from the nose tip datum, m) -- the weighted moment
   double cd{0.0};           ///< this part's drag-coeff contribution, already normalized to refArea
};

/**
 * @brief The assembled whole-rocket aero profile. cnAlpha and cnAlphaXcp add over parts; cd adds.
 *        cp() is the CNalpha-weighted CP; cpValid guards the body-only case where cnAlpha == 0.
 */
struct AeroProfile
{
   double cnAlpha{0.0};
   double cnAlphaXcp{0.0};
   double cd{0.0};
   double refArea{0.0};
   bool   cpValid{false};      ///< false when cnAlpha == 0 (CP undefined; e.g. body tube only)

   double cp() const { return cpValid ? cnAlphaXcp / cnAlpha : 0.0; } ///< axial CP from nose tip (m)

   AeroProfile& operator+=(const AeroComponent& c)
   { cnAlpha += c.cnAlpha; cnAlphaXcp += c.cnAlphaXcp; cd += c.cd; cpValid = (cnAlpha != 0.0); return *this; }
};

} // namespace sim
```
`sim/Aero.cpp` stays in the build (avoids touching the sim build graph and the existing `Propagatable::aeroData` member, which remains default-constructed and unread); it may become a trivial translation unit or host any non-inline helper. **Do not delete it** — `model/Propagatable.h` declares a `sim::Aero aeroData;` member, so `sim::Aero` must remain a complete, default-constructible type; the replacement struct(s) satisfy this.

**`Part` gains an aero hook (default inert), paralleling `getMass(t)`/`getCompositeI(t)`:**
```cpp
// Part.h  (add #include "sim/Aero.h"; model links sim PUBLIC, so this is legal)
/**
 * @brief This part's Barrowman aero contribution, normalized to the shared rocket reference area
 *        @p refArea. Default = aerodynamically inert (HollowSphere, Motor, BodyTube need no override
 *        beyond CNalpha=0). Pure function of geometry; NOT stored (no staleness, no -Werror field).
 */
virtual sim::AeroComponent getAero(double refArea [[maybe_unused]]) const { return {}; }

/**
 * @brief Assemble the composite Barrowman profile over this sub-tree, normalized to @p refArea.
 *        Folds each part's getAero(refArea) by the additive AeroComponent rule (CNalpha and the
 *        CNalpha-weighted moment add; Cd adds). A SEPARATE pass from computeCompositeAt(t) -- it does
 *        NOT touch the mass-delta-gated inertia cache (aero invalidation is Mach/Re, not t).
 *        Threads each child's axial station so x_cp shares one datum (the nose tip).
 *        NOTE(P5): RocketModel::getForces will consume this; nothing consumes it in P2.
 */
sim::AeroProfile getCompositeAero(double refArea); // implemented in Part.cpp
```
`getCompositeAero` walks the tree exactly like `computeCompositeAt` structurally (threading the cumulative axial offset from child `position`s so each part's `x_cp` is expressed from one datum), accumulating into an `AeroProfile` and setting `refArea`. It is `[[maybe_unused]]`-clean because every parameter is read.

**Per-part `getAero` (verified Barrowman, all referenced to `refArea`):**
- `ConicalNoseCone::getAero(refArea)` → `cnAlpha = 2.0 * (pi R^2 / refArea)`; `x_cp = (2/3) L` from the tip (cone-specific); `cnAlphaXcp = cnAlpha * x_cp`; `cd = 0` (documented P5 placeholder).
- `BodyTube::getAero(refArea)` → `cnAlpha = 0` (constant-diameter body, classic Barrowman; drops out of CP). `cnAlphaXcp = 0`. `cd = 0` placeholder (skin friction over `getWettedArea()` is the P5 input).
- `FinSet::getAero(refArea)` → `cnAlpha = Kfb * 4N(s/d)^2 / (1 + sqrt(1 + (2 lm/(cr+ct))^2))` rescaled to `refArea`, with `d = 2 rb`, `lm = sqrt(s^2 + (sweep + (ct-cr)/2)^2)`, `Kfb = 1 + rb/(s+rb)`; `x_cp` (from root LE) `= (sweep/3)(cr+2ct)/(cr+ct) + (1/6)((cr+ct) - cr ct/(cr+ct))`; `cnAlphaXcp = cnAlpha * (rootLEstation + x_cp)`; `cd = 0` placeholder. All verified correct.

Default `getAero` (inert) covers `HollowSphere` and `Motor`.

### 3.G `model/RocketModel.{h,cpp}` — reference area from geometry (manual override retained), Aero NOT wired

Per §4 the manual `referenceArea`/`setReferenceArea` override stays. Add the geometry-derived default behind an explicit override flag, computed from the tree's **single max-radius frontal disc**. **`getForces` is otherwise byte-for-byte unchanged** — it keeps `dragCoefficient` and `referenceArea`, and nothing reads `getCompositeAero` in P2, so every existing integration test is bit-identical (§6).

Add to `RocketModel`:
```cpp
bool referenceAreaOverridden{false};   ///< true once setReferenceArea() set a user value
void setReferenceArea(double a) { if(a >= 0.0) { referenceArea = a; referenceAreaOverridden = true; } }

/// @brief Rocket reference (frontal) area derived from geometry: pi * (max part getMaxRadius())^2 --
///        the single widest frontal disc (Barrowman/OpenRocket convention), NOT a sum of part areas
///        (which would double-count the silhouette). Used as the default unless the user overrode it.
double deriveReferenceAreaFromGeometry() const; // walks topPart, takes max getMaxRadius()
```
The geometry-derived value seeds `referenceArea` only when `!referenceAreaOverridden`. A `// NOTE(P5): consume getCompositeAero(referenceArea).cd here, manual dragCoefficient wins` breadcrumb goes on the `getForces` drag line.

---

## 4. Reference area: derived-from-geometry with manual override (the P2 requirement)

- Each part exposes `getReferenceArea()` (its own frontal disc) and `getMaxRadius()`.
- The **rocket** reference area = `pi * (max over parts of getMaxRadius())^2`. Verified: this is the correct Barrowman/OpenRocket convention; summing per-part areas would triple-count the same silhouette. Fins return `getMaxRadius() = rb + s` (extent only) but **must not inflate** the reference disc — pinned by a test (§7).
- Manual override is preserved exactly: `RocketModel::setReferenceArea(double)` keeps the existing `>= 0` guard and now sets `referenceAreaOverridden`; the geometry-derived default applies only when the user has not overridden. Default rocket (placeholder body, no override) keeps `1.134e-3` until a tree is assembled, so existing trajectories are unchanged.

---

## 5. Physics: what is used, and the one correction

### 5.1 Cone — all four constants verified exact
- Solid: `V = (1/3)πR²L`; CM at **L/4 from base** (3L/4 from apex); `Izz/m = (3/10)R²`; `Ixx/m = Iyy/m = (3/20)R² + (3/80)L²` (about CM).
- Thin shell: `m = ρ·t·πR·√(R²+L²)`; CM at **L/3 from base** (2L/3 from apex); `Izz/m = (1/2)R²`; `Ixx/m = Iyy/m = (1/4)R² + (1/18)L²` (about CM).
- Barrowman: `CNalpha = 2` (ref base area, shape-independent); cone `x_cp = (2/3)L` from tip.
- **Solid is the default** (`solid = true`), matching the TODO's "straight cone first." The thin-shell path is offered; a thick-wall conical frustum is deferred. (Plan 4's cone-difference hollow model is an exact alternative for a thick wall but has CM at L/4, not the true shell L/3 — we use the true thin shell and document the assumption.)

### 5.2 BodyTube — reuse `InertiaTensors::Tube`, all verified
- `m = ρπ(ro²−ri²)L`; CM at center; `Ixx/m=Iyy/m=(1/12)(3(ri²+ro²)+L²)`; `Izz/m=(1/2)(ri²+ro²)`. Barrowman `CNalpha = 0`.

### 5.3 FinSet — verified, with one corrected term and a mandatory numeric oracle
- Mass/area/centroids verified exact: `A1=(cr+ct)/2·s`, `yc=(s/3)(cr+2ct)/(cr+ct)`, axial mass centroid `xc=(cr²+cr·ct+ct²+Xt(cr+2ct))/(3(cr+ct))`.
- **Spin moment `Izz/m = J (+ thk²/12)` is exact** (J as in §3.A; verified to full precision — this is plan 8's contribution).
- **CORRECTION (verifier-confirmed):** plan 8's transverse chordwise term `Kz = (cr²+ct²)/24` is **wrong** — it is the rectangle-only special case and is ~1% (trapezoid) to 33% (triangle) low. Use the corrected trapezoid chordwise second moment `Kz = (cr⁴+2cr³ct+2cr·ct³+ct⁴)/(18(cr+ct)²)` (plus the sweep-induced axial spread), so `Ixx/m=Iyy/m = J/2 + Kz` for `N≥3`. Because this algebra is error-prone, **the acceptance gate is an independent Monte-Carlo / point-mass mesh oracle** (plans 1/3/4/5/6's discipline), not the closed form — the closed form is asserted *against* the mesh in tests, never trusted alone.
- **FinSet forces the `N≥3` axisymmetric domain** (`Ixx=Iyy`, off-diagonals 0 — verified). `N<3` is genuinely anisotropic and is **not modeled for now**: `TrapezoidalFinSet` returns the `N≥3` isotropic tensor (an approximation for `N<3`) and the ctor logs a warning (warn-don't-throw, kept for now). True anisotropic `N<3` support waits on per-part tensor rotation in the tree (P6 — see [TODO.md](../TODO.md)).
- Barrowman fin `CNalpha` (with `Kfb=1+rb/(s+rb)`, `lm` mid-chord length) and fin `x_cp` (from root LE) verified exact. The fin **mass centroid** and the aero **CP** are distinct quantities (different formulas) — do not conflate (verifier note).

### 5.4 Aero composition — verified
- `cnAlpha_total = Σ cnAlpha_i`; composite CP `= Σ(cnAlpha_i · x_cp_i)/Σ cnAlpha_i` — realized as literal addition of `cnAlphaXcp` then one divide. Zero-CNalpha parts vanish automatically. `cpValid=false` guards the body-only divide-by-zero. `cd` adds. All pieces share **one** `refArea` (the composition invariant) — `getCompositeAero(refArea)` passes it to every `getAero`.

---

## 6. Backward-compatibility / numerical-invariance

P2 changes **nothing** in the force/torque path:
- `getForces` still uses `dragCoefficient` and `referenceArea`; `getTorques` still returns zero. `getCompositeAero` is built and unit-tested but read by no production code.
- Geometry-derived reference area is **default-but-manual-wins** and only applies once a real tree is assembled with no override; the placeholder rocket and every test that calls `setReferenceArea`/`setMass` are unaffected.
- The new parts ride the unchanged `computeCompositeAt(t)` walk (they are leaves supplying a per-unit-mass tensor + mass + CM, exactly the `HollowSphere`/`Motor` contract).

Therefore the existing apogee / flight-time / terminal-velocity / motor-DB integration tests must pass **bit-identical**. The only externally visible additions are the new types, the new `InertiaTensors` helpers, the `Part::getAero` virtual (defaulted, inert), and the replaced `sim::Aero` contents.

---

## 7. Test plan

New files (PartTests/MotorTests idiom: anonymous-namespace **independent** oracle helpers; `EXPECT_NEAR` ~1e-9..1e-12; off-diagonals `EXPECT_NEAR(...,0,...)`; `EXPECT_THROW` for bad geometry; `dynamic_cast` + fresh-id clone test per part).

**`InertiaTensorsTests.cpp`** (raw helpers, no Part):
- `SolidConeMatchesNumericDiskIntegration` — Riemann/MC integrate `Izz, Ixx` over the cone volume; compare to `SolidCone(R,L)`; assert `Izz=(3/10)R²`, `Ixx=(3/20)R²+(3/80)L²`.
- `ConicalShellMatchesSurfaceIntegral` — surface-integral oracle for `(1/2)R²`, `(1/4)R²+(1/18)L²`.
- `TrapezoidalFinSetMatchesMesh` — **the key test**: discretize the N-fin solid (each fin a trapezoidal prism at azimuth `2πk/N`, radius `rb..rb+s`), brute-force the full per-unit-mass tensor about the set CM; `EXPECT_NEAR` to `TrapezoidalFinSet` for `N∈{3,4,6}` (MC tol ~1%, fixed seed). Assert `Ixx=Iyy`, off-diagonals ≈ 0. Spot-check `Izz=J` against the exact closed form. `N=2`: assert the helper returns the forced `N≥3` isotropic tensor (`Ixx≈Iyy`) — i.e. it does NOT compute a true anisotropic tensor (documents the forced-axisymmetric model).

**`NoseConeTests.cpp`** (solid R=0.019, L=0.10, ρ=2700 → m≈0.1021 kg; oracles independent):
- `SolidMassMatchesClosedForm` (`ρ·(1/3)πR²L`); `SolidCmOffsetIsLOver4FromBase`; `SolidInertiaMatchesNumericDiskIntegration` (validates the (3/80)L² term independently).
- `ShellMassCmInertiaMatchSurfaceIntegral` (CM at L/3 from base; `(1/2)R²`, `(1/4)R²+(1/18)L²`).
- `ReducesToShellVsSolidCorrectly`; off-diagonals 0.
- `AeroConeCNalphaAndCp` — `getAero(πR²).cnAlpha==2.0`; scales by `(πR²/refArea)` otherwise; `cnAlphaXcp == 2·(2/3)L`.
- `RejectsNonPhysical` (R≤0, L≤0, ρ≤0, shell with t≥R or t≤0); `CloneIsDeepTypePreserving`.

**`BodyTubeTests.cpp`** (reuse the existing `tubeMass` oracle idiom from PartTests):
- `MassMatchesHollowCylinder`; `CompositeIEqualsMassTimesTube` (compare `getCompositeI(0)` to `mass*InertiaTensors::Tube`, asserting via the independent integral forms).
- `TwoBodyTubesEndToEndEqualOneLongerTube` — reuse the exact PartTests end-to-end pattern with real `BodyTube`s (cross-checks the part feeds the composite walk correctly via the L² parallel-axis term).
- `AeroBodyHasZeroCNalpha`; `RejectsNonPhysical` (ri≥ro, L≤0, ρ≤0); `CloneIsDeepTypePreserving`.

**`FinSetTests.cpp`** (N=3, cr=0.10, ct=0.05, s=0.05, sweep=0.04, thk=0.003, rb=0.019, ρ=600):
- `MassEqualsNTrapezoids`; `CmOnAxisForN3AndN4` (`getCompositeCm` x≈y≈0, z = axial mass centroid).
- `InertiaMatchesMeshOracle` — `getCompositeI(0)` vs the brute-force mesh (the acceptance gate for the corrected `Kz`); `Ixx=Iyy`, off-diagonals ≈ 0.
- `SingleFinDegeneratesToRectangle` — ct=cr, sweep=0 → in-plane second moments reduce to the rectangular-plate values (independent oracle).
- `FinCountBelow3Warns` (N=2 constructs without throwing, logs the unsupported-fin-count warning, and returns the forced `N≥3` isotropic approximation `Ixx≈Iyy` — pins the warn-don't-throw policy); `AeroFinSetBarrowman` (`cnAlpha` and CP vs the independently-typed Barrowman formula for the worked example); `RejectsNonPhysical` (N=0, s≤0, thk≤0, cr≤0, cr+ct==0); `CloneIsDeepTypePreserving`.

**`AeroTests.cpp`** (the composition seam):
- `CompositeCpIsCNalphaWeighted` — nose+body+finset at known axial stations; assert `getCompositeAero(refArea).cp()` equals the hand-computed `(2·x_nose + 0·x_body + CNa_fins·x_fins)/(2 + 0 + CNa_fins)`; the zero-CNalpha body must not corrupt the average (headline pin).
- `CompositeCdIsAdditive`; `BodyOnlyRocketHasInvalidCp` (`cpValid==false` for a lone BodyTube); `SharedReferenceAreaInvariant` (two parts with different native ref areas compose correctly only after both rescale to the shared `refArea`).

**Composite cross-check** (in AeroTests or a small AssemblyTests):
- `AssembledRocketCmMassAndRefArea` — nose+body+finset (+Motor) attached along z at real CM-to-CM offsets (exercising the cone's L/4 offset placement); composite mass = Σ; composite CM = independent mass-weighted hand calc; `deriveReferenceAreaFromGeometry()` == `π·(max getMaxRadius())²` and adding fins does NOT inflate it; manual `setReferenceArea` still wins.

**Regression (no code change):** all existing `qtrocket_*` suites pass unchanged (§6).

**Run**
```bash
cmake --build --preset debug-clang
ctest --test-dir build -R 'qtrocket_*'
./build/model/tests/model_tests --gtest_filter='NoseCone*:BodyTube*:FinSet*:Aero*:InertiaTensors*'
```

---

## 8. Implementation order (prove the math before the wrappers)

1. **`InertiaTensors` helpers first** (`SolidCone`, `ConicalShell`, `TrapezoidalFinSet`) + `InertiaTensorsTests.cpp` with the numeric/mesh oracles. Isolate the riskiest math (the N-fin tensor + corrected `Kz`) with **no** Part wrapper. Add the new `InertiaTensors.cpp` to CMake. Build `model_tests` green.
2. **`BodyTube`** (lowest risk; reuses `Tube` + existing tube oracle) + `BodyTubeTests.cpp` incl. the end-to-end composition cross-check. Establishes the part-authoring pattern end to end.
3. **`ConicalNoseCone`** (solid path first, then shell) + `NoseConeTests.cpp` — introduces the non-central CM (L/4) and the `coneCmOffset` placement subtlety; the disk-integration oracle pins the (3/80)L² term.
4. **`FinSet`** (heaviest analytics, on a proven helper base) + `FinSetTests.cpp` with the mesh oracle and Barrowman checks; N<3 warning path.
5. **`sim::Aero` replacement** (`AeroComponent`/`AeroProfile`) + `Part::getAero` default + `getCompositeAero` walk + the three per-part overrides + `AeroTests.cpp`. Pure data model; consumed nowhere, so it cannot regress trajectories.
6. **Reference-area-from-geometry plumbing** — `getReferenceArea()`/`getMaxRadius()` on each part (already in §3.B–D), `RocketModel::deriveReferenceAreaFromGeometry()` + the `referenceAreaOverridden` flag; the max-disc test. `getForces` math untouched.
7. **Wire `Parts.h` + both CMakeLists**; build `-Werror` (gcc+clang); run all four ctest suites; confirm existing physics/integration tests are bit-identical; CLI flight sanity check.

Each step builds `model_tests` green before the next, exactly as the P1 spec sequenced Part-first.

---

## 9. Risks & mitigations

| Risk | Mitigation |
|---|---|
| **FinSet tensor algebra wrong** (rotate+sum, sweep product, the corrected `Kz`) — the single biggest correctness risk | Acceptance gate is an **independent brute-force mesh oracle** (N=3,4,6), not the closed form; FinSet built last on a proven helper; closed form asserted *against* the mesh. |
| **plan-8 `Kz=(cr²+ct²)/24` is rectangle-only (verifier-confirmed wrong)** | Replaced with the corrected trapezoid chordwise second moment `(cr⁴+2cr³ct+2cr·ct³+ct⁴)/(18(cr+ct)²)` + sweep term; mesh oracle catches any residual error and prevents regression. |
| **FinSet as N rotated children would be physically wrong** (tree never rotates child tensors, Part.h:40-42) | Architectural rule: FinSet is ONE analytic leaf; rotate-and-sum is internal. Pinned by the isotropy + mesh tests. |
| **N<3 not axisymmetric, but the model forces N≥3** | FinSet **requires N≥3** (the supported domain). N<3 returns the N≥3 isotropic approximation and the ctor logs a warning (warn-don't-throw, kept for now); `FinCountBelow3Warns` pins the policy. True anisotropic N<3 is deferred to P6 per-part tensor rotation (TODO.md). |
| **Cone CM at L/4 (not mid-length) → wrong `addChildPart` position** | `coneCmOffset` reports the centroidal offset; the assembled-rocket composite CM test pins the placement; cone disk oracle pins the part-frame offset. |
| **Thin-shell hollow cone wrong for thick walls** | Solid is the default/first-class path; shell documented as thin-wall (t<<R) lateral-surface model; thick-wall frustum deferred. |
| **Reference area double-counted if summed** | Use the single max-radius frontal-disc rule; `AssembledRocketCmMassAndRefArea` asserts fins do not inflate it. |
| **Replacing `sim::Aero` breaks the build** | `Propagatable::aeroData` member exists (default-constructed, unread). Keep `sim/Aero.cpp` in the build and keep `sim::Aero`-replacement types default-constructible; do NOT delete the .cpp. No production code reads the new struct in P2. |
| **`-Werror` unused field/param** | Aero is RETURNED on demand, never stored (no field); `getAero` reads `refArea`; default base `getAero` marks the param `[[maybe_unused]]` exactly as `getMass(double t [[maybe_unused]])` does; no moment coeffs declared. |
| **Reference-area / Cd change alters existing trajectories** | P2 does NOT rewire `getForces`; geometry-derived area is default-but-manual-wins behind `referenceAreaOverridden`; placeholder rocket keeps `1.134e-3`. Regression suite must stay bit-identical. |
| **mm vs SI meters trap** | All three parts take SI meters; ctor validation ranges make an mm slip throw or fail a test; no `/1000` anywhere (unlike `Motor`). |
| **Fin mass centroid vs aero CP conflated** | Distinct formulas in distinct functions (`finSetCmOffset` vs `getAero`); documented; the Barrowman test uses the CP form, the CM test uses the mass form. |
| **`InertiaTensors` becomes non-header-only** | New `InertiaTensors.cpp` (for `TrapezoidalFinSet`) added to `model/CMakeLists.txt`; existing inline helpers stay in the header. |

---

## 10. Follow-ups this unblocks

- **P5 Barrowman pipeline**: `getForces` consumes `topPart->getCompositeAero(referenceArea).cd` (manual `dragCoefficient` wins); static margin `= (cp() − getCompositeCm(t).z)/diameter` shown live. The seam, the addition rule, and the geometry are all in place — no schema change.
- **P6 6-DOF torques**: `getTorques` reads `cp()` and `getCompositeCm(t)` for the CP–CG restoring moment; the moment coefficients (Cl/Cm/Cn) are added to the Aero types then.
- **P3 GUI/serialization**: each part owns its own scalar geometry (the Barrowman inputs) + getters; a polymorphic type tag + these scalars + children serialize directly; the 2-D side-view reads the same geometry. Transition, inner tubes, launch lugs, and stages all slot in as more leaves with zero tree churn.
