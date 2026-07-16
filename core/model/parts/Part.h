#ifndef MODEL_PARTS_PART_H
#define MODEL_PARTS_PART_H

/// \cond
// C headers
// C++ headers
#include <vector>
#include <string>
#include <utility>
#include <memory>
#include <cstdint>
#include <limits>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/Aero.h"
#include "model/parts/Placement.h"   // Station (returned by stationAt), StationLink (used in childParts)
#include "utils/math/MathTypes.h"

namespace model::part
{

/**
 * @brief A node in a rocket's part tree: owns its mass, geometry-derived inertia, and CM, and
 *        aggregates those of all child parts. Each node is both one component and the root of a
 *        sub-tree, so it tracks its own mass/inertia and the composite of itself plus every descendant.
 *
 * The bare tensor (getI/setI) is per-unit-mass (geometric, m^2) for this part only; the composite
 * tensor (getCompositeI) is the full mass-weighted tensor (kg*m^2) of the sub-tree about the composite
 * CM, formed by shifting each child's tensor there via the parallel-axis theorem. Composite mass, CM,
 * and inertia come from one time-aware walk (computeCompositeAt) and are cached behind a mass-delta
 * gate, so they track a burning motor and freeze once mass is constant.
 *
 * The rocket is one rigid body -- parts never move relative to each other. In 3-DOF every part's body
 * frame is the identity, so child offsets are pure translations and tensors add; the placement layer
 * already carries a per-part orientation (StationLink::childRot, Pose::orient) for 6-DOF, identity
 * today. The one piece still assuming identity is the inertia shift, which parallel-axis-shifts each
 * child tensor without yet rotating it (the R*I*R^T term, identity in 3-DOF; see computeCompositeAt).
 */
class Part
{
    /// Test-only: lets the composition tests reach the parent pointers, child list, and dirty flag to
    /// verify clone re-parenting and upward dirty propagation. Defined in PartTests.cpp.
    friend class PartCompositionTestAccess;

public:
    /// Stable per-instance identifier type. @see getId()
    using Id = std::uint64_t;

    /**
     * @brief Construct a leaf part from its mass properties.
     * @param name       part name (identifies child parts within a tree)
     * @param I          per-unit-mass (geometric) inertia tensor about the part's CM (m^2)
     * @param m          part mass (kg)
     * @param centerMass CM relative to the component middle (stored as `cm`)
     */
    Part(const std::string& name,
           const Matrix3& I,
           double m,
           const Vector3& centerMass);

    virtual ~Part();

    // ---- Non-copyable / non-movable at the value level ---------------------------------------
    // A part lives at one place in one tree: attached by moving ownership into addChildPart(),
    // duplicated only via clone(). Deleting assignment also prevents slicing a subclass to a base Part;
    // the protected copy ctor (below) suppresses the implicit move, so a Part can't be moved either.
    Part& operator=(const Part&) = delete;
    Part& operator=(Part&&)      = delete;

    /// Set this part's own mass (kg); flags this part and all ancestors for recompute.
    virtual void setMass(double m) { mass = m; markAsNeedsRecomputing(); }

    /// Set the per-unit-mass (geometric) inertia tensor about this part's CM (m^2); flags for recompute.
    virtual void setI(const Matrix3& I) { inertiaTensor = I; markAsNeedsRecomputing(); }
    /// Per-unit-mass (geometric) inertia tensor (m^2). @see getCompositeI()
    virtual Matrix3 getI() const { return inertiaTensor; }

    /// CM relative to the component middle. Zero for symmetric parts, non-zero for a cone (CM at L/4 or
    /// L/3 from the base). Consumed by the composite walk as `-L/2 + getCenterMassOffset().z()`.
    Vector3 getCenterMassOffset() const { return cm; }

    /// The composite mass, CM (== CG), and full inertia tensor of a sub-tree at one instant -- only
    /// meaningful together, since the CM is the point the tensor is taken about.
    struct CompositeProperties
    {
        double  mass{0.0};                 ///< composite mass at t (kg)
        Vector3 cm{Vector3::Zero()};       ///< composite CM (== CG) relative to this part's own CM
        Matrix3 inertia{Matrix3::Zero()};  ///< full mass-weighted tensor (kg*m^2) about that CM
    };

    /// This part's own mass at simulation time @p t (kg). Lets overrides model time-varying mass (e.g. a motor).
    virtual double getMass(double t [[maybe_unused]]) const
    {
        return mass;
    }

    /// Composite mass of this part plus all children at @p t (kg): a cheap live mass-only sum (no tensor
    /// work). Both the ODE divisor and the cache gate key for getCompositeI(t); reflects a Motor.
    virtual double getCompositeMass(double t);

    /// Composite CM (== CG) at @p t, relative to this part's own CM (zero for a leaf). The point
    /// getCompositeI(t) is taken about; shifts as a child's mass changes. Same cache as getCompositeI(t).
    virtual Vector3 getCompositeCm(double t);

    /// Full mass-weighted composite inertia tensor (kg*m^2) about the composite CM at @p t. Rebuilt only
    /// when the tree is structurally dirty or the composite mass changed since the last build (the
    /// mass-delta gate), so it recomputes during a burn and freezes once mass is constant. Keying on
    /// mass keeps the cache immune to the integrator's repeated/rejected stage-time queries.
    virtual Matrix3 getCompositeI(double t);

    /// This part's Barrowman aero contribution, normalized to the shared reference area @p refArea.
    /// Default is aerodynamically inert (CNalpha = 0). x_cp is reported from this part's own CM (see
    /// model::AeroComponent). Pure function of geometry; not stored.
    virtual model::AeroComponent getAero(double refArea [[maybe_unused]]) const { return {}; }

    /// Assemble the composite Barrowman profile over this sub-tree, normalized to @p refArea, by folding
    /// each part's getAero(refArea) additively. Threads each part's resolved axial station so every x_cp
    /// shares the sub-tree-root (tip) datum -- the same datum as getCompositeCm(), so cp() - cg() is the
    /// static margin. A separate pass from computeCompositeAt: it reads no time-varying state.
    model::AeroProfile getCompositeAero(double refArea) const;

    /// Cached envelope-sweep verdict for this sub-tree (resolved once per structural change). @c ok ==
    /// false means the geometry self-intersects and @c diagnostics locate each offender -- the same
    /// verdict the composite gate throws on, exposed so the visualizer can flag offenders.
    const SolveResult& placementDiagnostics() const { ensurePlacementCache(); return resolvedDiagnostics; }

    /// This part's axial length L (m): it occupies z in [-L, 0], +z = forward. Base is 0 (a
    /// geometrically inert / zero-length node, e.g. the test-only base Part or a Motor); every concrete
    /// geometry type overrides it.
    virtual double getLength() const { return 0.0; }

    /// Outer radius (m) of this part's silhouette at local station @p zLocal (z in [-length, 0]). Base
    /// is 0; concrete types give the closed-form profile (cone tapers, tube is constant, sphere bulges).
    /// Callable independently of stationAt() -- the overlap sweep samples it across a span.
    virtual double radiusOuterAt(double zLocal [[maybe_unused]]) const { return 0.0; }

    /// Inner (bore) radius (m) at local station @p zLocal; 0 for a solid part. A BodyTube returns its
    /// constant inner wall, a HollowSphere its shell cavity.
    virtual double radiusInnerAt(double zLocal [[maybe_unused]]) const { return 0.0; }

    /// This part's axial span (m): it occupies z in [-axialLength(), 0]. Defaults to getLength(); the
    /// override point for a part whose envelope span differs from its nominal length.
    virtual double axialLength() const { return getLength(); }

    /// Whether this part is solid (no bore). Base infers it from the absence of a bore at the fore plane
    /// (radiusInnerAt(0) <= 0). Parts whose bore vanishes only at the poles (a HollowSphere shell) or
    /// whose solidity is authored (a shell cone) override it so innerCapacityAt() picks the right edge.
    virtual bool isSolid() const { return radiusInnerAt(0.0) <= 0.0; }

    /// Resolved axial landmark (z and radii) at fractional station @p station01 (0 = aft plane, 1 = fore
    /// plane), in this part's +z = forward frame. Clamped to [0,1]; maps to z = (station01 - 1) *
    /// getLength() and reads the radius profile there. Symmetric parts need no override.
    virtual Station stationAt(double station01) const;

    /// Radius (m) a host occupies at local station @p zLocal -- the solid-host rule: the bore for a
    /// bored part, the outer skin for a solid one. An offender of outer radius r fits iff r <= this + tol.
    double innerCapacityAt(double zLocal) const;

    /// This part's own aerodynamic reference (frontal) area (m^2); 0 for a part with no frontal disc.
    /// @see maxFrontalReferenceArea
    virtual double getReferenceArea() const { return 0.0; }

    /// The largest getReferenceArea() over this part and all descendants (m^2) -- the widest frontal
    /// disc. The Barrowman/OpenRocket reference area: the max disc, not a sum, and not inflated by fins
    /// (a FinSet reports the body disc, not its tip extent).
    double maxFrontalReferenceArea() const;

    /// This part's unique id (per process run, even across copies and identical names). Assigned at
    /// construction and never changed; a copy gets a new id. Use it, not the name, to identify a part.
    /// Not serialized -- ids are only meaningful within a single run.
    Id getId() const { return id; }

    /// This part's human-facing name; need not be unique (use getId() for identity).
    std::string getName() const { return name; }

    /// Stable type tag ("NoseCone", "BodyTube", ...). Pure: every concrete part defines its own. Doubles
    /// as the part-factory key, the design-file <part type=...> attribute, and the listparts label.
    virtual std::string typeName() const = 0;

    /// Find a part by @p targetId within this sub-tree (this part or any descendant). Returns a borrowed
    /// pointer (valid while the tree lives), or nullptr if no match. Call on the root to search a rocket.
    Part* findById(Id targetId);

    /// Read-only view of this part's direct children paired with their StationLink (placement intent),
    /// in attachment order. Absolute placement is derived by resolvePlacements, never stored. @see addChildPart.
    const std::vector<std::pair<std::shared_ptr<Part>, StationLink>>& getChildParts() const
    { return childParts; }

    /// Deep-copy this part and its whole sub-tree into a new, independent tree. Type-preserving; every
    /// cloned node gets a fresh id and the returned root has no parent. The only way to duplicate a part.
    std::shared_ptr<Part> clone() const;

    /**
     * @brief Attach an existing part as a child by transferring ownership (no copy, so dynamic type and
     *        id are preserved). Flags this part and all ancestors dirty (mass and placement). Logged
     *        no-op if @p child is null, already parented, or would form a cycle.
     * @param child part to adopt; the shared_ptr is moved from
     * @param link  station-pair placement intent; the default abuts child fore plane to parent aft plane
     */
    virtual void addChildPart(std::shared_ptr<Part> child, StationLink link = {});

    /// Detach the descendant with @p targetId and return it (the caller owns the now-rootless sub-tree),
    /// or nullptr if absent. Flags the ex-parent and all ancestors for recompute, so the next read
    /// rebuilds even when the removed sub-tree's mass was zero. The root has no parent and is never removed here.
    std::shared_ptr<Part> removeChildById(Id targetId);

    /// Borrowed pointer to this part's parent, or nullptr for the tree root. Lets display widgets
    /// (e.g. a QTreeView model) walk back up the tree.
    Part* getParent() const { return parent; }

protected:
    /// Shallow node copy for clone()/cloneShallow() only: copies this part's own mass properties (not
    /// children) and assigns a fresh id, with no parent. Protected so external code can't copy or slice.
    Part(const Part&);

    /// Type-preserving shallow copy of just this node (no children). Pure: each concrete subclass
    /// overrides it so clone() reproduces the right dynamic type. @see clone()
    virtual std::shared_ptr<Part> cloneShallow() const = 0;

private:

    /// Non-owning pointer to the parent, if any. Used to propagate recompute flags up the tree.
    Part* parent{nullptr};

    /// Unique per-instance id (see getId()). Assigned fresh in every constructor, including the copy
    /// ctor, and left untouched by assignment so a part keeps its identity when overwritten.
    Id id;

    std::string name; ///< human-facing label; need not be unique. Use id to identify a part.

    /// The single time-aware walk behind every composite accessor. Pass 1 accumulates composite mass and
    /// CM from getMass(t); pass 2 sums the parallel-axis-shifted child tensors about that CM.
    CompositeProperties computeCompositeAt(double t);

    /// Mass-delta gate: rebuild the cached compositeCm/compositeInertiaTensor via computeCompositeAt(t)
    /// iff structurally dirty or the composite mass moved since the last build, then clear the flag.
    void ensureCompositeCache(double t);

    /// Flag this part and every ancestor as needing a composite recompute.
    void markAsNeedsRecomputing()
    { needsRecomputing = true; if(parent) { parent->markAsNeedsRecomputing(); }}

    // A part stores both its own inertia tensor (without children) and the composite one (with them).
    Matrix3 inertiaTensor;          ///< per-unit-mass (geometric) tensor about this part's CM (m^2)
    Matrix3 compositeInertiaTensor; ///< full mass-weighted tensor of this part + children (kg*m^2)
    double mass;          ///< this part's own mass (kg)
    double compositeMass; ///< mass of this part plus all children (kg)

    /// Composite mass the cached CM/tensor were last built at -- the mass-delta gate key. NaN sentinel
    /// forces the first build. Compared exact == against getCompositeMass(t): post-burnout that value
    /// repeats bit-for-bit so the cache freezes; during a burn it differs every step.
    double builtAtCompositeMass{std::numeric_limits<double>::quiet_NaN()};

    /// CM relative to the component middle. Consumed by the composite walk as `-L/2 +
    /// getCenterMassOffset().z()` (computeCompositeAt / getCompositeAero); placement itself is geometric
    /// (StationLink), not CM-based. Set once at construction; radial components await 6-DOF force application.
    Vector3 cm;

    /// Composite CM of this part plus all descendants, relative to this part's own CM (zero for a leaf).
    /// The point getCompositeI()/compositeInertiaTensor is taken about.
    Vector3 compositeCm;

    bool needsRecomputing{false}; ///< true when the cached composite quantities are stale

    /// True when the resolved placement cache is stale. Set by a structural/geometry edit
    /// (addChildPart/removeChildById) and propagated up the tree; not set by a pure mass/inertia edit.
    /// Starts true so the first read resolves once. mutable so const readers can memoize through it.
    mutable bool placementDirty{true};

    /// Cached resolver output for this sub-tree (this part at the local origin), rebuilt by
    /// ensurePlacementCache only when placementDirty. computeCompositeAt re-weights it by getMass(t)
    /// every step, so geometry resolves once per structural change while mass tracks a burn every step.
    mutable std::vector<Placed> resolvedCache;

    /// Cached envelope-sweep verdict for resolvedCache, rebuilt alongside it (once per structural
    /// resolve). ok == false means the sub-tree self-intersects: computeCompositeAt refuses it and the
    /// visualizer flags the offender, so the two consumers can't disagree.
    mutable SolveResult resolvedDiagnostics;

    /// Resolve this sub-tree's placements into resolvedCache iff placementDirty, then clear the flag.
    void ensurePlacementCache() const;

    /// Flag this part and every ancestor as needing a placement re-resolve.
    void markPlacementDirty()
    { placementDirty = true; if(parent) { parent->markPlacementDirty(); }}

    /// Child parts paired with their stored placement intent (StationLink). Absolute pose is derived by
    /// resolvePlacements, never stored here.
    std::vector<std::pair<std::shared_ptr<Part>, StationLink>> childParts;
};

} // namespace model::part

#endif // MODEL_PARTS_PART_H
