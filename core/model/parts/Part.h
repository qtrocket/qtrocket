#ifndef MODEL_PARTS_PART_H
#define MODEL_PARTS_PART_H

/// \cond
// C headers
// C++ headers
#include <cstdint>
#include <memory>
#include <string>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/Aero.h"
#include "model/parts/Placement.h"   // Station (returned by stationAt)
#include "utils/math/MathTypes.h"

namespace model { class PartsModel; }

namespace model::part
{

/**
 * @brief One rocket component: its own mass, geometry-derived inertia, CM, silhouette profile, and
 *        aero contribution. A pure leaf.
 *
 * Tree structure, placement, and every composite (mass-weighted) quantity live in model::PartsModel,
 * which owns each Part through its nodes. The bare tensor here (getI) is per-unit-mass (geometric,
 * m^2) about this part's own CM; the composite kg*m^2 aggregation is the node's job. Post-attach
 * mutation goes only through PartsModel's routed verbs -- the setters are private to that seam --
 * so cache invalidation cannot be skipped by construction.
 */
class Part
{
public:
    /// Stable per-instance identifier type. @see getId()
    using Id = std::uint64_t;

    /**
     * @brief Construct a part from its mass properties.
     * @param name       part name (human-facing; ids are the identity)
     * @param I          per-unit-mass (geometric) inertia tensor about the part's CM (m^2)
     * @param m          part mass (kg)
     * @param centerMass CM relative to the component middle (stored as `cm`)
     */
    Part(const std::string& name,
           const Matrix3& I,
           double m,
           const Vector3& centerMass);

    virtual ~Part();

    // A part is a unique-ownership value: duplicated only via clone(). Deleting assignment also
    // prevents slicing a subclass to a base Part; the protected copy ctor suppresses implicit move.
    Part& operator=(const Part&) = delete;
    Part& operator=(Part&&)      = delete;

    /// Per-unit-mass (geometric) inertia tensor about this part's CM (m^2).
    virtual Matrix3 getI() const { return inertiaTensor; }

    /// CM relative to the component middle. Zero for symmetric parts, non-zero for a cone (CM at L/4 or
    /// L/3 from the base). Consumed by the composite walk as `-L/2 + getCenterMassOffset().z()`.
    Vector3 getCenterMassOffset() const { return cm; }

    /// This part's own mass at simulation time @p t (kg). Lets overrides model time-varying mass (e.g. a motor).
    virtual double getMass(double t [[maybe_unused]]) const
    {
        return mass;
    }

    /// This part's Barrowman aero contribution, normalized to the shared reference area @p refArea.
    /// Default is aerodynamically inert (CNalpha = 0). x_cp is reported from this part's own CM (see
    /// model::AeroComponent). Pure function of geometry; not stored.
    virtual model::AeroComponent getAero(double refArea [[maybe_unused]]) const { return {}; }

    /// This part's axial length L (m): it occupies z in [-L, 0], +z = forward. Base is 0 (a
    /// geometrically inert / zero-length node, e.g. a Motor); every concrete geometry type overrides it.
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
    virtual double getReferenceArea() const { return 0.0; }

    /// This part's unique id (per process run, even across copies and identical names). Assigned at
    /// construction and never changed; a copy gets a new id. Use it, not the name, to identify a part.
    /// Not serialized -- ids are only meaningful within a single run.
    Id getId() const { return id; }

    /// This part's human-facing name; need not be unique (use getId() for identity).
    std::string getName() const { return name; }

    /// Stable type tag ("NoseCone", "BodyTube", ...). Pure: every concrete part defines its own. Doubles
    /// as the part-factory key, the design-file <part type=...> attribute, and the listparts label.
    virtual std::string typeName() const = 0;

    /// Type-preserving copy of this part with a fresh id. Pure: each concrete part implements it via
    /// its protected copy ctor. The only way to duplicate a part.
    virtual std::unique_ptr<Part> clone() const = 0;

protected:
    /// Copy for concrete clone() implementations only: same mass properties, fresh id. Protected so
    /// external code can't copy or slice.
    Part(const Part&);

private:
    friend class model::PartsModel;  // the sole post-attach mutation seam

    /// Plain writes: PartsModel's routed verbs own cache invalidation.
    void setMass(double m) { mass = m; }
    void setI(const Matrix3& I) { inertiaTensor = I; }

    /// Unique per-instance id (see getId()). Assigned fresh in every constructor, including the copy
    /// ctor.
    Id id;

    std::string name; ///< human-facing label; need not be unique. Use id to identify a part.

    Matrix3 inertiaTensor; ///< per-unit-mass (geometric) tensor about this part's CM (m^2)
    double mass;           ///< this part's own mass (kg)

    /// CM relative to the component middle. Consumed by the node composite walk as `-L/2 +
    /// getCenterMassOffset().z()`; placement itself is geometric (StationLink), not CM-based. Set once
    /// at construction; radial components await 6-DOF force application.
    Vector3 cm;
};

} // namespace model::part

#endif // MODEL_PARTS_PART_H
