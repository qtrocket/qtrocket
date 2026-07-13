#include "model/parts/Part.h"
#include "utils/Logger.h"

/// \cond
// C++ headers
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
/// \endcond

namespace model::part
{

namespace
{
/// Parallel-axis displacement tensor f(d) = (d.d) I3 - d d^T. Times a body's mass and added to its CM
/// tensor, it shifts the tensor to a parallel axis offset by d. Even in d, so the sign of d is irrelevant.
Matrix3 parallelAxisTerm(const Vector3& d)
{
    return d.dot(d) * Matrix3::Identity() - d * d.transpose();
}

/// Process-wide source of unique Part ids. Atomic for concurrent construction (relaxed: we need only
/// uniqueness, not synchronization). Starts at 1, reserving 0 as "none/invalid".
std::atomic<Part::Id> nextPartId{1};
Part::Id makePartId() { return nextPartId.fetch_add(1, std::memory_order_relaxed); }

} // anonymous namespace

Part::Part(const std::string& n,
               const Matrix3& I,
               double m,
               const Vector3& centerMass)
    : parent(nullptr),
       id(makePartId()),
       name(n),
       inertiaTensor(I),
       // inertiaTensor is per-unit-mass (m^2); the composite is the full mass-weighted tensor (kg*m^2).
       compositeInertiaTensor(m * I),
       mass(m),
       compositeMass(m),
       cm(centerMass),
       // A childless part's composite CM coincides with its own CM, i.e. the zero offset.
       compositeCm(Vector3::Zero()),
       needsRecomputing(false),
       childParts()
{ }

Part::~Part()
{}

Part::Part(const Part& orig)
    // Shallow node copy used only by clone(): own mass properties, a fresh id, no parent, no children
    // (clone() deep-copies the sub-tree). See the class doc on copy/clone semantics.
    : parent(nullptr),
       id(makePartId()),
       name(orig.name),
       inertiaTensor(orig.inertiaTensor),
       compositeInertiaTensor(orig.compositeInertiaTensor),
       mass(orig.mass),
       compositeMass(orig.compositeMass),
       cm(orig.cm),
       compositeCm(orig.compositeCm),
       needsRecomputing(orig.needsRecomputing),
       childParts()
{ }

void Part::addChildPart(std::shared_ptr<Part> child, StationLink link)
{
    if(!child)
    {
        utils::Logger::getInstance()->error("Part::addChildPart: ignoring null child");
        return;
    }
    if(child->parent != nullptr)
    {
        utils::Logger::getInstance()->error(
            "Part::addChildPart: child already has a parent; clone() it or detach first");
        return;
    }
    // Reject attaching this part or any of its ancestors -- either would form a cycle (and an
    // ownership cycle through the child shared_ptrs).
    for(Part* p = this; p != nullptr; p = p->parent)
    {
        if(p == child.get())
        {
            utils::Logger::getInstance()->error(
                "Part::addChildPart: refusing to attach a part to itself or an ancestor (cycle)");
            return;
        }
    }

    // Adopt the child as-is (no copy, so its dynamic type and id are preserved) and re-parent it.
    child->parent = this;
    childParts.emplace_back(std::move(child), std::move(link));

    // A structural edit invalidates both caches up the tree: composite mass/CM/inertia and the resolved
    // placement.
    markAsNeedsRecomputing();
    markPlacementDirty();
}

std::shared_ptr<Part> Part::clone() const
{
    std::shared_ptr<Part> copy = cloneShallow(); // this node: correct dynamic type, fresh id, no kids
    for(const auto& [child, link] : childParts)
    {
        std::shared_ptr<Part> childCopy = child->clone(); // recurse polymorphically (no slicing)
        childCopy->parent = copy.get();
        copy->childParts.emplace_back(std::move(childCopy), link); // the placement intent clones verbatim
    }
    return copy;
}

double Part::getCompositeMass(double t)
{
    // Cheap live mass-only sum: this node plus every descendant. The ODE divisor and the gate key for
    // getCompositeI(t); does no tensor work.
    double m = getMass(t);
    for(const auto& [child, link] : childParts)
    {
        m += child->getCompositeMass(t);
    }
    return m;
}

void Part::ensureCompositeCache(double t)
{
    // Mass-delta gate. A structural edit (needsRecomputing) always rebuilds. Otherwise rebuild only when
    // the composite mass moved since the last build, so the tensor and CM recompute every step while a
    // motor burns and freeze once mass is constant (post-burnout getMass returns the bit-identical empty
    // mass, so mNow == builtAtCompositeMass exactly). The NaN sentinel forces the first call to build.
    const double mNow = getCompositeMass(t);
    if(needsRecomputing || mNow != builtAtCompositeMass)
    {
        const CompositeProperties c = computeCompositeAt(t);
        compositeMass          = c.mass;
        compositeCm            = c.cm;
        compositeInertiaTensor = c.inertia;
        builtAtCompositeMass   = mNow;
        needsRecomputing       = false;
    }
}

Vector3 Part::getCompositeCm(double t)
{
    ensureCompositeCache(t);
    return compositeCm;
}

Matrix3 Part::getCompositeI(double t)
{
    ensureCompositeCache(t);
    return compositeInertiaTensor;
}

void Part::ensurePlacementCache() const
{
    // Structural gate: re-resolve this sub-tree's geometry only when a part was added/removed or a length
    // changed (placementDirty), never on a mass change -- geometry is invariant under a burn.
    if(placementDirty)
    {
        resolvedCache = resolvePlacements(*this, Pose{}); // this part planted at the local origin
        // Run the Layer-2 envelope sweep once here and cache the verdict. Both consumers read it --
        // computeCompositeAt refuses a failed solve, the visualizer flags the offender -- so it costs
        // nothing per ODE step. Overlaps are invariant under a burn, so placementDirty alone gates it.
        resolvedDiagnostics = sweepOverlaps(resolvedCache);
        placementDirty = false;
    }
}

Part::CompositeProperties Part::computeCompositeAt(double t)
{
    // Geometry is resolved once per structural change (placement gate); here we re-weight it by
    // getMass(t). Each part's CM in the sub-tree-root frame is its resolved fore-plane origin plus the
    // local CM station (-L/2 + getCenterMassOffset().z()); +z = forward.
    ensurePlacementCache();

    // A self-intersecting design is a hard, located failure, never a silently-wrong mass/inertia. The
    // verdict is cached (computed once per structural resolve), so this is a flag read, not a re-sweep.
    if(!resolvedDiagnostics.ok)
    {
        const std::string detail = resolvedDiagnostics.diagnostics.empty()
                                                ? std::string("self-intersecting geometry")
                                                : resolvedDiagnostics.diagnostics.front().message;
        throw std::runtime_error("Part::computeCompositeAt: placement solve failed -- " + detail);
    }

    struct Contribution { Vector3 cmInRoot; const Part* part; double mass; };
    std::vector<Contribution> parts;
    parts.reserve(resolvedCache.size());

    // Pass 1: mass-weighted composite CM.
    double  m        = 0.0;
    Vector3 weighted = Vector3::Zero();
    for(const Placed& pl : resolvedCache)
    {
        const double  pm       = pl.part->getMass(t);
        const Vector3 cmOffset = pl.part->getCenterMassOffset();
        const Vector3 cmLocal(cmOffset.x(), cmOffset.y(),
                                     -pl.part->getLength() / 2.0 + cmOffset.z());
        const Vector3 cmInRoot = pl.pose.origin + pl.pose.orient * cmLocal;
        m        += pm;
        weighted += pm * cmInRoot;
        parts.push_back(Contribution{cmInRoot, pl.part, pm});
    }
    // Guard the divide: a fully massless sub-tree has no meaningful CM, so leave it at the origin.
    Vector3 temp_cm = Vector3::Zero();
    if(m > 0.0)
    {
        temp_cm = weighted / m;
    }

    // Pass 2: inertia about the composite CM. Shift each part's own mass-weighted tensor to temp_cm via
    // the parallel-axis theorem. 6-DOF would first rotate the tensor by pl.pose.orient (the R*I*R^T term,
    // identity today, omitted to stay bit-stable).
    // TODO(6-DOF): when that rotation is added, add a test pinning that it reduces to identity in 3-DOF.
    Matrix3 I = Matrix3::Zero();
    for(const Contribution& c : parts)
    {
        const Vector3 d = c.cmInRoot - temp_cm;
        I += c.mass * c.part->getI() + c.mass * parallelAxisTerm(d);
    }

    return CompositeProperties{m, temp_cm, I};
}

sim::AeroProfile Part::getCompositeAero(double refArea) const
{
    // Re-express every part's x_cp (reported from its own CM) onto the shared sub-tree-root (tip) datum:
    // the part's CM station = pose.origin.z + (-L/2 + getCenterMassOffset().z()). Adding cmStation back
    // cancels the part's own CM that cnAlphaXcp carried, so the composite cp depends on external shape
    // only, not mass distribution. cp() and cg() then share the tip datum, so cp() - cg() (the static
    // margin) is datum-independent.
    ensurePlacementCache();
    sim::AeroProfile profile;
    profile.refArea = refArea;
    for(const Placed& pl : resolvedCache)
    {
        const sim::AeroComponent c = pl.part->getAero(refArea);
        const double cmStationZ =
            pl.pose.origin.z() + (-pl.part->getLength() / 2.0 + pl.part->getCenterMassOffset().z());
        profile += sim::AeroComponent{c.cnAlpha, c.cnAlphaXcp + c.cnAlpha * cmStationZ, c.cd};
    }
    return profile;
}

double Part::maxFrontalReferenceArea() const
{
    double maxArea = getReferenceArea();
    for(const auto& [child, pos] : childParts)
    {
        maxArea = std::max(maxArea, child->maxFrontalReferenceArea());
    }
    return maxArea;
}

Part* Part::findById(Id targetId)
{
    if(id == targetId)
    {
        return this;
    }
    for(auto& [child, pos] : childParts)
    {
        if(Part* hit = child->findById(targetId))
        {
            return hit;
        }
    }
    return nullptr;
}

std::shared_ptr<Part> Part::removeChildById(Id targetId)
{
    // Symmetric with findById/addChildPart. Scan this node's direct children first: on a hit, move the
    // owning shared_ptr out, erase the (child, link) pair, clear the detached node's parent, and mark
    // this part and every ancestor dirty (both caches, so even a zero-mass sub-tree refreshes). Otherwise
    // recurse into the children.
    for(auto it = childParts.begin(); it != childParts.end(); ++it)
    {
        if(it->first->id == targetId)
        {
            std::shared_ptr<Part> detached = std::move(it->first);
            childParts.erase(it);
            detached->parent = nullptr;
            markAsNeedsRecomputing();
            markPlacementDirty();
            return detached;
        }
    }
    for(auto& [child, link] : childParts)
    {
        if(std::shared_ptr<Part> hit = child->removeChildById(targetId))
        {
            return hit;
        }
    }
    return nullptr;
}

Station Part::stationAt(double station01) const
{
    // Clamp the fraction (degenerate guard) and map it into the +z = forward local frame: station 1 is
    // the fore plane (z = 0, the origin), station 0 the aft plane (z = -length).
    const double s = std::clamp(station01, 0.0, 1.0);
    const double z = (s - 1.0) * getLength();
    return Station{z, radiusOuterAt(z), radiusInnerAt(z)};
}

double Part::innerCapacityAt(double zLocal) const
{
    // Solid-host rule: a solid part is bounded by its outer skin (the poke-through test); a bored part
    // by its inner wall. Branching here keeps the overlap sweep free of solidity special cases.
    return isSolid() ? radiusOuterAt(zLocal) : radiusInnerAt(zLocal);
}

} // namespace model::part
