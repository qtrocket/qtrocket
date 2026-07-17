#ifndef MODEL_PARTS_PLACEMENT_H
#define MODEL_PARTS_PLACEMENT_H

/// \cond
// C++ headers
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>
/// \endcond

// qtrocket headers
#include "utils/math/MathTypes.h"

namespace model::part
{

// A resolved placement refers to its Part by non-owning pointer. This header must not include Part.h
// -- Part.h includes this one (for Station, returned by stationAt), so the dependency points one way.
class Part;

/// Mirror of Part::Id (Part.h), declared here so this header stays free of the Part hierarchy;
/// identical underlying type, so the two interconvert with no cast.
using PartId = std::uint64_t;

/**
 * @brief How a child seats against its parent. Closed by design: a rigid axisymmetric stack has only
 *        three radial relationships, and the seat both chooses the seam's radial check and fixes the
 *        gap's sign (insertion direction).
 */
enum class SeatKind : std::uint8_t
{
    Abut,        ///< rim-to-rim, same radius; gap is a forward standoff (signed +z)
    NestInBore,  ///< child OD seats inside parent ID; gap is insertion depth (child moves aft, -z)
    OnSurface    ///< child seats radially on parent's outer wall; gap is an axial standoff
};

/// SeatKind <-> its stable name ("Abut", "NestInBore", "OnSurface") -- one shared table so the file
/// format and the CLI agree by construction. fromString is nullopt for an unknown name (fail-closed).
std::string seatKindToString(SeatKind seat);
std::optional<SeatKind> seatKindFromString(const std::string& s);

/**
 * @brief Physical intent for one parent->child attachment -- the only spatial relationship stored.
 *
 * Stations are fractions of live length (0 = aft, 1 = fore), resolved against getLength() on every
 * read, so editing a part's length moves the seam. gap is in metres, meaning selected by seat (a
 * forward standoff for Abut/OnSurface, a non-negative insertion depth for NestInBore). childRot is
 * the per-seam orientation reserved for 6-DOF; identity in 3-DOF, so a stored link is bit-identical
 * to a pure-translation tree. A default link means "child fore plane abuts parent aft plane" -- the
 * natural nose->body->... build order.
 */
struct StationLink
{
    double     parentStation01{0.0};             ///< 0 = aft plane, 1 = fore plane, on the PARENT
    double     childStation01{1.0};              ///< 0 = aft plane, 1 = fore plane, on the CHILD
    double     gap{0.0};                         ///< meters; meaning per SeatKind
    SeatKind   seat{SeatKind::Abut};             ///< selects the radial check and the gap sign
    Quaternion childRot{Quaternion::Identity()}; ///< (x,y,z,w) seam orientation; identity in 3-DOF
};

/**
 * @brief A resolved axial landmark on a part: a derived quantity, never stored. Produced by
 *        Part::stationAt(station01) by mapping the fraction to z and reading the radius profile.
 */
struct Station
{
    double z{0.0};      ///< axial station, +z forward (m) = (station01 - 1) * getLength()
    double rOuter{0.0}; ///< outer radius at z (m)
    double rInner{0.0}; ///< inner radius at z (m); 0 for solids
};

/**
 * @brief A rigid pose in the root frame: a fore-plane origin on the axis plus an orientation.
 *        Orientation is identity in 3-DOF, so compose() degenerates to vector addition; carrying the
 *        quaternion gives a zero-schema path to 6-DOF for one identity multiply per child.
 */
struct Pose
{
    Vector3    origin{Vector3::Zero()};        ///< fore-plane origin, on axis, in the root frame
    Quaternion orient{Quaternion::Identity()}; ///< (x,y,z,w); identity in 3-DOF

    /// Place @p childInThis (in this pose's frame) into the root frame: rotate its translation by this
    /// orientation, then add -- the rigid-transform composition law (vector addition when identity).
    Pose compose(const Pose& childInThis) const
    {
        return Pose{origin + orient * childInThis.origin,
                        (orient * childInThis.orient).normalized()};
    }
};

/**
 * @brief A part paired with its resolved pose in the root frame. Non-owning. Returned in
 *        deterministic depth-first (attachment) order -- parent before child -- so the overlap
 *        sweep is reproducible and parentId always names an earlier entry.
 */
struct Placed
{
    const Part* part{nullptr};
    Pose        pose{};
    PartId      parentId{0};  ///< id of the seat parent in the resolved tree; 0 for the root
};

/**
 * @brief One located overlap violation. Parts are named by stable id (Part::getId()), never by name
 *        (not unique) or raw pointer (would dangle if the tree were edited between resolve and report).
 */
struct OverlapDiagnostic
{
    PartId      offender{0};      ///< the intruding part
    PartId      host{0};          ///< the part it intrudes into (may be a non-tree neighbour)
    double      zWorld{0.0};      ///< located station of worst violation (root frame, +z forward)
    double      penetration{0.0}; ///< metres the offender radius exceeds the host capacity
    std::string message{};
};

/**
 * @brief Verdict of the diagnostics layer for one structural resolve. When @p ok is false both
 *        consumers refuse the solve -- the composite pass throws, the mesh walk renders an error --
 *        so a self-intersecting design is a hard, located failure.
 */
struct SolveResult
{
    bool                           ok{true};
    std::vector<OverlapDiagnostic> diagnostics{};
};

// --- Snap-operator verbs -------------------------------------------------------------------------
// Each verb just produces a StationLink by value -- no pose stored, no part mutated. Sugar over the
// brace-initializer form, for when a relationship reads more clearly as a verb.

/// @brief Child fore plane against parent aft plane, with an optional forward standoff @p gap.
inline StationLink abut(double gap = 0.0)
{
    return StationLink{.parentStation01 = 0.0, .childStation01 = 1.0, .gap = gap,
                             .seat = SeatKind::Abut};
}

/// @brief Child aft plane inserted into the parent's fore bore to insertion @p depth (>= 0).
inline StationLink nestInBore(double depth)
{
    return StationLink{.parentStation01 = 1.0, .childStation01 = 0.0, .gap = depth,
                             .seat = SeatKind::NestInBore};
}

/// @brief Child aft plane seated on the parent's outer wall at fractional @p parentStation01.
inline StationLink seatOnWall(double parentStation01)
{
    return StationLink{.parentStation01 = parentStation01, .childStation01 = 0.0, .gap = 0.0,
                             .seat = SeatKind::OnSurface};
}

// --- The resolver -------------------------------------------------------------------------------
// The single authority for absolute placement: intent (StationLink) in, geometry (Pose) out. The
// tree walk lives with the tree (PartsModel resolves its node tree and caches the result); the
// per-edge geometry below is pure leaf math, so every consumer shares one resolve.

/// @brief Resolve one child's pose in the root frame from its parent's resolved pose and the link
///        binding them. Pure geometry: line the parent station up with the child station, apply the
///        seat-signed gap (childOriginZ = p.z + signedGap - c.z); coaxial in 3-DOF (x = y = 0).
Pose placeChild(const Pose& parentPose, const Part& parent, const Part& child,
                const StationLink& link);

// --- Diagnostics --------------------------------------------------------------------------------

/// @brief The radial seam check at one attachment, dispatched on SeatKind: equal outer radii for
///        Abut/OnSurface, child OD within parent bore for NestInBore. Returns a located diagnostic
///        when the mated radii are incompatible, else nullopt.
std::optional<OverlapDiagnostic> radialSeamCheck(const Part& parent, const Part& child,
                                                 const StationLink& link, double tol = 1e-9);

/// @brief Envelope sweep over a resolved tree. Builds a world axial interval per part, sorts them
///        O(N log N), samples each offender at feature breakpoints (its own and every other endpoint
///        within its span), and tests its outer radius against the smallest-id covering host's
///        capacity (the solid-host rule), excluding the offender, its descendants, and its direct
///        seat parent. Returns the worst (deepest) violation per offender/host pair.
SolveResult sweepOverlaps(const std::vector<Placed>& placed, double tol = 1e-9);

} // namespace model::part

#endif // MODEL_PARTS_PLACEMENT_H
