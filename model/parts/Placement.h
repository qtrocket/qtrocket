#ifndef MODEL_PARTS_PLACEMENT_H
#define MODEL_PARTS_PLACEMENT_H

/// \cond
// C++ headers
#include <cstdint>
#include <string>
#include <vector>
/// \endcond

// qtrocket headers
#include "utils/math/MathTypes.h"

namespace model::part
{

// Forward declaration: a resolved placement refers to its Part by non-owning pointer. This header
// must NOT include Part.h -- Part.h will include THIS header (for StationLink inside childParts),
// so the dependency points one way only and there is no include cycle.
class Part;

/// @brief Mirror of Part::Id (model/parts/Part.h). Declared here independently so this header stays
///        free of the Part hierarchy; identical underlying type, so a Part::Id and a PartId
///        interconvert with no cast.
using PartId = std::uint64_t;

/**
 * @brief How a child seats against its parent.
 *
 * Deliberately CLOSED: a rigid axisymmetric stack expresses exactly three radial relationships --
 * rim-to-rim of equal radius (Abut), one OD inside another's ID (NestInBore), and a part on
 * another's outer wall (OnSurface). The seat has two jobs: (a) choose the radial compatibility
 * CHECK at the seam, and (b) fix the SIGN of the gap (insertion direction). There is no fourth
 * relationship for the parts QtRocket models, so leaving room to extend would only invite
 * half-defined cases.
 */
enum class SeatKind : std::uint8_t
{
   Abut,        ///< rim-to-rim, same radius; gap is a forward standoff (signed +z)
   NestInBore,  ///< child OD seats inside parent ID; gap is insertion DEPTH (child moves aft, -z)
   OnSurface    ///< child seats radially on parent's outer wall; gap is an axial standoff
};

/**
 * @brief Physical intent for one parent->child attachment -- the only spatial relationship stored.
 *
 * Stations are FRACTIONS of live length (0 = aft plane, 1 = fore plane), resolved against
 * getLength() on every read, so editing a part's length moves the seam. @p gap is in metres; its
 * meaning is selected by @p seat (a forward standoff for Abut/OnSurface, a non-negative insertion
 * depth for NestInBore). @p childRot is the per-seam orientation reserved for 6-DOF; it is the
 * identity in 3-DOF, so a stored link is bit-identical to a pure-translation tree and activating
 * 6-DOF costs no schema reshape.
 *
 * A default-constructed StationLink means "child fore plane abuts parent aft plane" -- the natural
 * nose->body->... build order, costing zero authored numbers.
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
 *
 * Orientation is the identity in 3-DOF, so compose() degenerates to vector addition and the
 * resolved tree is bit-identical to the legacy pure-translation walk. Carrying the quaternion from
 * day one costs one identity multiply per child and buys a zero-schema path to 6-DOF.
 */
struct Pose
{
   Vector3    origin{Vector3::Zero()};        ///< fore-plane origin, on axis, in the ROOT frame
   Quaternion orient{Quaternion::Identity()}; ///< (x,y,z,w); identity in 3-DOF

   /// @brief Place @p childInThis (expressed in THIS pose's frame) into the root frame. Rotates the
   ///        child translation by this orientation BEFORE adding it -- the exact rigid-transform
   ///        composition law. With identity orientations everywhere this reduces to vector addition.
   Pose compose(const Pose& childInThis) const
   {
      return Pose{origin + orient * childInThis.origin,
                  (orient * childInThis.orient).normalized()};
   }
};

/**
 * @brief A part paired with its resolved pose in the root frame. Non-owning. The resolver returns
 *        these in deterministic depth-first (attachment) order, giving the overlap sweep a stable
 *        ordering and making the resolved sequence reproducible across runs.
 */
struct Placed
{
   const Part* part{nullptr};
   Pose        pose{};
};

/**
 * @brief One located overlap violation found by the diagnostics layer.
 *
 * Parts are named by stable id (Part::getId()), never by name (names need not be unique) or by raw
 * pointer (which would dangle if the tree were edited between resolve and report).
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
 * @brief The verdict of the diagnostics layer for one structural resolve. When @p ok is false both
 *        downstream consumers refuse the solve -- the composite pass throws and the mesh walk
 *        renders an error -- so a self-intersecting design is a hard, located failure, not a silent
 *        render.
 */
struct SolveResult
{
   bool                           ok{true};
   std::vector<OverlapDiagnostic> diagnostics{};
};

// --- Snap-operator verbs -------------------------------------------------------------------------
// Each verb is a pure factory: it PRODUCES a StationLink by value -- it stores no pose, mutates no
// part, and fixes no coordinate. All resolution from intent to absolute placement happens later, in
// the one resolver. They add no expressive power over the brace-initializer form; they exist so an
// author may write a relationship as a verb when that reads more clearly.

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

} // namespace model::part

#endif // MODEL_PARTS_PLACEMENT_H
