#include "model/parts/Part.h"

/// \cond
// C++ headers
#include <algorithm>
#include <atomic>
/// \endcond

namespace model::part
{

namespace
{
/// Process-wide source of unique Part ids. Atomic for concurrent construction (relaxed: we need only
/// uniqueness, not synchronization). Starts at 1, reserving 0 as "none/invalid".
std::atomic<Part::Id> nextPartId{1};
Part::Id makePartId() { return nextPartId.fetch_add(1, std::memory_order_relaxed); }

} // anonymous namespace

Part::Part(const std::string& n,
               const Matrix3& I,
               double m,
               const Vector3& centerMass)
    : id(makePartId()),
       name(n),
       inertiaTensor(I),
       mass(m),
       cm(centerMass)
{ }

Part::~Part()
{}

Part::Part(const Part& orig)
    // Copy for concrete clone() only: same mass properties, fresh id.
    : id(makePartId()),
       name(orig.name),
       inertiaTensor(orig.inertiaTensor),
       mass(orig.mass),
       cm(orig.cm)
{ }

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
