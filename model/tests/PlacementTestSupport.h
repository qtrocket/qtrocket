#ifndef MODEL_TESTS_PLACEMENTTESTSUPPORT_H
#define MODEL_TESTS_PLACEMENTTESTSUPPORT_H

// Test-only placement convenience. Several composition / aero / motor tests pin their expected values
// to a geometric-center-to-center placement: "the child's CM sits gapZ along +z from the parent's CM."
// With the CM-to-CM authoring shim retired (production now speaks StationLink only), this helper
// re-expresses that intent as an explicit StationLink so those tests keep exercising the exact same
// resolved geometry. It is NOT a production path -- it just joins the parent's CM station to the child's
// CM station with an axial gap, which the resolver turns back into the same fore-plane origin the old
// CM-to-CM offset produced (childOriginZ = parentCmLocalZ - childCmLocalZ + gapZ).

#include "model/parts/Part.h"
#include "model/parts/Placement.h"

namespace model::part::test
{
/// Fractional station (0 = aft, 1 = fore) of a part's own CM. A zero-length point mass has its CM at
/// the origin for any station, so 0 is as good as any (the resolved origin depends only on the gap).
inline double cmStation(const Part& p)
{
   const double L = p.getLength();
   return (L > 0.0) ? 0.5 + p.getCenterMassOffset().z() / L : 0.0;
}

/// The StationLink that places @p child's CM @p gapZ along +z from @p parent's CM -- the explicit form
/// of the legacy geometric-center-to-center offset, reproduced exactly through the resolver.
inline StationLink cmToCm(const Part& parent, const Part& child, double gapZ)
{
   return StationLink{.parentStation01 = cmStation(parent), .childStation01 = cmStation(child),
                      .gap = gapZ, .seat = SeatKind::Abut};
}
} // namespace model::part::test

#endif // MODEL_TESTS_PLACEMENTTESTSUPPORT_H
