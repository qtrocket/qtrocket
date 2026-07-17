#include "model/parts/Placement.h"

/// \cond
// C++ headers
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <tuple>
#include <utility>
#include <vector>
/// \endcond

// qtrocket headers
#include "model/parts/Part.h"  // the geometry virtuals (stationAt / radius*At / innerCapacityAt)

namespace model::part
{

std::string seatKindToString(SeatKind seat)
{
   switch(seat)
   {
      case SeatKind::Abut:       return "Abut";
      case SeatKind::NestInBore: return "NestInBore";
      case SeatKind::OnSurface:  return "OnSurface";
   }
   return "Abut";
}

std::optional<SeatKind> seatKindFromString(const std::string& s)
{
   if(s == "Abut")       { return SeatKind::Abut; }
   if(s == "NestInBore") { return SeatKind::NestInBore; }
   if(s == "OnSurface")  { return SeatKind::OnSurface; }
   return std::nullopt;
}

Pose placeChild(const Pose& parentPose, const Part& parent, const Part& child,
                     const StationLink& link)
{
    const Station p = parent.stationAt(link.parentStation01);  // landmark on the parent
    const Station c = child.stationAt(link.childStation01);    // landmark on the child

    // signedGap encodes seat direction in +z = forward: NestInBore inserts aft (-z, gap is depth),
    // Abut/OnSurface stand off forward (+z). The author always writes a non-negative number.
    const double signedGap = (link.seat == SeatKind::NestInBore) ? -link.gap : link.gap;

    // Line the child's station up with the parent's, then displace by the gap. The pose locates the
    // child's fore plane, which sits c.z from its chosen station -- hence the `- c.z`.
    const double childOriginZ = p.z + signedGap - c.z;

    const Pose childInParent{Vector3(0.0, 0.0, childOriginZ), link.childRot};  // coaxial in 3-DOF
    return parentPose.compose(childInParent);
}

std::optional<OverlapDiagnostic> radialSeamCheck(const Part& parent, const Part& child,
                                                                 const StationLink& link, double tol)
{
    const Station p = parent.stationAt(link.parentStation01);
    const Station c = child.stationAt(link.childStation01);

    double      penetration = 0.0;
    const char* kind        = nullptr;
    switch(link.seat)
    {
        case SeatKind::Abut:        // rim-to-rim: the two outer radii must match
            if(std::fabs(p.rOuter - c.rOuter) > tol) { penetration = std::fabs(p.rOuter - c.rOuter); kind = "Abut rim radius mismatch"; }
            break;
        case SeatKind::OnSurface:   // child seats on the parent's outer wall: outer radii must match
            if(std::fabs(c.rOuter - p.rOuter) > tol) { penetration = std::fabs(c.rOuter - p.rOuter); kind = "OnSurface wall radius mismatch"; }
            break;
        case SeatKind::NestInBore:  // child OD must fit inside the parent bore
            if(c.rOuter > p.rInner + tol) { penetration = c.rOuter - p.rInner; kind = "NestInBore: child OD exceeds parent bore"; }
            break;
    }
    if(kind == nullptr)
    {
        return std::nullopt;
    }

    char msg[160];
    std::snprintf(msg, sizeof(msg), "%s (child r=%.5g m, parent r=%.5g m, over by %.5g m)", kind,
                       c.rOuter, (link.seat == SeatKind::NestInBore) ? p.rInner : p.rOuter, penetration);
    OverlapDiagnostic d;
    d.offender    = child.getId();
    d.host        = parent.getId();
    d.zWorld      = p.z;  // parent-local seam station (the root-frame z is added once a pose is known)
    d.penetration = penetration;
    d.message     = msg;
    return d;
}

namespace
{
/// A part's resolved world axial interval [zAft, zFore] (3-DOF: orientation is the identity, so the
/// local span [-axialLength, 0] maps to world by adding the fore-plane origin's z).
struct Interval
{
    const Part* part{nullptr};
    PartId      id{0};
    double      originZ{0.0};  // world z of the fore plane (local 0); world z -> local z is `- originZ`
    double      zAft{0.0};
    double      zFore{0.0};
};
}  // namespace

SolveResult sweepOverlaps(const std::vector<Placed>& placed, double tol)
{
    SolveResult result;

    // 1. World intervals.
    std::vector<Interval> intervals;
    intervals.reserve(placed.size());
    for(const Placed& pl : placed)
    {
        const double originZ = pl.pose.origin.z();
        const double span    = pl.part->axialLength();
        intervals.push_back(Interval{pl.part, pl.part->getId(), originZ, originZ - span, originZ});
    }

    // Tree structure (childId -> parentId), read from Placed::parentId, to exclude the legit
    // neighbours of an offender: itself, its descendants, and its direct seat parent. The sweep
    // needs no access to the ownership tree itself.
    std::map<PartId, PartId> parentOf;
    for(const Placed& pl : placed)
    {
        if(pl.parentId != 0)
        {
            parentOf[pl.part->getId()] = pl.parentId;
        }
    }
    const auto isDescendantOf = [&](PartId h, PartId ancestor)
    {
        for(auto it = parentOf.find(h); it != parentOf.end(); it = parentOf.find(it->second))
        {
            if(it->second == ancestor) { return true; }
        }
        return false;
    };
    const auto excludedHost = [&](PartId h, PartId offender)
    {
        if(h == offender) { return true; }              // self
        if(isDescendantOf(h, offender)) { return true; } // offender contains h (legit nesting)
        const auto it = parentOf.find(offender);
        return it != parentOf.end() && it->second == h;  // h is the offender's direct seat parent
    };

    // 2. Sort by aft station for the active-set host lookup (O(N log N)).
    std::vector<Interval> byAft = intervals;
    std::sort(byAft.begin(), byAft.end(),
                 [](const Interval& a, const Interval& b) { return a.zAft < b.zAft; });

    // Worst (deepest) violation per (offender, host) pair.
    std::map<std::pair<PartId, PartId>, OverlapDiagnostic> worst;

    for(const Interval& off : intervals)
    {
        // 3. Feature breakpoints: the offender's own endpoints plus every other endpoint within its span
        //    (profiles are closed-form and monotone between breakpoints, so this is exact, not sampled).
        std::vector<double> bps{off.zAft, off.zFore};
        for(const Interval& other : intervals)
        {
            if(other.id == off.id) { continue; }
            for(const double e : {other.zAft, other.zFore})
            {
                if(e >= off.zAft - tol && e <= off.zFore + tol) { bps.push_back(e); }
            }
        }
        std::sort(bps.begin(), bps.end());
        bps.erase(std::unique(bps.begin(), bps.end(),
                                     [](double a, double b) { return std::fabs(a - b) <= 1e-12; }),
                     bps.end());

        for(const double zWorld : bps)
        {
            // Smallest-id covering host, excluding the offender's legit neighbours.
            const Interval* host = nullptr;
            for(const Interval& cand : byAft)
            {
                if(cand.zAft > zWorld + tol) { break; }              // sorted: no later interval covers
                if(cand.zFore < zWorld - tol) { continue; }          // ended before zWorld
                if(cand.zFore - cand.zAft <= tol) { continue; }      // zero-span node (a Motor): no interior, hosts nothing
                if(excludedHost(cand.id, off.id)) { continue; }
                if(host == nullptr || cand.id < host->id) { host = &cand; }
            }
            if(host == nullptr) { continue; }

            const double rOff = off.part->radiusOuterAt(zWorld - off.originZ);
            const double cap  = host->part->innerCapacityAt(zWorld - host->originZ);

            // A hollow host's bore capacity gates only an offender nested within its outer envelope. Once
            // the offender's outer radius reaches the host skin (rOff >= hostOuter) it sits on or outside
            // that skin -- e.g. a fin set's body disc on a co-radial aft coupler -- so a bore "intrusion"
            // there is a modelling artifact, not a collision. A solid host has cap == hostOuter, so this
            // guard never trips and poke-through still flags.
            const double hostOuter = host->part->radiusOuterAt(zWorld - host->originZ);
            const bool   boreHost  = cap < hostOuter - tol;        // hollow: capacity is the bore, not the skin
            if(boreHost && rOff >= hostOuter - tol) { continue; }  // offender lies on/outside the host skin

            if(rOff > cap + tol)
            {
                const double pen = rOff - cap;
                const std::pair<PartId, PartId> key{off.id, host->id};
                const auto                      existing = worst.find(key);
                if(existing == worst.end() || pen > existing->second.penetration)
                {
                    char msg[192];
                    std::snprintf(msg, sizeof(msg),
                                       "part %llu (OD %.5g m) intrudes into part %llu (capacity %.5g m) by "
                                       "%.5g m at z=%.5g m",
                                       static_cast<unsigned long long>(off.id), rOff,
                                       static_cast<unsigned long long>(host->id), cap, pen, zWorld);
                    OverlapDiagnostic d;
                    d.offender    = off.id;
                    d.host        = host->id;
                    d.zWorld      = zWorld;
                    d.penetration = pen;
                    d.message     = msg;
                    worst[key]    = d;
                }
            }
        }
    }

    for(const auto& [key, diag] : worst)
    {
        result.diagnostics.push_back(diag);
    }
    result.ok = result.diagnostics.empty();
    return result;
}

}  // namespace model::part
