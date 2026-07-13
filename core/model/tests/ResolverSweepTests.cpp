// T2 (resolver) + T4 (overlap diagnostics) -- implementation plan Part II Step 5. Exercises the free
// functions resolvePlacements / radialSeamCheck / sweepOverlaps against the whitepaper xl75_multi
// stack, built programmatically with StationLinks (the storage swap that puts links in childParts is
// Step 6, so the resolver is fed an explicit links map here -- the Step-5 scaffold).
//
// The worked numbers (whitepaper Section 6, 10): nose span [-0.30, 0], body [-1.20, -0.30], coupler
// origin -0.26 spanning [-0.34, -0.26]; the coupler's fore plane projects 0.04 m past the body rim and
// the solid nose skin (0.0342 m at z=-0.26) is thinner than the coupler OD (0.0376 m), so the sweep
// flags a coupler-vs-nose poke-through of ~0.0034 m at z=-0.26.

#include <gtest/gtest.h>

#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "model/parts/BodyTube.h"
#include "model/parts/ConicalNoseCone.h"
#include "model/parts/FinSet.h"
#include "model/parts/Part.h"
#include "model/parts/Placement.h"
#include "utils/math/MathTypes.h"

namespace
{
using model::part::BodyTube;
using model::part::ConicalNoseCone;
using model::part::FinSet;
using model::part::Part;
using model::part::PartId;
using model::part::Placed;
using model::part::Pose;
using model::part::radialSeamCheck;
using model::part::resolvePlacements;
using model::part::SeatKind;
using model::part::SolveResult;
using model::part::StationLink;
using model::part::sweepOverlaps;

struct Stack
{
    std::shared_ptr<Part>         root;  // the nose, owning the whole tree
    PartId                        nose{0};
    PartId                        body{0};
    PartId                        fins{0};
    PartId                        coupler{0};
    std::map<PartId, StationLink> links;
};

// Build the whitepaper xl75_multi stack as a REAL Part tree (placeholder offsets, ignored by the
// StationLink resolver) plus the StationLink map keyed by child id. body's children are added in the
// fixture order (fins, then coupler), so DFS pre-order is nose, body, fins, coupler.
Stack buildXl75()
{
    auto nose    = std::make_shared<ConicalNoseCone>("MultiNose", 0.0395, 0.30, 0.0, 1700.0, true);
    auto body    = std::make_shared<BodyTube>("MultiBody", 0.0376, 0.0395, 0.90, 1700.0);
    auto fins    = std::make_shared<FinSet>("MultiFins", 6, 0.10, 0.04, 0.06, 0.04, 0.003, 0.0395, 1700.0);
    auto coupler = std::make_shared<BodyTube>("MultiCoupler", 0.036, 0.0376, 0.08, 1700.0);

    Stack s;
    s.nose    = nose->getId();
    s.body    = body->getId();
    s.fins    = fins->getId();
    s.coupler = coupler->getId();

    s.links[s.body] = StationLink{};  // abut default: body fore plane -> nose aft plane
    s.links[s.fins] =
        StationLink{.parentStation01 = 0.06, .childStation01 = 0.0, .seat = SeatKind::OnSurface};
    s.links[s.coupler] = StationLink{
        .parentStation01 = 1.0, .childStation01 = 0.0, .gap = 0.04, .seat = SeatKind::NestInBore};

    body->addChildPart(fins, s.links[s.fins]);     // links now live in childParts (Step 6)
    body->addChildPart(coupler, s.links[s.coupler]);
    nose->addChildPart(body, s.links[s.body]);
    s.root = nose;
    return s;
}

const Placed& placedOf(const std::vector<Placed>& placed, PartId id)
{
    for(const Placed& p : placed)
    {
        if(p.part->getId() == id) { return p; }
    }
    ADD_FAILURE() << "part id " << id << " not present in the resolved placements";
    static const Placed sentinel{};
    return sentinel;
}
}  // namespace

// ---- Resolver (T2) ------------------------------------------------------------------------------

TEST(ResolverTests, RootPlantedAtRootPose)
{
    const Stack                 s      = buildXl75();
    const std::vector<Placed>   placed = resolvePlacements(*s.root, Pose{});
    const Pose&                 nose   = placedOf(placed, s.nose).pose;
    EXPECT_DOUBLE_EQ(nose.origin.x(), 0.0);
    EXPECT_DOUBLE_EQ(nose.origin.y(), 0.0);
    EXPECT_DOUBLE_EQ(nose.origin.z(), 0.0);  // nose tip at the world origin; rocket extends into -z
}

TEST(ResolverTests, AbutDefaultStacksAft)
{
    const Stack               s      = buildXl75();
    const std::vector<Placed> placed = resolvePlacements(*s.root, Pose{});
    const Placed&             body   = placedOf(placed, s.body);
    EXPECT_DOUBLE_EQ(body.pose.origin.z(), -0.30);  // body fore plane abuts nose aft plane
    // span [-1.20, -0.30]
    EXPECT_DOUBLE_EQ(body.pose.origin.z() - body.part->axialLength(), -1.20);
}

TEST(ResolverTests, NestInBoreSignsGapAft)
{
    const Stack               s       = buildXl75();
    const std::vector<Placed> placed  = resolvePlacements(*s.root, Pose{});
    const Placed&             coupler = placedOf(placed, s.coupler);
    EXPECT_DOUBLE_EQ(coupler.pose.origin.z(), -0.26);  // nested 0.04 aft of the body fore rim
    // span [-0.34, -0.26]
    EXPECT_DOUBLE_EQ(coupler.pose.origin.z() - coupler.part->axialLength(), -0.34);
}

TEST(ResolverTests, OnSurfaceFinStation)
{
    const Stack               s      = buildXl75();
    const std::vector<Placed> placed = resolvePlacements(*s.root, Pose{});
    const Placed&             fins   = placedOf(placed, s.fins);
    // fore-plane origin -1.046; the seat (aft plane, childStation 0) lands at -1.146 = body's 0.06
    // station in world (-0.30 + (0.06-1)*0.90).
    EXPECT_NEAR(fins.pose.origin.z(), -1.046, 1e-12);
    EXPECT_NEAR(fins.pose.origin.z() - fins.part->axialLength(), -1.146, 1e-12);
}

TEST(ResolverTests, ComposeIsTranslationInThreeDof)
{
    const Stack               s      = buildXl75();
    const std::vector<Placed> placed = resolvePlacements(*s.root, Pose{});
    for(const Placed& p : placed)
    {
        EXPECT_DOUBLE_EQ(p.pose.orient.x(), 0.0);
        EXPECT_DOUBLE_EQ(p.pose.orient.y(), 0.0);
        EXPECT_DOUBLE_EQ(p.pose.orient.z(), 0.0);
        EXPECT_DOUBLE_EQ(p.pose.orient.w(), 1.0);  // identity orientation throughout
        EXPECT_DOUBLE_EQ(p.pose.origin.x(), 0.0);  // coaxial
        EXPECT_DOUBLE_EQ(p.pose.origin.y(), 0.0);
    }
}

TEST(ResolverTests, DeterministicDfsOrder)
{
    const Stack               s = buildXl75();
    const std::vector<Placed> a = resolvePlacements(*s.root, Pose{});
    const std::vector<Placed> b = resolvePlacements(*s.root, Pose{});
    ASSERT_EQ(a.size(), 4u);
    ASSERT_EQ(b.size(), 4u);
    for(std::size_t i = 0; i < a.size(); ++i)
    {
        EXPECT_EQ(a[i].part->getId(), b[i].part->getId());  // stable across calls
    }
    EXPECT_EQ(a[0].part->getId(), s.nose);  // DFS pre-order: nose, body, fins, coupler
    EXPECT_EQ(a[1].part->getId(), s.body);
    EXPECT_EQ(a[2].part->getId(), s.fins);
    EXPECT_EQ(a[3].part->getId(), s.coupler);
}

TEST(ResolverTests, LengthTracksFractionalStation)
{
    // A longer body moves the OnSurface fin seat aft: the fractional station resolves against live
    // length, so nothing is pinned to a stale coordinate.
    const auto finOriginForBodyLen = [](double bodyLen)
    {
        auto         nose  = std::make_shared<ConicalNoseCone>("N", 0.0395, 0.30, 0.0, 1700.0, true);
        auto         body  = std::make_shared<BodyTube>("B", 0.0376, 0.0395, bodyLen, 1700.0);
        auto         fins  = std::make_shared<FinSet>("F", 6, 0.10, 0.04, 0.06, 0.04, 0.003, 0.0395, 1700.0);
        const PartId finId = fins->getId();
        body->addChildPart(
            fins, StationLink{.parentStation01 = 0.06, .childStation01 = 0.0, .seat = SeatKind::OnSurface});
        nose->addChildPart(body, StationLink{});
        return placedOf(resolvePlacements(*nose, Pose{}), finId).pose.origin.z();
    };
    EXPECT_LT(finOriginForBodyLen(1.20), finOriginForBodyLen(0.90));  // longer body -> fin further aft
}

// ---- Layer 1: radial seam check (T4) ------------------------------------------------------------

TEST(SweepTests, Layer1AbutSeamClean)
{
    const Stack s    = buildXl75();
    const Part* body = s.root->findById(s.body);
    ASSERT_NE(body, nullptr);
    // nose base rim 0.0395 == body fore rim 0.0395.
    EXPECT_FALSE(radialSeamCheck(*s.root, *body, s.links.at(s.body)).has_value());
}

TEST(SweepTests, Layer1NestInBoreFits)
{
    const Stack s       = buildXl75();
    const Part* body    = s.root->findById(s.body);
    const Part* coupler = s.root->findById(s.coupler);
    ASSERT_NE(body, nullptr);
    ASSERT_NE(coupler, nullptr);
    // coupler OD 0.0376 <= body ID 0.0376 (boundary equality fits within tol).
    EXPECT_FALSE(radialSeamCheck(*body, *coupler, s.links.at(s.coupler)).has_value());
}

TEST(SweepTests, Layer1OnSurfaceFinMatch)
{
    const Stack s    = buildXl75();
    const Part* body = s.root->findById(s.body);
    const Part* fins = s.root->findById(s.fins);
    ASSERT_NE(body, nullptr);
    ASSERT_NE(fins, nullptr);
    // fin bodyRadius 0.0395 == body OD 0.0395.
    EXPECT_FALSE(radialSeamCheck(*body, *fins, s.links.at(s.fins)).has_value());
}

TEST(SweepTests, Layer1NestInBoreOverWideFlags)
{
    auto              body = std::make_shared<BodyTube>("B", 0.0376, 0.0395, 0.90, 1700.0);  // bore 0.0376
    auto              wide = std::make_shared<BodyTube>("W", 0.030, 0.039, 0.08, 1700.0);     // OD 0.039
    const StationLink link{
        .parentStation01 = 1.0, .childStation01 = 0.0, .gap = 0.04, .seat = SeatKind::NestInBore};
    const auto d = radialSeamCheck(*body, *wide, link);
    ASSERT_TRUE(d.has_value());
    EXPECT_EQ(d->offender, wide->getId());
    EXPECT_EQ(d->host, body->getId());
    EXPECT_NEAR(d->penetration, 0.039 - 0.0376, 1e-12);
}

TEST(SweepTests, Layer1AbutMismatchFlags)
{
    // Two parts abutted rim-to-rim whose outer radii differ by more than tol: the Abut seam check must
    // flag exactly one diagnostic (offender = child, host = parent) with the rim-radius gap as the
    // penetration. (The over-wide test above is NestInBore; this exercises the distinct Abut branch.)
    auto              host  = std::make_shared<BodyTube>("Host", 0.030, 0.040, 0.20, 1700.0);  // OD 0.040
    auto              child = std::make_shared<BodyTube>("Child", 0.020, 0.030, 0.10, 1700.0); // OD 0.030
    const StationLink link  = model::part::abut();  // default abut: child fore -> parent aft, equal radii expected
    const auto        d     = radialSeamCheck(*host, *child, link);
    ASSERT_TRUE(d.has_value());
    EXPECT_EQ(d->offender, child->getId());
    EXPECT_EQ(d->host, host->getId());
    EXPECT_NEAR(d->penetration, 0.040 - 0.030, 1e-12);  // |parent rim - child rim|
}

// ---- Layer 2: envelope sweep (T4) ---------------------------------------------------------------

TEST(SweepTests, CouplerPokesThroughNose)
{
    const Stack               s      = buildXl75();
    const std::vector<Placed> placed = resolvePlacements(*s.root, Pose{});
    const SolveResult         r      = sweepOverlaps(placed);

    ASSERT_FALSE(r.ok);
    ASSERT_EQ(r.diagnostics.size(), 1u);
    const model::part::OverlapDiagnostic& d = r.diagnostics.front();
    EXPECT_EQ(d.offender, s.coupler);
    EXPECT_EQ(d.host, s.nose);  // the host is the nose -- a non-tree neighbour (coupler is linked to the body)
    EXPECT_NEAR(d.zWorld, -0.26, 1e-12);
    EXPECT_NEAR(d.penetration, 0.0376 - 0.0395 * (0.26 / 0.30), 1e-12);  // ~0.003367 m
}

TEST(SweepTests, Layer2DiagnosticIsLocated)
{
    // A poke-through diagnostic carries a LOCATED, human-readable message, not a bare flag: it names the
    // offender and host by id, the offending OD, the host capacity, the penetration, and the world z.
    // (The plan's worked-example narrative -- "0.04 m forward of the body rim", "skin" -- is not produced
    // by the generic envelope sweep, which reasons over intervals and capacities, not seat relationships;
    // the realized message is this located OD/capacity/z form. See the plan's Part III as-built note.)
    const Stack               s      = buildXl75();
    const std::vector<Placed> placed = resolvePlacements(*s.root, Pose{});
    const SolveResult         r      = sweepOverlaps(placed);

    ASSERT_FALSE(r.ok);
    ASSERT_EQ(r.diagnostics.size(), 1u);
    const model::part::OverlapDiagnostic& d = r.diagnostics.front();

    ASSERT_FALSE(d.message.empty());
    EXPECT_NE(d.message.find("intrudes"), std::string::npos) << d.message;
    EXPECT_NE(d.message.find("z="), std::string::npos) << d.message;
    EXPECT_NE(d.message.find(std::to_string(d.offender)), std::string::npos)
        << "message must name the offender id: " << d.message;
    EXPECT_NE(d.message.find(std::to_string(d.host)), std::string::npos)
        << "message must name the host id: " << d.message;
}

TEST(SweepTests, OnSurfaceFinNotFalseDisc)
{
    // The fin set contributes only its body disc and is seated on the body (its direct seat parent),
    // so it must NOT false-collide with the adjacent body tube (regression for the rejected
    // bodyRadius+span disc and for the seat-parent exclusion).
    const Stack               s      = buildXl75();
    const std::vector<Placed> placed = resolvePlacements(*s.root, Pose{});
    const SolveResult         r      = sweepOverlaps(placed);
    for(const model::part::OverlapDiagnostic& d : r.diagnostics)
    {
        EXPECT_NE(d.offender, s.fins) << "fin set falsely flagged as an offender";
        EXPECT_NE(d.host, s.fins) << "fin set falsely flagged as a host";
    }
}

TEST(SweepTests, OnSurfaceFinOverCoRadialAftCouplerIsClean)
{
    // Regression for the fin-vs-coupler FALSE POSITIVE (corpus fixtures mid24_multi / large38_multi):
    // a fin set mounted on the body's outer wall, whose root chord protrudes a hair past the body aft
    // plane, axially overlaps a sibling AFT coupler of the SAME outer radius (a tail tube continuing the
    // airframe OD). The fin's body disc (r = bodyRadius) sits ON the coupler's outer skin, not inside its
    // bore -- they are radially separated by the airframe wall -- so the sweep must NOT flag it.
    auto body    = std::make_shared<BodyTube>("B", 0.0121, 0.013, 0.30, 1000.0);   // OD 0.013, bore 0.0121
    auto fins    = std::make_shared<FinSet>("F", 6, 0.04, 0.02, 0.025, 0.015, 0.0025, 0.013, 900.0);
    auto coupler = std::make_shared<BodyTube>("C", 0.0121, 0.013, 0.04, 1000.0);    // co-radial aft tube

    // Fin root near the aft end so it overhangs the body aft plane; coupler abutted aft of the body.
    body->addChildPart(
        fins, StationLink{.parentStation01 = 0.02, .childStation01 = 0.0, .seat = SeatKind::OnSurface});
    body->addChildPart(coupler, StationLink{.parentStation01 = 0.0, .childStation01 = 1.0,
                                                         .gap = 0.0, .seat = SeatKind::Abut});

    const SolveResult r = sweepOverlaps(resolvePlacements(*body, Pose{}));
    EXPECT_TRUE(r.ok) << "fin disc on a co-radial aft coupler must not false-collide";
    EXPECT_TRUE(r.diagnostics.empty());
}

TEST(SweepTests, TieBreakSmallestId)
{
    // When an offender's sample station is covered by MORE THAN ONE valid (non-excluded) host, the sweep
    // must select the smallest-Id covering host (Placement.cpp), so the diagnostic is deterministic and
    // reproducible -- independent of child-insertion order and the (unstable) interval sort. No real
    // fixture produces a multi-cover host, so this is a crafted geometry: three co-located coaxial solid
    // rods abutted to a trunk's aft plane -- two thin hosts of EQUAL outer radius (so they never flag each
    // other: rOff == cap) and one fat offender that pokes through BOTH at once.
    auto base     = std::make_shared<BodyTube>("Base", 0.0, 0.02, 0.05, 1000.0);     // solid trunk
    // Construct lo BEFORE hi so lo carries the SMALLER id; the fat offender pokes both equally.
    auto lo       = std::make_shared<BodyTube>("HostLo", 0.0, 0.01, 0.10, 1000.0);    // solid, OD 0.01
    auto hi       = std::make_shared<BodyTube>("HostHi", 0.0, 0.01, 0.10, 1000.0);    // solid, OD 0.01
    auto offender = std::make_shared<BodyTube>("Offender", 0.0, 0.03, 0.10, 1000.0);  // solid, OD 0.03
    const PartId loId = lo->getId(), hiId = hi->getId(), offId = offender->getId();
    ASSERT_LT(loId, hiId) << "construction order should make lo the smaller id";

    // Co-locate all three on the trunk's aft plane (identical abut link => identical resolved span). Add
    // the LARGER-id host FIRST, so a correct result proves the choice tracks id, not insertion order.
    const StationLink coincide = model::part::abut();
    base->addChildPart(hi, coincide);
    base->addChildPart(lo, coincide);
    base->addChildPart(offender, coincide);

    const SolveResult r = sweepOverlaps(resolvePlacements(*base, Pose{}));
    ASSERT_FALSE(r.ok);
    ASSERT_EQ(r.diagnostics.size(), 1u);  // only the fat offender pokes; the equal-radius hosts do not
    const model::part::OverlapDiagnostic& d = r.diagnostics.front();
    EXPECT_EQ(d.offender, offId);
    EXPECT_EQ(d.host, loId) << "multi-cover host must be the smallest id (" << loId << "), not " << hiId;
    EXPECT_NEAR(d.penetration, 0.03 - 0.01, 1e-12);

    // Reproducible: re-resolving the SAME tree (what a reload does) yields the identical host, with no
    // dependence on the unstable interval sort.
    const SolveResult r2 = sweepOverlaps(resolvePlacements(*base, Pose{}));
    ASSERT_EQ(r2.diagnostics.size(), 1u);
    EXPECT_EQ(r2.diagnostics.front().host, loId);
}

TEST(SweepTests, FullyNestedCouplerIsClean)
{
    // Insert the coupler its full length (depth 0.08) so it stays entirely inside the body bore and
    // never projects past the body fore rim into the nose: no poke-through.
    auto         nose    = std::make_shared<ConicalNoseCone>("N", 0.0395, 0.30, 0.0, 1700.0, true);
    auto         body    = std::make_shared<BodyTube>("B", 0.0376, 0.0395, 0.90, 1700.0);
    auto         coupler = std::make_shared<BodyTube>("C", 0.036, 0.0376, 0.08, 1700.0);
    body->addChildPart(coupler, StationLink{
        .parentStation01 = 1.0, .childStation01 = 0.0, .gap = 0.08, .seat = SeatKind::NestInBore});
    nose->addChildPart(body, StationLink{});

    const SolveResult r = sweepOverlaps(resolvePlacements(*nose, Pose{}));
    EXPECT_TRUE(r.ok);
    EXPECT_TRUE(r.diagnostics.empty());
}
