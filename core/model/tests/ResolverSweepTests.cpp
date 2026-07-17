// Placement resolver + overlap diagnostics, exercised through the PartNode placement cache
// (resolvedPlacements / placementDiagnostics) and the free radialSeamCheck, against the whitepaper
// xl75_multi stack. Worked numbers (whitepaper Section 6, 10): nose span [-0.30, 0], body
// [-1.20, -0.30], coupler origin -0.26 spanning [-0.34, -0.26]; the coupler's fore plane projects
// 0.04 m past the body rim and the solid nose skin (0.0342 m at z=-0.26) is thinner than the coupler
// OD (0.0376 m), so the sweep flags a coupler-vs-nose poke-through of ~0.0034 m at z=-0.26.

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <span>
#include <string>
#include <utility>

#include "model/PartsModel.h"
#include "model/parts/BodyTube.h"
#include "model/parts/ConicalNoseCone.h"
#include "model/parts/FinSet.h"
#include "model/parts/Part.h"
#include "model/parts/Placement.h"

namespace
{
using model::PartNode;
using model::part::BodyTube;
using model::part::ConicalNoseCone;
using model::part::FinSet;
using model::part::Part;
using model::part::PartId;
using model::part::Placed;
using model::part::radialSeamCheck;
using model::part::SeatKind;
using model::part::SolveResult;
using model::part::StationLink;

struct Stack
{
    std::unique_ptr<PartNode>     root;  // the nose node, owning the whole tree
    PartId                        nose{0};
    PartId                        body{0};
    PartId                        fins{0};
    PartId                        coupler{0};
    // leaf parts, borrowed from the nodes -- for the seam-check tests
    const Part*                   nosePart{nullptr};
    const Part*                   bodyPart{nullptr};
    const Part*                   finsPart{nullptr};
    const Part*                   couplerPart{nullptr};
    std::map<PartId, StationLink> links;
};

// Build the whitepaper xl75_multi stack as a detached PartNode tree, plus each child's StationLink
// keyed by id for the seam checks. body's children are added in the fixture order (fins, then
// coupler), so DFS pre-order is nose, body, fins, coupler.
Stack buildXl75()
{
    auto nose    = std::make_unique<ConicalNoseCone>("MultiNose", 0.0395, 0.30, 0.0, 1700.0, true);
    auto body    = std::make_unique<BodyTube>("MultiBody", 0.0376, 0.0395, 0.90, 1700.0);
    auto fins    = std::make_unique<FinSet>("MultiFins", 6, 0.10, 0.04, 0.06, 0.04, 0.003, 0.0395, 1700.0);
    auto coupler = std::make_unique<BodyTube>("MultiCoupler", 0.036, 0.0376, 0.08, 1700.0);

    Stack s;
    s.nosePart    = nose.get();
    s.bodyPart    = body.get();
    s.finsPart    = fins.get();
    s.couplerPart = coupler.get();
    s.nose        = nose->getId();
    s.body        = body->getId();
    s.fins        = fins->getId();
    s.coupler     = coupler->getId();

    s.links[s.body] = StationLink{};  // abut default: body fore plane -> nose aft plane
    s.links[s.fins] =
        StationLink{.parentStation01 = 0.06, .childStation01 = 0.0, .seat = SeatKind::OnSurface};
    s.links[s.coupler] = StationLink{
        .parentStation01 = 1.0, .childStation01 = 0.0, .gap = 0.04, .seat = SeatKind::NestInBore};

    auto bodyNode = PartNode::make(std::move(body));
    bodyNode->addChild(PartNode::make(std::move(fins)), s.links.at(s.fins));
    bodyNode->addChild(PartNode::make(std::move(coupler)), s.links.at(s.coupler));
    auto noseNode = PartNode::make(std::move(nose));
    noseNode->addChild(std::move(bodyNode), s.links.at(s.body));
    s.root = std::move(noseNode);
    return s;
}

const Placed& placedOf(std::span<const Placed> placed, PartId id)
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
    const Stack                   s      = buildXl75();
    const std::span<const Placed> placed = s.root->resolvedPlacements();
    const model::part::Pose&      nose   = placedOf(placed, s.nose).pose;
    EXPECT_DOUBLE_EQ(nose.origin.x(), 0.0);
    EXPECT_DOUBLE_EQ(nose.origin.y(), 0.0);
    EXPECT_DOUBLE_EQ(nose.origin.z(), 0.0);  // nose tip at the world origin; rocket extends into -z
}

TEST(ResolverTests, AbutDefaultStacksAft)
{
    const Stack                   s      = buildXl75();
    const std::span<const Placed> placed = s.root->resolvedPlacements();
    const Placed&                 body   = placedOf(placed, s.body);
    EXPECT_DOUBLE_EQ(body.pose.origin.z(), -0.30);  // body fore plane abuts nose aft plane
    // span [-1.20, -0.30]
    EXPECT_DOUBLE_EQ(body.pose.origin.z() - body.part->axialLength(), -1.20);
}

TEST(ResolverTests, NestInBoreSignsGapAft)
{
    const Stack                   s       = buildXl75();
    const std::span<const Placed> placed  = s.root->resolvedPlacements();
    const Placed&                 coupler = placedOf(placed, s.coupler);
    EXPECT_DOUBLE_EQ(coupler.pose.origin.z(), -0.26);  // nested 0.04 aft of the body fore rim
    // span [-0.34, -0.26]
    EXPECT_DOUBLE_EQ(coupler.pose.origin.z() - coupler.part->axialLength(), -0.34);
}

TEST(ResolverTests, OnSurfaceFinStation)
{
    const Stack                   s      = buildXl75();
    const std::span<const Placed> placed = s.root->resolvedPlacements();
    const Placed&                 fins   = placedOf(placed, s.fins);
    // fore-plane origin -1.046; the seat (aft plane, childStation 0) lands at -1.146 = body's 0.06
    // station in world (-0.30 + (0.06-1)*0.90).
    EXPECT_NEAR(fins.pose.origin.z(), -1.046, 1e-12);
    EXPECT_NEAR(fins.pose.origin.z() - fins.part->axialLength(), -1.146, 1e-12);
}

TEST(ResolverTests, ComposeIsTranslationInThreeDof)
{
    const Stack s = buildXl75();
    for(const Placed& p : s.root->resolvedPlacements())
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
    // Two independently-built identical trees resolve in the same DFS pre-order (attachment order),
    // so a reload reproduces the placement list exactly.
    const Stack                   a  = buildXl75();
    const Stack                   b  = buildXl75();
    const std::span<const Placed> pa = a.root->resolvedPlacements();
    const std::span<const Placed> pb = b.root->resolvedPlacements();
    ASSERT_EQ(pa.size(), 4u);
    ASSERT_EQ(pb.size(), 4u);
    EXPECT_EQ(pa[0].part->getId(), a.nose);  // DFS pre-order: nose, body, fins, coupler
    EXPECT_EQ(pa[1].part->getId(), a.body);
    EXPECT_EQ(pa[2].part->getId(), a.fins);
    EXPECT_EQ(pa[3].part->getId(), a.coupler);
    EXPECT_EQ(pb[0].part->getId(), b.nose);
    EXPECT_EQ(pb[1].part->getId(), b.body);
    EXPECT_EQ(pb[2].part->getId(), b.fins);
    EXPECT_EQ(pb[3].part->getId(), b.coupler);
    // parent before child: each parentId names an earlier entry (0 on the root).
    EXPECT_EQ(pa[0].parentId, 0u);
    EXPECT_EQ(pa[1].parentId, a.nose);
    EXPECT_EQ(pa[2].parentId, a.body);
    EXPECT_EQ(pa[3].parentId, a.body);
}

TEST(ResolverTests, LengthTracksFractionalStation)
{
    // A longer body moves the OnSurface fin seat aft: the fractional station resolves against live
    // length, so nothing is pinned to a stale coordinate.
    const auto finOriginForBodyLen = [](double bodyLen)
    {
        auto         nose  = std::make_unique<ConicalNoseCone>("N", 0.0395, 0.30, 0.0, 1700.0, true);
        auto         body  = std::make_unique<BodyTube>("B", 0.0376, 0.0395, bodyLen, 1700.0);
        auto         fins  = std::make_unique<FinSet>("F", 6, 0.10, 0.04, 0.06, 0.04, 0.003, 0.0395, 1700.0);
        const PartId finId = fins->getId();
        auto bodyNode = PartNode::make(std::move(body));
        bodyNode->addChild(
            PartNode::make(std::move(fins)),
            StationLink{.parentStation01 = 0.06, .childStation01 = 0.0, .seat = SeatKind::OnSurface});
        auto root = PartNode::make(std::move(nose));
        root->addChild(std::move(bodyNode), StationLink{});
        return placedOf(root->resolvedPlacements(), finId).pose.origin.z();
    };
    EXPECT_LT(finOriginForBodyLen(1.20), finOriginForBodyLen(0.90));  // longer body -> fin further aft
}

// ---- Layer 1: radial seam check (T4) ------------------------------------------------------------

TEST(SweepTests, Layer1AbutSeamClean)
{
    const Stack s = buildXl75();
    // nose base rim 0.0395 == body fore rim 0.0395.
    EXPECT_FALSE(radialSeamCheck(*s.nosePart, *s.bodyPart, s.links.at(s.body)).has_value());
}

TEST(SweepTests, Layer1NestInBoreFits)
{
    const Stack s = buildXl75();
    // coupler OD 0.0376 <= body ID 0.0376 (boundary equality fits within tol).
    EXPECT_FALSE(radialSeamCheck(*s.bodyPart, *s.couplerPart, s.links.at(s.coupler)).has_value());
}

TEST(SweepTests, Layer1OnSurfaceFinMatch)
{
    const Stack s = buildXl75();
    // fin bodyRadius 0.0395 == body OD 0.0395.
    EXPECT_FALSE(radialSeamCheck(*s.bodyPart, *s.finsPart, s.links.at(s.fins)).has_value());
}

TEST(SweepTests, Layer1NestInBoreOverWideFlags)
{
    const BodyTube    body{"B", 0.0376, 0.0395, 0.90, 1700.0};  // bore 0.0376
    const BodyTube    wide{"W", 0.030, 0.039, 0.08, 1700.0};    // OD 0.039
    const StationLink link{
        .parentStation01 = 1.0, .childStation01 = 0.0, .gap = 0.04, .seat = SeatKind::NestInBore};
    const auto d = radialSeamCheck(body, wide, link);
    ASSERT_TRUE(d.has_value());
    EXPECT_EQ(d->offender, wide.getId());
    EXPECT_EQ(d->host, body.getId());
    EXPECT_NEAR(d->penetration, 0.039 - 0.0376, 1e-12);
}

TEST(SweepTests, Layer1AbutMismatchFlags)
{
    // Two parts abutted rim-to-rim whose outer radii differ by more than tol: the Abut seam check must
    // flag exactly one diagnostic (offender = child, host = parent) with the rim-radius gap as the
    // penetration. (The over-wide test above is NestInBore; this exercises the distinct Abut branch.)
    const BodyTube    host{"Host", 0.030, 0.040, 0.20, 1700.0};   // OD 0.040
    const BodyTube    child{"Child", 0.020, 0.030, 0.10, 1700.0}; // OD 0.030
    const StationLink link = model::part::abut();  // default abut: child fore -> parent aft, equal radii expected
    const auto        d    = radialSeamCheck(host, child, link);
    ASSERT_TRUE(d.has_value());
    EXPECT_EQ(d->offender, child.getId());
    EXPECT_EQ(d->host, host.getId());
    EXPECT_NEAR(d->penetration, 0.040 - 0.030, 1e-12);  // |parent rim - child rim|
}

// ---- Layer 2: envelope sweep (T4) ---------------------------------------------------------------

TEST(SweepTests, CouplerPokesThroughNose)
{
    const Stack       s = buildXl75();
    const SolveResult& r = s.root->placementDiagnostics();

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
    // A poke-through diagnostic carries a located, human-readable message, not a bare flag: it names
    // the offender and host by id, the offending OD, the host capacity, the penetration, and the world
    // z. The sweep reasons over intervals and capacities, not seat relationships, so the message takes
    // this located OD/capacity/z form rather than a seat-narrative sentence.
    const Stack        s = buildXl75();
    const SolveResult& r = s.root->placementDiagnostics();

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
    const Stack        s = buildXl75();
    const SolveResult& r = s.root->placementDiagnostics();
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
    auto body    = std::make_unique<BodyTube>("B", 0.0121, 0.013, 0.30, 1000.0);   // OD 0.013, bore 0.0121
    auto fins    = std::make_unique<FinSet>("F", 6, 0.04, 0.02, 0.025, 0.015, 0.0025, 0.013, 900.0);
    auto coupler = std::make_unique<BodyTube>("C", 0.0121, 0.013, 0.04, 1000.0);   // co-radial aft tube

    // Fin root near the aft end so it overhangs the body aft plane; coupler abutted aft of the body.
    auto root = PartNode::make(std::move(body));
    root->addChild(
        PartNode::make(std::move(fins)),
        StationLink{.parentStation01 = 0.02, .childStation01 = 0.0, .seat = SeatKind::OnSurface});
    root->addChild(PartNode::make(std::move(coupler)),
                   StationLink{.parentStation01 = 0.0, .childStation01 = 1.0,
                               .gap = 0.0, .seat = SeatKind::Abut});

    const SolveResult& r = root->placementDiagnostics();
    EXPECT_TRUE(r.ok) << "fin disc on a co-radial aft coupler must not false-collide";
    EXPECT_TRUE(r.diagnostics.empty());
}

TEST(SweepTests, TieBreakSmallestId)
{
    // When an offender's sample station is covered by MORE THAN ONE valid (non-excluded) host, the
    // sweep must select the smallest-id covering host, so the diagnostic is deterministic and
    // reproducible -- independent of child-insertion order and the (unstable) interval sort. No real
    // fixture produces a multi-cover host, so this is a crafted geometry: three co-located coaxial
    // solid rods abutted to a trunk's aft plane -- two thin hosts of EQUAL outer radius (so they never
    // flag each other: rOff == cap) and one fat offender that pokes through BOTH at once.
    struct Built
    {
        std::unique_ptr<PartNode> root;
        PartId                    lo{0}, hi{0}, off{0};
    };
    const auto build = []
    {
        auto base     = std::make_unique<BodyTube>("Base", 0.0, 0.02, 0.05, 1000.0);     // solid trunk
        // Construct lo BEFORE hi so lo carries the SMALLER id; the fat offender pokes both equally.
        auto lo       = std::make_unique<BodyTube>("HostLo", 0.0, 0.01, 0.10, 1000.0);    // solid, OD 0.01
        auto hi       = std::make_unique<BodyTube>("HostHi", 0.0, 0.01, 0.10, 1000.0);    // solid, OD 0.01
        auto offender = std::make_unique<BodyTube>("Offender", 0.0, 0.03, 0.10, 1000.0);  // solid, OD 0.03
        Built b{nullptr, lo->getId(), hi->getId(), offender->getId()};

        // Co-locate all three on the trunk's aft plane (identical abut link => identical resolved
        // span). Add the LARGER-id host FIRST, so a correct result proves the choice tracks id, not
        // insertion order.
        const StationLink coincide = model::part::abut();
        b.root = PartNode::make(std::move(base));
        b.root->addChild(PartNode::make(std::move(hi)), coincide);
        b.root->addChild(PartNode::make(std::move(lo)), coincide);
        b.root->addChild(PartNode::make(std::move(offender)), coincide);
        return b;
    };

    const Built a = build();
    ASSERT_LT(a.lo, a.hi) << "construction order should make lo the smaller id";
    const SolveResult& r = a.root->placementDiagnostics();
    ASSERT_FALSE(r.ok);
    ASSERT_EQ(r.diagnostics.size(), 1u);  // only the fat offender pokes; the equal-radius hosts do not
    const model::part::OverlapDiagnostic& d = r.diagnostics.front();
    EXPECT_EQ(d.offender, a.off);
    EXPECT_EQ(d.host, a.lo) << "multi-cover host must be the smallest id (" << a.lo << "), not " << a.hi;
    EXPECT_NEAR(d.penetration, 0.03 - 0.01, 1e-12);

    // Reproducible: resolving an identically-built tree (what a reload does) yields the same host,
    // with no dependence on the unstable interval sort.
    const Built b2 = build();
    ASSERT_LT(b2.lo, b2.hi);
    const SolveResult& r2 = b2.root->placementDiagnostics();
    ASSERT_EQ(r2.diagnostics.size(), 1u);
    EXPECT_EQ(r2.diagnostics.front().host, b2.lo);
}

TEST(SweepTests, FullyNestedCouplerIsClean)
{
    // Insert the coupler its full length (depth 0.08) so it stays entirely inside the body bore and
    // never projects past the body fore rim into the nose: no poke-through.
    auto nose    = std::make_unique<ConicalNoseCone>("N", 0.0395, 0.30, 0.0, 1700.0, true);
    auto body    = std::make_unique<BodyTube>("B", 0.0376, 0.0395, 0.90, 1700.0);
    auto coupler = std::make_unique<BodyTube>("C", 0.036, 0.0376, 0.08, 1700.0);
    auto bodyNode = PartNode::make(std::move(body));
    bodyNode->addChild(PartNode::make(std::move(coupler)),
                       StationLink{.parentStation01 = 1.0, .childStation01 = 0.0,
                                   .gap = 0.08, .seat = SeatKind::NestInBore});
    auto root = PartNode::make(std::move(nose));
    root->addChild(std::move(bodyNode), StationLink{});

    const SolveResult& r = root->placementDiagnostics();
    EXPECT_TRUE(r.ok);
    EXPECT_TRUE(r.diagnostics.empty());
}
