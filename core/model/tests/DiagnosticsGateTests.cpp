// The diagnostics gate wired into PartNode's composite readers. The envelope sweep runs once per
// structural resolve (cached on the node alongside the resolved placements); a failed solve makes
// the composite CM/inertia readers a hard, located failure instead of a silently-wrong number,
// while a burning motor re-weights the cached geometry every step without re-running the sweep.

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <utility>

#include "model/PartsModel.h"
#include "model/parts/BodyTube.h"
#include "model/parts/ConicalNoseCone.h"
#include "model/parts/Placement.h"

namespace
{
using model::PartNode;
using model::PartsModel;
using model::part::BodyTube;
using model::part::ConicalNoseCone;
using model::part::SeatKind;
using model::part::StationLink;

// An over-nested coupler: inserted only 0.04 m into the body bore, its fore plane projects past the
// body rim into the solid nose, whose skin (0.0342 m at z=-0.26) is thinner than the coupler OD
// (0.0376 m) -- a poke-through. sweepOverlaps must flag it.
std::unique_ptr<PartNode> buildOverNested()
{
    auto nose = PartNode::make(
        std::make_unique<ConicalNoseCone>("Nose", 0.0395, 0.30, 0.0, 1700.0, true));
    auto body = PartNode::make(std::make_unique<BodyTube>("Body", 0.0376, 0.0395, 0.90, 1700.0));
    body->addChild(PartNode::make(std::make_unique<BodyTube>("Coupler", 0.036, 0.0376, 0.08, 1700.0)),
                   StationLink{.parentStation01 = 1.0, .childStation01 = 0.0,
                               .gap = 0.04, .seat = SeatKind::NestInBore});
    nose->addChild(std::move(body), StationLink{}); // abut
    return nose;
}

// A clean nose -> body abut of equal rim radii: no overlap anywhere.
std::unique_ptr<PartNode> buildClean()
{
    auto nose = PartNode::make(
        std::make_unique<ConicalNoseCone>("Nose", 0.0395, 0.30, 0.0, 1700.0, true));
    nose->addChild(PartNode::make(std::make_unique<BodyTube>("Body", 0.0376, 0.0395, 0.90, 1700.0)),
                   StationLink{}); // abut, rims 0.0395 == 0.0395
    return nose;
}

// A body tube whose mass ramps with time, so the composite mass-delta gate fires on every step (a
// stand-in for a burning motor), forcing computeCompositeAt to re-run -- the condition under which
// the sweep must not re-fire.
struct RampMassTube : BodyTube
{
    using BodyTube::BodyTube;
    double getMass(double t) const override { return 0.5 + t; } // strictly increasing, always positive
};

// A body tube that counts every radiusOuterAt() query. That virtual is touched only by the resolver
// (via stationAt) and the sweep -- both inside the node's structural resolve -- never by the
// per-step composite re-weight, so its call count is a faithful witness to "did the geometry/sweep
// cache rebuild?".
struct CountingTube : BodyTube
{
    using BodyTube::BodyTube;
    mutable int radiusCalls{0};
    double radiusOuterAt(double zLocal) const override { ++radiusCalls; return BodyTube::radiusOuterAt(zLocal); }
};
} // namespace

TEST(DiagnosticsGateTests, OverNestedCouplerStopsSolve)
{
    PartsModel pm;
    pm.installRoot(buildOverNested());
    const PartNode* root = pm.root();
    ASSERT_NE(root, nullptr);

    // The cached verdict is a located failure ...
    const model::part::SolveResult& diag = root->placementDiagnostics();
    ASSERT_FALSE(diag.ok);
    ASSERT_EQ(diag.diagnostics.size(), 1u);
    EXPECT_EQ(diag.diagnostics.front().host, root->id()); // pokes through the nose (the root)

    // ... and the geometry-dependent composite readers (which run computeCompositeAt) refuse it
    // rather than return a silently-wrong number.
    EXPECT_THROW((void)root->compositeI(0.0), std::runtime_error);
    EXPECT_THROW((void)root->compositeCm(0.0), std::runtime_error);

    // compositeMass is deliberately the cheap live getMass(t) sum (the hot ODE divisor): it does no
    // placement work, so it is not gated -- the gate guards the inertia/CM path, where geometry matters.
    EXPECT_NO_THROW((void)root->compositeMass(0.0));
}

TEST(DiagnosticsGateTests, CleanDesignSolves)
{
    PartsModel pm;
    pm.installRoot(buildClean());
    const PartNode* root = pm.root();
    ASSERT_NE(root, nullptr);

    EXPECT_TRUE(root->placementDiagnostics().ok);
    EXPECT_TRUE(root->placementDiagnostics().diagnostics.empty());

    EXPECT_NO_THROW((void)root->compositeI(0.0));
    EXPECT_GT(root->compositeMass(0.0), 0.0);
}

TEST(DiagnosticsGateTests, GateDoesNotReFirePerStepDuringBurn)
{
    // root counts radiusOuterAt queries; a ramp-mass child forces a per-step composite recompute.
    auto rootPart = std::make_unique<CountingTube>("Body", 0.0, 0.02, 0.30, 680.0);
    const CountingTube* counter = rootPart.get();
    auto rootNode = PartNode::make(std::move(rootPart));
    rootNode->addChild(
        PartNode::make(std::make_unique<RampMassTube>("Tip", 0.0, 0.02, 0.05, 680.0)),
        model::part::abut()); // OD 0.02 == root OD: clean abut

    PartsModel pm;
    pm.installRoot(std::move(rootNode));
    const PartNode* root = pm.root();
    ASSERT_NE(root, nullptr);

    // First inertia read resolves the geometry and runs the sweep once (compositeI goes through
    // computeCompositeAt -> the structural resolve).
    (void)root->compositeI(0.0);
    const int afterResolve = counter->radiusCalls;
    ASSERT_GT(afterResolve, 0) << "the first composite-inertia read must resolve + sweep";
    ASSERT_TRUE(root->placementDiagnostics().ok);

    // Advance through a "burn": the ramp-mass child changes the composite mass each step, so the
    // mass-delta gate fires and computeCompositeAt genuinely re-runs (the CM shifts toward the
    // growing child). Yet the geometry/sweep cache is gated by the structural flag, so radiusOuterAt
    // is never queried again -- the sweep does not re-fire.
    double prevMass = root->compositeMass(0.0);
    double prevCmZ  = root->compositeCm(0.0).z();
    for(double t : {0.25, 0.5, 1.0, 2.0})
    {
        (void)root->compositeI(t); // forces computeCompositeAt at this t
        const double m   = root->compositeMass(t);
        const double cmZ = root->compositeCm(t).z();
        EXPECT_GT(m, prevMass) << "ramp-mass child should raise the composite mass each step";
        EXPECT_NE(cmZ, prevCmZ) << "computeCompositeAt should re-run and shift the CM each step";
        prevMass = m;
        prevCmZ  = cmZ;
    }
    EXPECT_EQ(counter->radiusCalls, afterResolve) << "the sweep/resolve re-fired during the burn";
}
