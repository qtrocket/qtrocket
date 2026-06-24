// Step 11 -- the diagnostics gate wired into the composite consumer. The Layer-2 envelope sweep runs
// ONCE per structural resolve (cached in Part alongside the resolved placements); a failed solve makes
// the composite mass/inertia readers a hard, located failure instead of a silently-wrong number, while
// a burning motor re-weights the cached geometry every step WITHOUT re-running the sweep.

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

#include "model/parts/BodyTube.h"
#include "model/parts/ConicalNoseCone.h"
#include "model/parts/Part.h"
#include "model/parts/Placement.h"

namespace
{
using model::part::BodyTube;
using model::part::ConicalNoseCone;
using model::part::SeatKind;
using model::part::StationLink;

// An over-nested coupler: inserted only 0.04 m into the body bore, its fore plane projects past the
// body rim into the solid nose, whose skin (0.0342 m at z=-0.26) is thinner than the coupler OD
// (0.0376 m) -- the whitepaper xl75 poke-through. sweepOverlaps must flag it.
std::shared_ptr<ConicalNoseCone> buildOverNested()
{
   auto nose    = std::make_shared<ConicalNoseCone>("Nose", 0.0395, 0.30, 0.0, 1700.0, true);
   auto body    = std::make_shared<BodyTube>("Body", 0.0376, 0.0395, 0.90, 1700.0);
   auto coupler = std::make_shared<BodyTube>("Coupler", 0.036, 0.0376, 0.08, 1700.0);
   body->addChildPart(coupler, StationLink{.parentStation01 = 1.0, .childStation01 = 0.0,
                                           .gap = 0.04, .seat = SeatKind::NestInBore});
   nose->addChildPart(body, StationLink{}); // abut
   return nose;
}

// A clean nose -> body abut of equal rim radii: no overlap anywhere.
std::shared_ptr<ConicalNoseCone> buildClean()
{
   auto nose = std::make_shared<ConicalNoseCone>("Nose", 0.0395, 0.30, 0.0, 1700.0, true);
   auto body = std::make_shared<BodyTube>("Body", 0.0376, 0.0395, 0.90, 1700.0);
   nose->addChildPart(body, StationLink{}); // abut, rims 0.0395 == 0.0395
   return nose;
}

// A body tube whose mass ramps with time, so the composite mass-delta gate fires on every step (a
// stand-in for a burning motor), forcing computeCompositeAt to re-run -- the condition under which the
// sweep must NOT re-fire.
struct RampMassTube : BodyTube
{
   using BodyTube::BodyTube;
   double getMass(double t) const override { return 0.5 + t; } // strictly increasing, always positive
};

// A body tube that counts every radiusOuterAt() query. That virtual is touched only by the resolver
// (via stationAt) and the sweep -- both inside ensurePlacementCache -- never by the per-step composite
// re-weight, so its call count is a faithful witness to "did the geometry/sweep cache rebuild?".
struct CountingTube : BodyTube
{
   using BodyTube::BodyTube;
   mutable int radiusCalls{0};
   double radiusOuterAt(double zLocal) const override { ++radiusCalls; return BodyTube::radiusOuterAt(zLocal); }
};
} // namespace

TEST(DiagnosticsGateTests, OverNestedCouplerStopsSolve)
{
   auto root = buildOverNested();

   // The cached verdict is a located failure ...
   const model::part::SolveResult& diag = root->placementDiagnostics();
   ASSERT_FALSE(diag.ok);
   ASSERT_EQ(diag.diagnostics.size(), 1u);
   EXPECT_EQ(diag.diagnostics.front().host, root->getId()); // pokes through the nose (the root)

   // ... and the geometry-dependent composite readers (which run computeCompositeAt) refuse it rather
   // than return a silently-wrong number.
   EXPECT_THROW((void)root->getCompositeI(0.0), std::runtime_error);
   EXPECT_THROW((void)root->getCompositeCm(0.0), std::runtime_error);

   // getCompositeMass is deliberately the cheap LIVE getMass(t) sum (the hot ODE divisor): it does no
   // placement work, so it is NOT gated -- the gate guards the inertia/CM path, where geometry matters.
   EXPECT_NO_THROW((void)root->getCompositeMass(0.0));
}

TEST(DiagnosticsGateTests, CleanDesignSolves)
{
   auto root = buildClean();

   EXPECT_TRUE(root->placementDiagnostics().ok);
   EXPECT_TRUE(root->placementDiagnostics().diagnostics.empty());

   EXPECT_NO_THROW((void)root->getCompositeI(0.0));
   EXPECT_GT(root->getCompositeMass(0.0), 0.0);
}

TEST(DiagnosticsGateTests, GateDoesNotReFirePerStepDuringBurn)
{
   // root counts radiusOuterAt queries; a ramp-mass child forces a per-step composite recompute.
   auto root = std::make_shared<CountingTube>("Body", 0.0, 0.02, 0.30, 680.0);
   auto tip  = std::make_shared<RampMassTube>("Tip", 0.0, 0.02, 0.05, 680.0); // OD 0.02 == root OD: clean abut
   root->addChildPart(tip, model::part::abut());

   // First inertia read resolves the geometry and runs the sweep ONCE (getCompositeI goes through
   // computeCompositeAt -> ensurePlacementCache).
   (void)root->getCompositeI(0.0);
   const int afterResolve = root->radiusCalls;
   ASSERT_GT(afterResolve, 0) << "the first composite-inertia read must resolve + sweep";
   ASSERT_TRUE(root->placementDiagnostics().ok);

   // Advance through a "burn": the ramp-mass child changes the composite mass each step, so the
   // mass-delta gate fires and computeCompositeAt genuinely re-runs (the CM shifts toward the growing
   // child). Yet the geometry/sweep cache is gated by placementDirty, so radiusOuterAt is never queried
   // again -- the sweep does not re-fire.
   double prevMass = root->getCompositeMass(0.0);
   double prevCmZ  = root->getCompositeCm(0.0).z();
   for(double t : {0.25, 0.5, 1.0, 2.0})
   {
      (void)root->getCompositeI(t); // forces computeCompositeAt at this t
      const double m   = root->getCompositeMass(t);
      const double cmZ = root->getCompositeCm(t).z();
      EXPECT_GT(m, prevMass) << "ramp-mass child should raise the composite mass each step";
      EXPECT_NE(cmZ, prevCmZ) << "computeCompositeAt should re-run and shift the CM each step";
      prevMass = m;
      prevCmZ  = cmZ;
   }
   EXPECT_EQ(root->radiusCalls, afterResolve) << "the sweep/resolve re-fired during the burn";
}
