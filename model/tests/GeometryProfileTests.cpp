// T1 -- geometry profile unit tests (implementation plan Part II Step 4, Part III T1). These pin the
// closed-form extent profiles added to Part and the four concrete geometry types: the cone's linear
// taper (and its zero-length divide guard), the tube's constant walls, the fin set's body disc (NOT
// its tip extent), the hollow sphere's silhouette, the solid-host capacity rule, the station mapping
// with its [0,1] clamp, and the zero-length point collapse. The profiles are unconsumed in Step 4, so
// these tests are the only thing exercising them until the resolver/sweep land (Steps 5-6).

#include <gtest/gtest.h>

#include <cmath>

#include "model/parts/BodyTube.h"
#include "model/parts/ConicalNoseCone.h"
#include "model/parts/FinSet.h"
#include "model/parts/HollowSphere.h"
#include "model/parts/Part.h"
#include "model/parts/Placement.h"
#include "model/tests/TestPart.h"
#include "utils/math/MathTypes.h"

namespace
{
using model::part::BodyTube;
using model::part::ConicalNoseCone;
using model::part::FinSet;
using model::part::HollowSphere;
using model::part::Part;
}  // namespace

// ---- Cone ---------------------------------------------------------------------------------------

TEST(GeometryProfileTests, ConeTaperLinear)
{
   // xl75 nose geometry: baseRadius 0.0395, L 0.30, solid. Tip (z=0) -> 0, base (z=-L) -> baseRadius.
   const ConicalNoseCone cone("Nose", 0.0395, 0.30, 0.0, 1700.0, true);
   EXPECT_DOUBLE_EQ(cone.radiusOuterAt(0.0), 0.0);            // tip / fore plane
   EXPECT_DOUBLE_EQ(cone.radiusOuterAt(-0.30), 0.0395);       // base rim / aft plane
   EXPECT_NEAR(cone.radiusOuterAt(-0.26), 0.0395 * (0.26 / 0.30), 1e-15);  // ~0.0342
   EXPECT_DOUBLE_EQ(cone.axialLength(), 0.30);
}

TEST(GeometryProfileTests, ConeTaperDegenerateGuard)
{
   // A near-zero-length cone (L <= 1e-9): the taper divide is guarded, so radiusOuterAt returns the
   // base radius for any station instead of evaluating 0/0. (L must be > 0 to construct at all.)
   const ConicalNoseCone flat("Flat", 0.02, 1e-10, 0.0, 1700.0, true);
   EXPECT_DOUBLE_EQ(flat.radiusOuterAt(0.0), 0.02);
   EXPECT_DOUBLE_EQ(flat.radiusOuterAt(-1.0), 0.02);
   EXPECT_TRUE(std::isfinite(flat.radiusOuterAt(-1.0)));
}

TEST(GeometryProfileTests, ConeIsSolidAndHasNoBore)
{
   const ConicalNoseCone cone("Nose", 0.0395, 0.30, 0.0, 1700.0, true);
   EXPECT_TRUE(cone.isSolid());
   EXPECT_DOUBLE_EQ(cone.radiusInnerAt(-0.15), 0.0);  // solid: no bore anywhere
   // Solid-host rule: a solid part's capacity is its outer skin (the poke-through test), not its bore.
   EXPECT_DOUBLE_EQ(cone.innerCapacityAt(-0.26), cone.radiusOuterAt(-0.26));

   // A shell cone reports its authored solidity -- the base default (bore at z=0) could not tell it
   // apart from a solid cone, since the cone models no bore.
   const ConicalNoseCone shell("Shell", 0.0395, 0.30, 0.001, 1700.0, false);
   EXPECT_FALSE(shell.isSolid());
   EXPECT_DOUBLE_EQ(shell.innerCapacityAt(-0.15), shell.radiusInnerAt(-0.15));  // bored -> 0
}

// ---- Tube ---------------------------------------------------------------------------------------

TEST(GeometryProfileTests, TubeConstantWalls)
{
   // xl75 coupler: ri 0.036, ro 0.0376, L 0.08 -- constant walls over the whole span.
   const BodyTube coupler("Coupler", 0.036, 0.0376, 0.08, 1700.0);
   for(double z : {0.0, -0.02, -0.04, -0.08})
   {
      EXPECT_DOUBLE_EQ(coupler.radiusOuterAt(z), 0.0376);
      EXPECT_DOUBLE_EQ(coupler.radiusInnerAt(z), 0.036);
   }
   EXPECT_FALSE(coupler.isSolid());
   EXPECT_DOUBLE_EQ(coupler.innerCapacityAt(-0.04), 0.036);  // bored -> capacity is the bore
   EXPECT_DOUBLE_EQ(coupler.axialLength(), 0.08);
}

TEST(GeometryProfileTests, TubeSolidRodIsSolid)
{
   const BodyTube rod("Rod", 0.0, 0.019, 0.20, 680.0);  // ri == 0
   EXPECT_TRUE(rod.isSolid());
   EXPECT_DOUBLE_EQ(rod.radiusInnerAt(-0.10), 0.0);
   EXPECT_DOUBLE_EQ(rod.innerCapacityAt(-0.10), 0.019);  // solid -> capacity is the outer skin
}

// ---- FinSet -------------------------------------------------------------------------------------

TEST(GeometryProfileTests, FinSetReportsBodyDiscOnly)
{
   // xl75 fins: rootChord 0.10, span 0.06, bodyRadius 0.0395. The sweep must see the body disc, NOT
   // the bodyRadius + span tip extent (which would false-collide with every adjacent tube).
   const FinSet fins("Fins", 6, 0.10, 0.04, 0.06, 0.04, 0.003, 0.0395, 1700.0);
   EXPECT_DOUBLE_EQ(fins.radiusOuterAt(-0.05), fins.getBodyRadius());
   EXPECT_DOUBLE_EQ(fins.radiusOuterAt(-0.05), 0.0395);
   EXPECT_NE(fins.radiusOuterAt(-0.05), fins.getMaxRadius());  // NOT bodyRadius + span
   EXPECT_DOUBLE_EQ(fins.axialLength(), 0.10);                 // = rootChord
   EXPECT_TRUE(fins.isSolid());                                // base default: no bore
}

// ---- HollowSphere -------------------------------------------------------------------------------

TEST(GeometryProfileTests, SphereSilhouetteAndDiameter)
{
   // Solid sphere ro = 0.05 (ri = 0): center at z = -0.05, span [-0.10, 0].
   const HollowSphere solid("Ball", 0.0, 0.05, 2700.0);
   EXPECT_DOUBLE_EQ(solid.axialLength(), 0.10);            // diameter, pole to pole
   EXPECT_DOUBLE_EQ(solid.radiusOuterAt(0.0), 0.0);        // +z pole
   EXPECT_DOUBLE_EQ(solid.radiusOuterAt(-0.10), 0.0);      // -z pole
   EXPECT_NEAR(solid.radiusOuterAt(-0.05), 0.05, 1e-12);   // equator -> outerRadius
   EXPECT_TRUE(solid.isSolid());

   // Shell ri 0.04, ro 0.05: the cavity bulges to ri at the equator and vanishes outside the band.
   const HollowSphere shell("Shell", 0.04, 0.05, 2700.0);
   EXPECT_FALSE(shell.isSolid());
   EXPECT_NEAR(shell.radiusInnerAt(-0.05), 0.04, 1e-12);   // inner cavity at the equator
   EXPECT_DOUBLE_EQ(shell.radiusInnerAt(0.0), 0.0);        // pole: outside the inner band
   EXPECT_NEAR(shell.innerCapacityAt(-0.05), 0.04, 1e-12); // bored -> capacity is the cavity
}

// ---- stationAt + degenerate collapse ------------------------------------------------------------

TEST(GeometryProfileTests, StationAtClampsAndMaps)
{
   const ConicalNoseCone cone("Nose", 0.0395, 0.30, 0.0, 1700.0, true);
   EXPECT_DOUBLE_EQ(cone.stationAt(1.0).z, 0.0);     // fore plane (tip)
   EXPECT_DOUBLE_EQ(cone.stationAt(0.0).z, -0.30);   // aft plane (base)
   EXPECT_DOUBLE_EQ(cone.stationAt(0.5).z, -0.15);   // mid-length

   // The landmark also carries the radii read from the profile at z.
   EXPECT_DOUBLE_EQ(cone.stationAt(0.0).rOuter, 0.0395);  // base rim
   EXPECT_DOUBLE_EQ(cone.stationAt(1.0).rOuter, 0.0);     // tip

   // Out-of-range fractions clamp to the nearest end (degenerate guard).
   EXPECT_DOUBLE_EQ(cone.stationAt(-0.1).z, cone.stationAt(0.0).z);
   EXPECT_DOUBLE_EQ(cone.stationAt(1.1).z, cone.stationAt(1.0).z);
}

TEST(GeometryProfileTests, ZeroLengthCollapsesToPoint)
{
   // A bare node has getLength() == 0 (a geometrically inert node), so every station maps to z = 0 --
   // the sweep treats it as a single point sample rather than dividing by a zero span. TestPart is the
   // concrete stand-in for the (now abstract) base Part.
   const model::part::TestPart point("Point", Matrix3::Identity(), 1.0, Vector3::Zero());
   EXPECT_DOUBLE_EQ(point.axialLength(), 0.0);
   EXPECT_DOUBLE_EQ(point.stationAt(0.0).z, 0.0);
   EXPECT_DOUBLE_EQ(point.stationAt(0.5).z, 0.0);
   EXPECT_DOUBLE_EQ(point.stationAt(1.0).z, 0.0);
}
