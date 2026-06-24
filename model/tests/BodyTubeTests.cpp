#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>

#include "model/InertiaTensors.h"
#include "model/parts/BodyTube.h"
#include "model/parts/Part.h"

namespace
{
constexpr double pi = std::numbers::pi;

// Independent closed forms for a uniform hollow cylinder, re-derived (not copied from
// InertiaTensors::Tube): mass = rho * pi (ro^2 - ri^2) L; per-unit-mass moments from a stack of
// annular disks -- (1/2)(ri^2+ro^2) about the central axis, (1/4)(ri^2+ro^2) about a diameter, plus
// the L^2/12 axial spread.
double tubeMass(double ri, double ro, double L, double density)
{
   return density * pi * (ro * ro - ri * ri) * L;
}
double tubeIzzPerMass(double ri, double ro) { return 0.5 * (ri * ri + ro * ro); }
double tubeIxxPerMass(double ri, double ro, double L)
{
   return 0.25 * (ri * ri + ro * ro) + L * L / 12.0;
}

std::shared_ptr<model::part::Part> pointMass(const std::string& name, double mass)
{
   return std::make_shared<model::part::Part>(name, Matrix3::Zero(), mass, Vector3::Zero());
}
} // namespace

TEST(BodyTubeTest, MassMatchesHollowCylinder)
{
   const double ri = 0.018, ro = 0.019, L = 0.30, density = 680.0; // ~ a cardboard 38 mm tube
   model::part::BodyTube tube("body", ri, ro, L, density);

   EXPECT_NEAR(tube.getMass(0.0), tubeMass(ri, ro, L, density), 1e-12);
   EXPECT_NEAR(tube.getReferenceArea(), pi * ro * ro, 1e-15);
   EXPECT_NEAR(tube.getWettedArea(), 2.0 * pi * ro * L, 1e-15);
   EXPECT_NEAR(tube.getMaxRadius(), ro, 1e-15);
}

TEST(BodyTubeTest, CompositeIEqualsMassTimesTube)
{
   const double ri = 0.018, ro = 0.019, L = 0.30, density = 680.0;
   model::part::BodyTube tube("body", ri, ro, L, density);

   const double mass = tube.getMass(0.0);
   const Matrix3 I = tube.getCompositeI(0.0); // full mass-weighted tensor (kg*m^2)

   // Wires the per-unit-mass tensor through the Part base correctly ...
   const Matrix3 expected = mass * model::InertiaTensors::Tube(ri, ro, L);
   EXPECT_NEAR(I(0, 0), expected(0, 0), 1e-12);
   EXPECT_NEAR(I(2, 2), expected(2, 2), 1e-12);

   // ... and those values equal the independent disk-stack closed forms.
   EXPECT_NEAR(I(0, 0), mass * tubeIxxPerMass(ri, ro, L), 1e-12);
   EXPECT_NEAR(I(1, 1), mass * tubeIxxPerMass(ri, ro, L), 1e-12);
   EXPECT_NEAR(I(2, 2), mass * tubeIzzPerMass(ri, ro), 1e-12);
   EXPECT_DOUBLE_EQ(I(0, 1), 0.0);
   EXPECT_DOUBLE_EQ(I(0, 2), 0.0);
   EXPECT_DOUBLE_EQ(I(1, 2), 0.0);
}

TEST(BodyTubeTest, TwoBodyTubesEndToEndEqualOneLongerTube)
{
   // Two coaxial body tubes of identical radii, stacked end-to-end along z, must be indistinguishable
   // from one tube of the summed length: same mass, CM at the merged center, same full inertia. The
   // transverse moment depends on L^2, so this sharply exercises the parallel-axis composition.
   const double ri = 0.018, ro = 0.019, density = 680.0;
   const double L1 = 0.10, L2 = 0.20;

   auto assembly = std::make_shared<model::part::BodyTube>("t1", ri, ro, L1, density);
   assembly->addChildPart(std::make_shared<model::part::BodyTube>("t2", ri, ro, L2, density),
                          model::part::StationLink{.parentStation01 = 1.0, .childStation01 = 0.0,
                                                  .seat = model::part::SeatKind::Abut});

   const double totalLength = L1 + L2;
   const double totalMass = tubeMass(ri, ro, totalLength, density);

   EXPECT_NEAR(assembly->getCompositeMass(0.0), totalMass, 1e-12);

   const Vector3 cm = assembly->getCompositeCm(0.0);
   EXPECT_NEAR(cm(0), 0.0, 1e-12);
   EXPECT_NEAR(cm(1), 0.0, 1e-12);
   // Composite CG is now reported in the root's fore-plane (tip) datum, not the root's own CM: it
   // shifts by cmLocalZ_root = -L1/2, so the merged center sits at L2/2 - L1/2 = (L2 - L1)/2.
   EXPECT_NEAR(cm(2), (L2 - L1) / 2.0, 1e-12);

   const Matrix3 merged = totalMass * model::InertiaTensors::Tube(ri, ro, totalLength);
   const Matrix3 I = assembly->getCompositeI(0.0);
   for(int r = 0; r < 3; ++r)
   {
      for(int c = 0; c < 3; ++c)
      {
         SCOPED_TRACE(testing::Message() << "inertia element (" << r << ", " << c << ")");
         EXPECT_NEAR(I(r, c), merged(r, c), 1e-12);
      }
   }
}

TEST(BodyTubeTest, AeroBodyHasZeroCNalpha)
{
   model::part::BodyTube tube("body", 0.018, 0.019, 0.30, 680.0);
   const sim::AeroComponent aero = tube.getAero(pi * 0.019 * 0.019);
   EXPECT_DOUBLE_EQ(aero.cnAlpha, 0.0);
   EXPECT_DOUBLE_EQ(aero.cnAlphaXcp, 0.0);
   EXPECT_DOUBLE_EQ(aero.cd, 0.0);
}

TEST(BodyTubeTest, RejectsNonPhysical)
{
   EXPECT_THROW(model::part::BodyTube("bad", 0.020, 0.019, 0.30, 680.0), std::invalid_argument); // ri>ro
   EXPECT_THROW(model::part::BodyTube("bad", 0.019, 0.019, 0.30, 680.0), std::invalid_argument); // ri==ro
   EXPECT_THROW(model::part::BodyTube("bad", 0.018, 0.019, 0.00, 680.0), std::invalid_argument); // L==0
   EXPECT_THROW(model::part::BodyTube("bad", 0.018, 0.019, 0.30, 0.0),   std::invalid_argument); // rho==0
}

TEST(BodyTubeTest, CloneIsDeepTypePreserving)
{
   auto tube = std::make_shared<model::part::BodyTube>("body", 0.018, 0.019, 0.30, 680.0);
   tube->addChildPart(pointMass("tip", 0.05), model::part::abut(0.2));

   auto copy = tube->clone();
   const double massBefore = copy->getCompositeMass(0.0);
   const double iyyBefore = copy->getCompositeI(0.0)(1, 1);

   tube->setMass(99.0);
   tube->addChildPart(pointMass("extra", 50.0), model::part::abut(1.0));

   EXPECT_DOUBLE_EQ(copy->getCompositeMass(0.0), massBefore);
   EXPECT_DOUBLE_EQ(copy->getCompositeI(0.0)(1, 1), iyyBefore);
   EXPECT_NE(dynamic_cast<model::part::BodyTube*>(copy.get()), nullptr);
   EXPECT_NE(copy->getId(), tube->getId());
}
