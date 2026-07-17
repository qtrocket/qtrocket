#include <gtest/gtest.h>

#include <memory>
#include <numbers>
#include <stdexcept>

#include "model/InertiaTensors.h"
#include "model/PartsModel.h"
#include "model/parts/BodyTube.h"
#include "model/parts/HollowSphere.h"
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

    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(
        std::make_unique<model::part::BodyTube>("body", ri, ro, L, density)));
    const model::PartNode* root = pm.root();
    ASSERT_NE(root, nullptr);

    const double mass = root->part().getMass(0.0);
    const Matrix3 I = root->compositeI(0.0); // full mass-weighted tensor (kg*m^2)

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

    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(
        std::make_unique<model::part::BodyTube>("t1", ri, ro, L1, density)));
    const auto attached = pm.attach(
        pm.root()->id(), std::make_unique<model::part::BodyTube>("t2", ri, ro, L2, density),
        model::part::StationLink{.parentStation01 = 1.0, .childStation01 = 0.0,
                                        .seat = model::part::SeatKind::Abut});
    ASSERT_TRUE(attached.has_value());
    const model::PartNode& root = *pm.root();

    const double totalLength = L1 + L2;
    const double totalMass = tubeMass(ri, ro, totalLength, density);

    EXPECT_NEAR(root.compositeMass(0.0), totalMass, 1e-12);

    const Vector3 cm = root.compositeCm(0.0);
    EXPECT_NEAR(cm(0), 0.0, 1e-12);
    EXPECT_NEAR(cm(1), 0.0, 1e-12);
    // Composite CG is reported in the root's fore-plane (tip) datum: the stack occupies z in
    // [-L1, L2], so the merged center sits at (L2 - L1)/2.
    EXPECT_NEAR(cm(2), (L2 - L1) / 2.0, 1e-12);

    const Matrix3 merged = totalMass * model::InertiaTensors::Tube(ri, ro, totalLength);
    const Matrix3 I = root.compositeI(0.0);
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
    const model::AeroComponent aero = tube.getAero(pi * 0.019 * 0.019);
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

TEST(BodyTubeTest, TubeAndSphereCmAtMid)
{
    // T3 uniform-rule: an axially symmetric part has its own CM at mid-length, so getCenterMassOffset().z()
    // is exactly 0 and its local CM station is -L/2. Pinned DIRECTLY here (not via a composite) for the two
    // symmetric primitives -- a body tube and a hollow sphere -- so a stray non-zero offset cannot hide.
    const double cmTol = 1e-15;

    const double ri = 0.018, ro = 0.019, L = 0.30;
    model::part::BodyTube tube("body", ri, ro, L, 680.0);
    EXPECT_NEAR(tube.getCenterMassOffset().z(), 0.0, cmTol);
    EXPECT_NEAR(-tube.getLength() / 2.0 + tube.getCenterMassOffset().z(), -L / 2.0, cmTol);  // cmLocalZ

    const double sphereRo = 0.05;  // axialLength = 2*ro, so -L/2 = -ro
    model::part::HollowSphere sphere("ball", 0.0, sphereRo, 900.0);
    EXPECT_NEAR(sphere.getCenterMassOffset().z(), 0.0, cmTol);
    EXPECT_NEAR(sphere.getLength(), 2.0 * sphereRo, cmTol);
    EXPECT_NEAR(-sphere.getLength() / 2.0 + sphere.getCenterMassOffset().z(), -sphereRo, cmTol);  // cmLocalZ
}

TEST(BodyTubeTest, CloneIsDeepTypePreserving)
{
    const double ri = 0.018, ro = 0.019, density = 680.0;

    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(
        std::make_unique<model::part::BodyTube>("body", ri, ro, 0.30, density)));
    const auto tip = pm.attach(
        pm.root()->id(), std::make_unique<model::part::BodyTube>("tip", ri, ro, 0.05, density),
        model::part::abut(0.2));
    ASSERT_TRUE(tip.has_value());

    const std::unique_ptr<model::PartNode> copy = pm.root()->clone();
    const double massBefore = copy->compositeMass(0.0);
    const double iyyBefore = copy->compositeI(0.0)(1, 1);

    // edits to the source tree must not leak into the detached copy
    ASSERT_TRUE(pm.setPartMass(pm.root()->id(), 99.0));
    ASSERT_TRUE(pm.attach(pm.root()->id(),
                          std::make_unique<model::part::BodyTube>("extra", ri, ro, 0.10, density),
                          model::part::abut(1.0)).has_value());

    EXPECT_DOUBLE_EQ(copy->compositeMass(0.0), massBefore);
    EXPECT_DOUBLE_EQ(copy->compositeI(0.0)(1, 1), iyyBefore);

    // type-preserving, with fresh ids at every node of the copy
    EXPECT_NE(dynamic_cast<const model::part::BodyTube*>(&copy->part()), nullptr);
    EXPECT_NE(copy->id(), pm.root()->id());
    ASSERT_EQ(copy->children().size(), 1u);
    EXPECT_NE(copy->children()[0]->id(), *tip);
}
