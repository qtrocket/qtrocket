#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>

#include "model/parts/ConicalNoseCone.h"
#include "model/parts/Part.h"
#include "model/tests/TestPart.h"

namespace
{
constexpr double pi = std::numbers::pi;

// Independent per-unit-mass oracles (disk / surface integration), as in InertiaTensorsTests but
// re-applied here to the assembled ConicalNoseCone part (not just the raw helper).
struct AxisymInertia { double izz{0.0}; double ixx{0.0}; double cmFromApex{0.0}; };

AxisymInertia solidConeDiskIntegral(double R, double L, int n = 400000)
{
    const double dz = L / n;
    double m = 0.0, mz = 0.0;
    for(int i = 0; i < n; ++i)
    { const double z = (i + 0.5) * dz, r = R * z / L, dm = r * r; m += dm; mz += dm * z; }
    const double cm = mz / m;
    double izz = 0.0, ixx = 0.0;
    for(int i = 0; i < n; ++i)
    {
        const double z = (i + 0.5) * dz, r = R * z / L, dm = r * r;
        izz += dm * 0.5 * r * r;
        ixx += dm * (0.25 * r * r + (z - cm) * (z - cm));
    }
    return {izz / m, ixx / m, cm};
}

AxisymInertia conicalShellSurfaceIntegral(double R, double L, int n = 400000)
{
    const double dz = L / n;
    double m = 0.0, mz = 0.0;
    for(int i = 0; i < n; ++i)
    { const double z = (i + 0.5) * dz, r = R * z / L, dm = r; m += dm; mz += dm * z; }
    const double cm = mz / m;
    double izz = 0.0, ixx = 0.0;
    for(int i = 0; i < n; ++i)
    {
        const double z = (i + 0.5) * dz, r = R * z / L, dm = r;
        izz += dm * r * r;
        ixx += dm * (0.5 * r * r + (z - cm) * (z - cm));
    }
    return {izz / m, ixx / m, cm};
}

std::shared_ptr<model::part::Part> pointMass(const std::string& name, double mass)
{
    return std::make_shared<model::part::TestPart>(name, Matrix3::Zero(), mass, Vector3::Zero());
}
} // namespace

TEST(NoseConeTest, SolidMassMatchesClosedForm)
{
    const double R = 0.019, L = 0.10, density = 2700.0;
    model::part::ConicalNoseCone cone("nose", R, L, 0.0, density, true);
    EXPECT_NEAR(cone.getMass(0.0), density * (1.0 / 3.0) * pi * R * R * L, 1e-12);
    EXPECT_NEAR(cone.getReferenceArea(), pi * R * R, 1e-15);
    EXPECT_NEAR(cone.getMaxRadius(), R, 1e-15);
}

TEST(NoseConeTest, SolidCmOffsetIsLOver4FromBase)
{
    const double R = 0.019, L = 0.10;
    model::part::ConicalNoseCone cone("nose", R, L, 0.0, 2700.0, true);
    // CM is L/4 forward of the base (the wide, aft end) => -L/4 from the middle in the +z = forward
    // frame (base at -L/2 aft). (Corrected sign; see whitepaper 2.3.)
    const Vector3 off = cone.getCenterMassOffset();
    EXPECT_NEAR(off.z(), -L / 4.0, 1e-15);
    EXPECT_NEAR(off.x(), 0.0, 1e-15);
    EXPECT_NEAR(off.y(), 0.0, 1e-15);
}

TEST(NoseConeTest, SolidInertiaMatchesNumericDiskIntegration)
{
    const double R = 0.019, L = 0.10, density = 2700.0;
    model::part::ConicalNoseCone cone("nose", R, L, 0.0, density, true);

    const double mass = cone.getMass(0.0);
    const Matrix3 I = cone.getCompositeI(0.0); // full mass-weighted tensor about the CM
    const AxisymInertia oracle = solidConeDiskIntegral(R, L);

    EXPECT_NEAR(oracle.cmFromApex, 0.75 * L, 1e-6); // 3L/4 from apex == L/4 from base
    EXPECT_NEAR(I(2, 2) / mass, oracle.izz, 1e-5 * oracle.izz);
    EXPECT_NEAR(I(0, 0) / mass, oracle.ixx, 1e-5 * oracle.ixx);
    // Independently pins the (3/80)L^2 transverse term (it is CM-specific).
    EXPECT_NEAR(I(0, 0) / mass, 3.0 / 20.0 * R * R + 3.0 / 80.0 * L * L, 1e-12);
    EXPECT_NEAR(I(2, 2) / mass, 3.0 / 10.0 * R * R, 1e-12);
    EXPECT_DOUBLE_EQ(I(0, 1), 0.0);
    EXPECT_DOUBLE_EQ(I(0, 2), 0.0);
    EXPECT_DOUBLE_EQ(I(1, 2), 0.0);
}

TEST(NoseConeTest, ShellMassCmInertiaMatchSurfaceIntegral)
{
    const double R = 0.019, L = 0.10, t = 0.001, density = 2700.0;
    model::part::ConicalNoseCone cone("nose", R, L, t, density, false);

    // Thin shell mass = rho * t * pi R * slant.
    const double slant = std::sqrt(R * R + L * L);
    EXPECT_NEAR(cone.getMass(0.0), density * t * pi * R * slant, 1e-12);

    // CM is L/3 forward of the base => -L/6 from the middle in the +z = forward frame.
    EXPECT_NEAR(cone.getCenterMassOffset().z(), -L / 6.0, 1e-15);

    const double mass = cone.getMass(0.0);
    const Matrix3 I = cone.getCompositeI(0.0);
    const AxisymInertia oracle = conicalShellSurfaceIntegral(R, L);
    EXPECT_NEAR(oracle.cmFromApex, 2.0 / 3.0 * L, 1e-6);
    EXPECT_NEAR(I(2, 2) / mass, oracle.izz, 1e-5 * oracle.izz);
    EXPECT_NEAR(I(0, 0) / mass, oracle.ixx, 1e-5 * oracle.ixx);
    EXPECT_NEAR(I(2, 2) / mass, 1.0 / 2.0 * R * R, 1e-12);
    EXPECT_NEAR(I(0, 0) / mass, 1.0 / 4.0 * R * R + 1.0 / 18.0 * L * L, 1e-12);
}

TEST(NoseConeTest, AeroConeCNalphaAndCp)
{
    const double R = 0.019, L = 0.10;
    model::part::ConicalNoseCone solid("nose", R, L, 0.0, 2700.0, true);

    // CNalpha == 2 at the base reference area, scaling as (pi R^2 / refArea).
    EXPECT_NEAR(solid.getAero(pi * R * R).cnAlpha, 2.0, 1e-12);
    EXPECT_NEAR(solid.getAero(2.0 * pi * R * R).cnAlpha, 1.0, 1e-12);

    // x_cp reported from the cone's own CM in the corrected +z = forward frame: L/3 - hbar = +L/12
    // (solid), so cnAlphaXcp = 2 * (L/12) at refArea = pi R^2.
    EXPECT_NEAR(solid.getAero(pi * R * R).cnAlphaXcp, 2.0 * (L / 12.0), 1e-12);
    EXPECT_DOUBLE_EQ(solid.getAero(pi * R * R).cd, 0.0);

    // For the thin shell the CP coincides with the CM (both 2/3 L from the tip), so x_cp(from CM) = 0.
    model::part::ConicalNoseCone shell("nose", R, L, 0.001, 2700.0, false);
    EXPECT_NEAR(shell.getAero(pi * R * R).cnAlpha, 2.0, 1e-12);
    EXPECT_NEAR(shell.getAero(pi * R * R).cnAlphaXcp, 0.0, 1e-12);

    // Frame-independent anchor: whatever the datum, the cone CP is 2/3 L aft of the TIP. Reconstruct
    // the station relative to the tip -- x_cp_from_tip = cnAlphaXcp/cnAlpha (CM-relative) +
    // getCenterMassOffset().z() (CM->middle) - L/2 (middle->tip, tip at +L/2 in the +z = forward frame)
    // -- and confirm it is -(2/3)L for BOTH solid and shell, independent of the +L/12 / 0 CM arithmetic.
    auto tipStation = [&](const model::part::ConicalNoseCone& c)
    {
        const model::AeroComponent a = c.getAero(pi * R * R);
        return a.cnAlphaXcp / a.cnAlpha + c.getCenterMassOffset().z() - L / 2.0;
    };
    EXPECT_NEAR(tipStation(solid), -2.0 / 3.0 * L, 1e-12);
    EXPECT_NEAR(tipStation(shell), -2.0 / 3.0 * L, 1e-12);
}

// CP is a function of external SHAPE only, never of mass distribution. A solid cone and a thin-shell
// cone of identical base radius and length are aerodynamically identical (same CNalpha, same CP at
// 2/3 L aft of the tip) but have DIFFERENT centers of mass (-L/4 vs -L/6 from the middle). So the
// COMPOSITE cp must be the textbook -2/3 L for BOTH, independent of the CM. This locks in a placement-
// migration correction: the legacy aero walk leaked each part's own CM into cnAlphaXcp/cnAlpha (it gave
// +L/12 vs 0 for these two cones -- the source of the static-margin drift across the corpus), which the
// migrated CM-station formula (Part::getCompositeAero) exactly cancels. A regression that re-contaminates
// cp with the CM would split these two values apart.
TEST(NoseConeTest, CompositeCpIsCmIndependentSolidVsShell)
{
    const double R = 0.0395, L = 0.30;
    const double refArea = pi * R * R;
    auto solid = std::make_shared<model::part::ConicalNoseCone>("solid", R, L, 0.0,   1700.0, true);
    auto shell = std::make_shared<model::part::ConicalNoseCone>("shell", R, L, 0.001, 1700.0, false);

    // Precondition: the two cones really DO have different CM (else CM-independence proves nothing).
    ASSERT_NEAR(solid->getCenterMassOffset().z(), -L / 4.0, 1e-12);
    ASSERT_NEAR(shell->getCenterMassOffset().z(), -L / 6.0, 1e-12);

    // ... yet identical composite CP, equal to the textbook 2/3 L aft of the tip (CM-independent).
    const double cpSolid = solid->getCompositeAero(refArea).cp();
    const double cpShell = shell->getCompositeAero(refArea).cp();
    EXPECT_NEAR(cpSolid, -2.0 / 3.0 * L, 1e-12);
    EXPECT_NEAR(cpShell, -2.0 / 3.0 * L, 1e-12);
    EXPECT_DOUBLE_EQ(cpSolid, cpShell);
}

TEST(NoseConeTest, ReducesToShellVsSolidCorrectly)
{
    // The `solid` flag must switch BOTH the CM location and the inertia tensor (guards against the flag
    // being ignored / one branch always taken). Same R, L for a direct contrast.
    const double R = 0.02, L = 0.12;
    model::part::ConicalNoseCone solid("s", R, L, 0.001, 2700.0, true);
    model::part::ConicalNoseCone shell("h", R, L, 0.001, 2700.0, false);

    // CM: solid is L/4 forward of the base (-L/4 from the middle), shell L/3 forward (-L/6). Distinct.
    EXPECT_NEAR(solid.getCenterMassOffset().z(), -L / 4.0, 1e-15);
    EXPECT_NEAR(shell.getCenterMassOffset().z(), -L / 6.0, 1e-15);
    EXPECT_LT(solid.getCenterMassOffset().z(), shell.getCenterMassOffset().z()); // -L/4 < -L/6

    // Per-unit-mass tensors differ: shell Izz/m = R^2/2 strictly exceeds solid 3R^2/10; the transverse
    // moments also differ.
    EXPECT_NEAR(solid.getI()(2, 2), 3.0 / 10.0 * R * R, 1e-15);
    EXPECT_NEAR(shell.getI()(2, 2), 1.0 / 2.0 * R * R, 1e-15);
    EXPECT_GT(shell.getI()(2, 2), solid.getI()(2, 2));
    EXPECT_NEAR(solid.getI()(0, 0), 3.0 / 20.0 * R * R + 3.0 / 80.0 * L * L, 1e-15);
    EXPECT_NEAR(shell.getI()(0, 0), 1.0 / 4.0 * R * R + 1.0 / 18.0 * L * L, 1e-15);
    EXPECT_NE(solid.getI()(0, 0), shell.getI()(0, 0));
}

TEST(NoseConeTest, RejectsNonPhysical)
{
    using model::part::ConicalNoseCone;
    EXPECT_THROW(ConicalNoseCone("bad", 0.0,  0.10, 0.0, 2700.0, true),  std::invalid_argument); // R<=0
    EXPECT_THROW(ConicalNoseCone("bad", 0.019, 0.0, 0.0, 2700.0, true),  std::invalid_argument); // L<=0
    EXPECT_THROW(ConicalNoseCone("bad", 0.019, 0.10, 0.0, 0.0,  true),   std::invalid_argument); // rho<=0
    EXPECT_THROW(ConicalNoseCone("bad", 0.019, 0.10, 0.019, 2700.0, false), std::invalid_argument); // t>=R
    EXPECT_THROW(ConicalNoseCone("bad", 0.019, 0.10, 0.0, 2700.0, false), std::invalid_argument); // t<=0 shell
    EXPECT_NO_THROW(ConicalNoseCone("ok", 0.019, 0.10, 0.0, 2700.0, true)); // solid ignores t
}

TEST(NoseConeTest, CloneIsDeepTypePreserving)
{
    auto cone = std::make_shared<model::part::ConicalNoseCone>("nose", 0.019, 0.10, 0.0, 2700.0, true);
    cone->addChildPart(pointMass("ballast", 0.02), model::part::abut(0.03));

    auto copy = cone->clone();
    const double massBefore = copy->getCompositeMass(0.0);
    const double iyyBefore = copy->getCompositeI(0.0)(1, 1);

    cone->setMass(99.0);
    cone->addChildPart(pointMass("extra", 50.0), model::part::abut(1.0));

    EXPECT_DOUBLE_EQ(copy->getCompositeMass(0.0), massBefore);
    EXPECT_DOUBLE_EQ(copy->getCompositeI(0.0)(1, 1), iyyBefore);
    EXPECT_NE(dynamic_cast<model::part::ConicalNoseCone*>(copy.get()), nullptr);
    EXPECT_NE(copy->getId(), cone->getId());
}
