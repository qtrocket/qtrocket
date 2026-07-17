#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <utility>
#include <vector>

#include "model/MotorModel.h"
#include "model/PartsModel.h"
#include "model/RocketModel.h"
#include "model/tests/PlacementTestSupport.h"
#include "model/ThrustCurve.h"
#include "model/parts/Part.h"
#include "model/parts/Parts.h"
#include "model/Aero.h"

// Exercises the Barrowman composition seam: per-part getAero folded by PartNode::compositeAero into a
// whole-rocket AeroProfile, every x_cp re-expressed on the sub-tree-root datum shared with
// compositeCm, so cp() - cg is the static margin.

namespace
{
constexpr double pi = std::numbers::pi;

// A minimal Part whose getAero returns a fixed AeroComponent, to test the composition walk's
// accumulation/station-threading independently of the real parts' zero cd.
class FixedAeroPart : public model::part::Part
{
public:
    FixedAeroPart(const std::string& name, model::AeroComponent aero)
        : Part(name, Matrix3::Zero(), 1.0, Vector3::Zero()), aero_(aero) {}
    model::AeroComponent getAero(double /*refArea*/) const override { return aero_; }
    std::string typeName() const override { return "FixedAeroPart"; } // Part is abstract; concrete stub

protected:
    FixedAeroPart(const FixedAeroPart&) = default; // uses Part's protected copy ctor (fresh id)
    std::unique_ptr<model::part::Part> clone() const override
    { return std::unique_ptr<model::part::Part>(new FixedAeroPart(*this)); }

private:
    model::AeroComponent aero_;
};

double finAxialMassCentroid(double cr, double ct, double sweep)
{
    return (cr * cr + cr * ct + ct * ct + sweep * (cr + 2.0 * ct)) / (3.0 * (cr + ct));
}

// Synthetic flat-thrust motor (mirrors the MotorTests helper) so a real Motor part -- not a stub --
// can be attached to the airframe tree.
model::MotorModel makeTestMotor(double totalWeight, double propWeight, double burnTime,
                                           double totalImpulse)
{
    const double flatThrust = totalImpulse / burnTime;
    std::vector<std::pair<double, double>> samples{{0.0, flatThrust}, {burnTime, flatThrust}};
    ThrustCurve tc(samples);
    model::MotorModel m;
    m.addThrustCurve(tc); // must precede setMetaData (computeMassCurve reads the curve)
    model::MotorModel::MetaData md;
    md.totalWeight = totalWeight; md.propWeight = propWeight;
    md.burnTime = burnTime;       md.totalImpulse = totalImpulse;
    md.diameter = 24.0; md.length = 70.0; // mm
    m.setMetaData(md);
    return m;
}
} // namespace

TEST(AeroTest, CompositeCpIsCNalphaWeighted)
{
    const double R = 0.019, Lnose = 0.10;
    const double refArea = pi * R * R;
    auto nose = std::make_unique<model::part::ConicalNoseCone>("nose", R, Lnose, 0.0, 900.0, true);
    auto body = std::make_unique<model::part::BodyTube>("body", 0.018, R, 0.40, 680.0);
    auto fins = std::make_unique<model::part::FinSet>("fins", 3, 0.10, 0.05, 0.05, 0.04, 0.003, R, 600.0);

    // Per-part aero is a pure function of geometry (independent of placement) -- capture before moving.
    const model::AeroComponent na = nose->getAero(refArea);
    const model::AeroComponent ba = body->getAero(refArea);
    const model::AeroComponent fa = fins->getAero(refArea);
    EXPECT_DOUBLE_EQ(ba.cnAlpha, 0.0); // the body carries no normal force

    const double zBody = 0.225, zFins = 0.38; // CM-to-CM z stations from the nose CM (root datum)
    const auto bodyLink = model::part::test::cmToCm(*nose, *body, zBody);
    const auto finsLink = model::part::test::cmToCm(*nose, *fins, zFins);
    auto root = model::PartNode::make(std::move(nose));
    root->addChild(model::PartNode::make(std::move(body)), bodyLink);
    root->addChild(model::PartNode::make(std::move(fins)), finsLink);
    model::PartsModel pm;
    pm.installRoot(std::move(root));

    const model::AeroProfile prof = pm.root()->compositeAero(refArea);

    // Hand-assembled CNalpha-weighted CP. Each part's x_cp is its CM-relative value (cnAlphaXcp) plus
    // cnAlpha * station; the zero-CNalpha body drops out automatically. The composite cp is reported
    // in the tip datum, so it equals the root-CM-datum value plus the single shift cmLocalZ_root (the
    // solid nose CM station = -3 L/4).
    const double cmLocalZRoot    = -3.0 * Lnose / 4.0;
    const double expectedCnAlpha = na.cnAlpha + fa.cnAlpha;
    const double expectedMoment  = (na.cnAlphaXcp + na.cnAlpha * 0.0)
                                           + (fa.cnAlphaXcp + fa.cnAlpha * zFins);
    EXPECT_NEAR(prof.cnAlpha, expectedCnAlpha, 1e-12);
    EXPECT_TRUE(prof.cpValid);
    EXPECT_NEAR(prof.cp(), expectedMoment / expectedCnAlpha + cmLocalZRoot, 1e-12);

    // The body must not corrupt the average: dropping its (zero) term changes nothing.
    const double cpWithoutBody = (na.cnAlphaXcp + fa.cnAlphaXcp + fa.cnAlpha * zFins)
                                        / (na.cnAlpha + fa.cnAlpha);
    EXPECT_NEAR(prof.cp(), cpWithoutBody + cmLocalZRoot, 1e-12);
}

TEST(AeroTest, CompositeCdIsAdditive)
{
    auto root = model::PartNode::make(
        std::make_unique<FixedAeroPart>("root", model::AeroComponent{0.0, 0.0, 0.10}));
    root->addChild(model::PartNode::make(
                       std::make_unique<FixedAeroPart>("c1", model::AeroComponent{0.5, 0.0, 0.20})),
                   model::part::abut(0.10));
    root->addChild(model::PartNode::make(
                       std::make_unique<FixedAeroPart>("c2", model::AeroComponent{0.0, 0.0, 0.30})),
                   model::part::abut(0.20));
    model::PartsModel pm;
    pm.installRoot(std::move(root));

    const model::AeroProfile prof = pm.root()->compositeAero(1.0);
    EXPECT_NEAR(prof.cd, 0.60, 1e-12);      // cd adds over all parts
    EXPECT_NEAR(prof.cnAlpha, 0.50, 1e-12); // only c1 carries CNalpha
    EXPECT_NEAR(prof.refArea, 1.0, 1e-12);
}

TEST(AeroTest, BodyOnlyRocketHasInvalidCp)
{
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(
        std::make_unique<model::part::BodyTube>("body", 0.018, 0.019, 0.40, 680.0)));
    const model::AeroProfile prof = pm.root()->compositeAero(pi * 0.019 * 0.019);
    EXPECT_DOUBLE_EQ(prof.cnAlpha, 0.0);
    EXPECT_FALSE(prof.cpValid);    // CP is undefined when total CNalpha == 0
    EXPECT_DOUBLE_EQ(prof.cp(), 0.0); // guarded divide
}

TEST(AeroTest, SharedReferenceAreaInvariant)
{
    // Nose (native ref = pi*R^2) and fins (native ref = pi*rb^2) have DIFFERENT native reference areas;
    // they are only additive once both rescale to one shared refArea. compositeAero passes a single
    // refArea to every getAero, so: (a) composite CNalpha == sum of the rescaled parts, and (b) cp() is
    // invariant to the choice of refArea (a pure ratio), while CNalpha scales as 1/refArea.
    const double R = 0.025, rb = 0.019;
    auto nose = std::make_unique<model::part::ConicalNoseCone>("nose", R, 0.12, 0.0, 900.0, true);
    auto fins = std::make_unique<model::part::FinSet>("fins", 4, 0.10, 0.05, 0.05, 0.04, 0.003, rb, 600.0);

    const double rA = pi * R * R;
    const model::AeroComponent noseA = nose->getAero(rA);
    const model::AeroComponent finsA = fins->getAero(rA);
    const auto finsLink = model::part::test::cmToCm(*nose, *fins, 0.30);
    auto root = model::PartNode::make(std::move(nose));
    root->addChild(model::PartNode::make(std::move(fins)), finsLink);
    model::PartsModel pm;
    pm.installRoot(std::move(root));

    const model::AeroProfile profA = pm.root()->compositeAero(rA);
    EXPECT_NEAR(profA.cnAlpha, noseA.cnAlpha + finsA.cnAlpha, 1e-12); // additive only after rescaling

    const double rB = pi * rb * rb;
    const model::AeroProfile profB = pm.root()->compositeAero(rB);
    EXPECT_NEAR(profA.cp(), profB.cp(), 1e-12);                  // CP independent of refArea
    EXPECT_NEAR(profA.cnAlpha * rA, profB.cnAlpha * rB, 1e-12);  // CNalpha scales as 1/refArea
}

TEST(AeroTest, AssembledRocketCmMassAndRefArea)
{
    // nose + body + fins stacked along z at REAL CM-to-CM offsets -- exercising the cone's non-central
    // CM. Equal radii (R == ro == rb) so the reference disc is unambiguous.
    const double R = 0.019, Lnose = 0.10;
    const double ri = 0.018, Lbody = 0.40;
    const double cr = 0.10, ct = 0.05, s = 0.05, sweep = 0.04, thk = 0.003;

    auto nose = std::make_unique<model::part::ConicalNoseCone>("nose", R, Lnose, 0.0, 900.0, true);
    auto body = std::make_unique<model::part::BodyTube>("body", ri, R, Lbody, 680.0);
    auto fins = std::make_unique<model::part::FinSet>("fins", 3, cr, ct, s, sweep, thk, R, 600.0);

    const double mN = nose->getMass(0.0), mB = body->getMass(0.0), mF = fins->getMass(0.0);

    // Place the body using the cone's reported CM offset. In the +z = forward frame the cone CM is at
    // getCenterMassOffset().z() = -Lnose/4 from the middle, so the base (aft, at -Lnose/2) is
    // Lnose/2 - (-Lnose/4) = 3 Lnose/4 below the CM; the body CM is another Lbody/2 aft. This same
    // quantity is also the magnitude of cmLocalZ_root (= -coneBaseToCm), the tip-datum shift below.
    const double coneBaseToCm = Lnose / 2.0 - nose->getCenterMassOffset().z();
    const double zBody = coneBaseToCm + Lbody / 2.0;
    // Fins mount at the body's aft end: root LE at body CM + (Lbody/2 - cr), set CM another xc aft.
    const double zFinsFromBody = (Lbody / 2.0 - cr) + finAxialMassCentroid(cr, ct, sweep);

    const auto finsLink = model::part::test::cmToCm(*body, *fins, zFinsFromBody);
    const auto bodyLink = model::part::test::cmToCm(*nose, *body, zBody);
    auto bodyNode = model::PartNode::make(std::move(body));
    bodyNode->addChild(model::PartNode::make(std::move(fins)), finsLink);
    auto root = model::PartNode::make(std::move(nose));
    root->addChild(std::move(bodyNode), bodyLink);
    model::PartsModel pm;
    pm.installRoot(std::move(root));

    // Composite mass = sum of the parts.
    EXPECT_NEAR(pm.root()->compositeMass(0.0), mN + mB + mF, 1e-12);

    // Composite CM = independent mass-weighted average of the three on-axis CMs (from the nose CM),
    // re-expressed into the tip datum by the single shift cmLocalZ_root = -coneBaseToCm.
    const double zFinsFromRoot = zBody + zFinsFromBody;
    const double expectedCmZ = (mN * 0.0 + mB * zBody + mF * zFinsFromRoot) / (mN + mB + mF);
    const Vector3 cm = pm.root()->compositeCm(0.0);
    EXPECT_NEAR(cm.x(), 0.0, 1e-12);
    EXPECT_NEAR(cm.y(), 0.0, 1e-12);
    EXPECT_NEAR(cm.z(), expectedCmZ - coneBaseToCm, 1e-12);

    // Reference area = the single widest frontal disc (pi R^2); the fins do NOT inflate it to
    // pi (rb+s)^2 even though their tip extent (getMaxRadius) is rb+s.
    EXPECT_NEAR(pm.root()->maxFrontalReferenceArea(), pi * R * R, 1e-15);
    EXPECT_LT(pm.root()->maxFrontalReferenceArea(), pi * (R + s) * (R + s));
}

TEST(AeroTest, ManualReferenceAreaOverrideWins)
{
    model::RocketModel rocket;
    rocket.installDesign(model::PartNode::make(
        std::make_unique<model::part::HollowSphere>("Body", 0.04, 0.05, 1956.8)));
    // The placeholder body presents no frontal disc, so the geometry-derived area is 0 ...
    EXPECT_DOUBLE_EQ(rocket.deriveReferenceAreaFromGeometry(), 0.0);
    EXPECT_FALSE(rocket.isReferenceAreaOverridden());

    // ... and a manual setReferenceArea wins and is flagged as the override.
    rocket.setReferenceArea(0.005);
    EXPECT_TRUE(rocket.isReferenceAreaOverridden());
    EXPECT_DOUBLE_EQ(rocket.getReferenceArea(), 0.005);
}

TEST(AeroTest, MotorIsMassButNotAeroOrReferenceContributor)
{
    // A real Motor part joins the airframe tree through the attach verb (which owns the motor slot).
    // Inheriting the inert base defaults, it must add mass (and shift the CM aft) but must not inflate
    // the reference disc or the aero profile.
    const double R = 0.019;
    const double refArea = pi * R * R;
    auto nose = std::make_unique<model::part::ConicalNoseCone>("nose", R, 0.10, 0.0, 900.0, true);
    auto body = std::make_unique<model::part::BodyTube>("body", 0.018, R, 0.40, 680.0);
    auto fins = std::make_unique<model::part::FinSet>("fins", 3, 0.10, 0.05, 0.05, 0.04, 0.003, R, 600.0);
    const auto bodyLink = model::part::test::cmToCm(*nose, *body, 0.225);
    const auto finsLink = model::part::test::cmToCm(*nose, *fins, 0.38);
    auto root = model::PartNode::make(std::move(nose));
    root->addChild(model::PartNode::make(std::move(body)), bodyLink);
    root->addChild(model::PartNode::make(std::move(fins)), finsLink);
    model::PartsModel pm;
    pm.installRoot(std::move(root));

    const double massNoMotor = pm.root()->compositeMass(0.0);
    const double refNoMotor  = pm.root()->maxFrontalReferenceArea();
    const model::AeroProfile aeroNoMotor = pm.root()->compositeAero(refArea);

    auto motor = std::make_unique<model::part::Motor>("motor", makeTestMotor(0.060, 0.030, 1.5, 20.0));
    const double motorMass = motor->getMass(0.0);
    EXPECT_GT(motorMass, 0.0);
    const auto motorLink = model::part::test::cmToCm(pm.root()->part(), *motor, 0.50);
    const auto attached = pm.attach(pm.root()->id(), std::move(motor), motorLink);
    ASSERT_TRUE(attached.has_value());

    // Mass grows by exactly the motor's mass; CM shifts aft toward it.
    EXPECT_NEAR(pm.root()->compositeMass(0.0), massNoMotor + motorMass, 1e-12);
    EXPECT_GT(pm.root()->compositeCm(0.0).z(), 0.0);
    // The inert motor changes neither the reference disc nor the aero profile.
    EXPECT_DOUBLE_EQ(pm.root()->maxFrontalReferenceArea(), refNoMotor);
    const model::AeroProfile aeroWithMotor = pm.root()->compositeAero(refArea);
    EXPECT_DOUBLE_EQ(aeroWithMotor.cnAlpha, aeroNoMotor.cnAlpha);
    EXPECT_DOUBLE_EQ(aeroWithMotor.cnAlphaXcp, aeroNoMotor.cnAlphaXcp);
}

TEST(AeroTest, CompositeCpThreadsNestedStations)
{
    // A 2-level tree (nose -> body -> fins): the fins' CP must be threaded by the CUMULATIVE axial
    // station (zBody + zFinsFromBody), not just the immediate parent offset. Guards the pose
    // accumulation through the placement resolve (the flat-tree CpIsCNalphaWeighted test cannot
    // distinguish the two).
    const double R = 0.019;
    const double refArea = pi * R * R;
    auto nose = std::make_unique<model::part::ConicalNoseCone>("nose", R, 0.10, 0.0, 900.0, true);
    auto body = std::make_unique<model::part::BodyTube>("body", 0.018, R, 0.40, 680.0);
    auto fins = std::make_unique<model::part::FinSet>("fins", 3, 0.10, 0.05, 0.05, 0.04, 0.003, R, 600.0);

    const model::AeroComponent na = nose->getAero(refArea);
    const model::AeroComponent fa = fins->getAero(refArea);

    const double zBody = 0.225, zFinsFromBody = 0.15; // fins nested UNDER body
    const auto finsLink = model::part::test::cmToCm(*body, *fins, zFinsFromBody);
    const auto bodyLink = model::part::test::cmToCm(*nose, *body, zBody);
    auto bodyNode = model::PartNode::make(std::move(body));
    bodyNode->addChild(model::PartNode::make(std::move(fins)), finsLink);
    auto root = model::PartNode::make(std::move(nose));
    root->addChild(std::move(bodyNode), bodyLink);
    model::PartsModel pm;
    pm.installRoot(std::move(root));

    const model::AeroProfile prof = pm.root()->compositeAero(refArea);
    const double zFins = zBody + zFinsFromBody; // cumulative station from the root CM
    const double cmLocalZRoot = -3.0 * 0.10 / 4.0; // solid nose (L = 0.10) CM station: the tip-datum shift
    const double expectedCnAlpha = na.cnAlpha + fa.cnAlpha; // body cnAlpha == 0
    const double expectedMoment  = na.cnAlphaXcp + (fa.cnAlphaXcp + fa.cnAlpha * zFins);
    EXPECT_NEAR(prof.cnAlpha, expectedCnAlpha, 1e-12);
    EXPECT_NEAR(prof.cp(), expectedMoment / expectedCnAlpha + cmLocalZRoot, 1e-12);

    // If the walk wrongly used only the immediate parent offset, the CP would differ measurably.
    const double wrongCp = (na.cnAlphaXcp + fa.cnAlphaXcp + fa.cnAlpha * zFinsFromBody) / expectedCnAlpha;
    EXPECT_GT(std::abs(prof.cp() - wrongCp), 1e-6);
}
