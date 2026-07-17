#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <stdexcept>

#include "model/PartsModel.h"
#include "model/parts/PartFactory.h"
#include "model/parts/Parts.h"
#include "model/tests/TestPart.h"

namespace
{
using model::part::PartParams;
using model::part::makePart;
using model::part::params;

// Assert two parts have identical own-mass and full composite CM/inertia at t=0 -- the strongest
// black-box check that a factory-built part equals a directly-constructed one (and that a reflected
// round-trip preserves the geometry the composite math consumes). Composite queries live on the
// node, so each part is cloned into its own single-node tree.
void expectSameMassAndComposite(const model::part::Part& a, const model::part::Part& b)
{
    EXPECT_DOUBLE_EQ(a.getMass(0.0), b.getMass(0.0));
    model::PartsModel ma;
    ma.installRoot(model::PartNode::make(a.clone()));
    model::PartsModel mb;
    mb.installRoot(model::PartNode::make(b.clone()));
    const Vector3 ca = ma.root()->compositeCm(0.0);
    const Vector3 cb = mb.root()->compositeCm(0.0);
    const Matrix3 ia = ma.root()->compositeI(0.0);
    const Matrix3 ib = mb.root()->compositeI(0.0);
    for(int i = 0; i < 3; ++i)
    {
        EXPECT_DOUBLE_EQ(ca(i), cb(i));
    }
    for(int r = 0; r < 3; ++r)
    {
        for(int c = 0; c < 3; ++c)
        {
            EXPECT_DOUBLE_EQ(ia(r, c), ib(r, c));
        }
    }
}

// A fully-specified PartParams for each geometry type (no reliance on makePart defaults).
PartParams bodyTubeParams()
{
    PartParams p;
    p.name = "Body";
    p.innerRadius = 0.0;
    p.outerRadius = 0.019;
    p.length = 0.20;
    p.density = 680.0;
    return p;
}
PartParams noseConeParams(bool solid)
{
    PartParams p;
    p.name = "Nose";
    p.baseRadius = 0.019;
    p.length = 0.10;
    p.wallThickness = solid ? 0.0 : 0.001;
    p.density = 2700.0;
    p.solid = solid;
    return p;
}
PartParams finSetParams()
{
    PartParams p;
    p.name = "Fins";
    p.finCount = 3u;
    p.rootChord = 0.10;
    p.tipChord = 0.05;
    p.span = 0.05;
    p.sweep = 0.04;
    p.thickness = 0.003;
    p.bodyRadius = 0.019;
    p.density = 600.0;
    return p;
}
PartParams hollowSphereParams()
{
    PartParams p;
    p.name = "Ball";
    p.innerRadius = 0.04;
    p.outerRadius = 0.05;
    p.density = 2700.0;
    return p;
}
} // namespace

TEST(PartFactoryTest, MakePartBuildsEachTypeMatchingDirectConstruction)
{
    {
        auto made = makePart("BodyTube", bodyTubeParams());
        auto* bt = dynamic_cast<model::part::BodyTube*>(made.get());
        ASSERT_NE(bt, nullptr);
        EXPECT_EQ(bt->typeName(), "BodyTube");
        EXPECT_DOUBLE_EQ(bt->getInnerRadius(), 0.0);
        EXPECT_DOUBLE_EQ(bt->getOuterRadius(), 0.019);
        EXPECT_DOUBLE_EQ(bt->getLength(), 0.20);
        EXPECT_DOUBLE_EQ(bt->getDensity(), 680.0);
        model::part::BodyTube direct("Body", 0.0, 0.019, 0.20, 680.0);
        expectSameMassAndComposite(*made, direct);
    }
    {
        auto made = makePart("NoseCone", noseConeParams(true));
        auto* nc = dynamic_cast<model::part::ConicalNoseCone*>(made.get());
        ASSERT_NE(nc, nullptr);
        EXPECT_EQ(nc->typeName(), "NoseCone");
        EXPECT_TRUE(nc->isSolid());
        model::part::ConicalNoseCone direct("Nose", 0.019, 0.10, 0.0, 2700.0, true);
        expectSameMassAndComposite(*made, direct);
    }
    {
        // The thin-shell path: the solid flag must reach the ctor and change the mass/inertia.
        auto made = makePart("NoseCone", noseConeParams(false));
        auto* nc = dynamic_cast<model::part::ConicalNoseCone*>(made.get());
        ASSERT_NE(nc, nullptr);
        EXPECT_FALSE(nc->isSolid());
        EXPECT_DOUBLE_EQ(nc->getWallThickness(), 0.001);
        model::part::ConicalNoseCone direct("Nose", 0.019, 0.10, 0.001, 2700.0, false);
        expectSameMassAndComposite(*made, direct);
    }
    {
        auto made = makePart("FinSet", finSetParams());
        auto* fs = dynamic_cast<model::part::FinSet*>(made.get());
        ASSERT_NE(fs, nullptr);
        EXPECT_EQ(fs->typeName(), "FinSet");
        EXPECT_EQ(fs->getFinCount(), 3u);
        model::part::FinSet direct("Fins", 3, 0.10, 0.05, 0.05, 0.04, 0.003, 0.019, 600.0);
        expectSameMassAndComposite(*made, direct);
    }
    {
        auto made = makePart("HollowSphere", hollowSphereParams());
        auto* hs = dynamic_cast<model::part::HollowSphere*>(made.get());
        ASSERT_NE(hs, nullptr);
        EXPECT_EQ(hs->typeName(), "HollowSphere");
        model::part::HollowSphere direct("Ball", 0.04, 0.05, 2700.0);
        expectSameMassAndComposite(*made, direct);
    }
}

TEST(PartFactoryTest, ParamsThenMakePartRoundTripsGeometry)
{
    // The drift self-test: a directly-built part -> params() -> makePart() must reproduce the same
    // geometry. If the reflector reads a wrong getter or the factory writes a wrong field, the
    // reconstructed composite diverges and this fails.
    model::part::BodyTube body("Body", 0.0, 0.019, 0.20, 680.0);
    auto bodyRT = makePart(body.typeName(), params(body));
    ASSERT_NE(dynamic_cast<model::part::BodyTube*>(bodyRT.get()), nullptr);
    expectSameMassAndComposite(body, *bodyRT);

    model::part::ConicalNoseCone shell("Nose", 0.019, 0.10, 0.001, 2700.0, false);
    auto shellRT = makePart(shell.typeName(), params(shell));
    auto* shellCone = dynamic_cast<model::part::ConicalNoseCone*>(shellRT.get());
    ASSERT_NE(shellCone, nullptr);
    EXPECT_FALSE(shellCone->isSolid()); // the solid flag survived the reflect/rebuild
    expectSameMassAndComposite(shell, *shellRT);

    model::part::FinSet fins("Fins", 3, 0.10, 0.05, 0.05, 0.04, 0.003, 0.019, 600.0);
    auto finsRT = makePart(fins.typeName(), params(fins));
    ASSERT_NE(dynamic_cast<model::part::FinSet*>(finsRT.get()), nullptr);
    expectSameMassAndComposite(fins, *finsRT);

    model::part::HollowSphere ball("Ball", 0.04, 0.05, 2700.0);
    auto ballRT = makePart(ball.typeName(), params(ball));
    ASSERT_NE(dynamic_cast<model::part::HollowSphere*>(ballRT.get()), nullptr);
    expectSameMassAndComposite(ball, *ballRT);

    // The name round-trips too.
    EXPECT_EQ(bodyRT->getName(), "Body");
}

TEST(PartFactoryTest, MakePartPropagatesConcreteCtorValidation)
{
    PartParams bad = bodyTubeParams();
    bad.innerRadius = 0.02;  // ri > ro -> BodyTube ctor throws
    bad.outerRadius = 0.019;
    EXPECT_THROW(makePart("BodyTube", bad), std::invalid_argument);

    PartParams badCone = noseConeParams(true);
    badCone.baseRadius = -1.0; // R <= 0 -> ConicalNoseCone ctor throws
    EXPECT_THROW(makePart("NoseCone", badCone), std::invalid_argument);

    PartParams badFin = finSetParams();
    badFin.finCount = 0u; // N < 1 -> FinSet ctor throws (N<3 only warns; N==0 throws)
    EXPECT_THROW(makePart("FinSet", badFin), std::invalid_argument);

    PartParams badSphere = hollowSphereParams();
    badSphere.innerRadius = 0.06; // ri > ro -> HollowSphere ctor throws
    badSphere.outerRadius = 0.05;
    EXPECT_THROW(makePart("HollowSphere", badSphere), std::invalid_argument);
}

TEST(PartFactoryTest, MakePartThrowsOnMissingRequiredField)
{
    PartParams p;
    p.name = "Body";
    p.length = 0.20;
    p.density = 680.0; // outerRadius omitted -> required field missing
    EXPECT_THROW(makePart("BodyTube", p), std::invalid_argument);

    PartParams finNoCount = finSetParams();
    finNoCount.finCount = std::nullopt; // FinSet requires finCount
    EXPECT_THROW(makePart("FinSet", finNoCount), std::invalid_argument);

    PartParams coneNoRadius = noseConeParams(true);
    coneNoRadius.baseRadius = std::nullopt; // ConicalNoseCone requires baseRadius
    EXPECT_THROW(makePart("NoseCone", coneNoRadius), std::invalid_argument);
}

TEST(PartFactoryTest, MakePartThrowsOnUnknownOrNonConstructibleType)
{
    EXPECT_THROW(makePart("Frobnicator", bodyTubeParams()), std::invalid_argument);
    // Motor is real but is not factory-built (it needs a MotorModel; installed via PartsModel::setMotor).
    EXPECT_THROW(makePart("Motor", PartParams{}), std::invalid_argument);
}

TEST(PartFactoryTest, ParamsOnMotorOrBasePartReflectsOnlyTheName)
{
    model::part::Motor motor("M1", model::MotorModel{});
    const PartParams mp = params(motor);
    EXPECT_EQ(mp.name, "M1");
    EXPECT_FALSE(mp.density.has_value());   // no geometry fields engaged for a Motor
    EXPECT_FALSE(mp.outerRadius.has_value());
    EXPECT_FALSE(mp.finCount.has_value());

    model::part::TestPart base("B", Matrix3::Zero(), 1.0, Vector3::Zero());
    const PartParams bp = params(base);
    EXPECT_EQ(bp.name, "B");
    EXPECT_FALSE(bp.density.has_value());
}

TEST(PartFactoryTest, MakePartAppliesDocumentedDefaultsForOmittedOptionalFields)
{
    // innerRadius (0), wallThickness (0), sweep (0), and solid (true) are defaulted when omitted, so a
    // minimal PartParams still builds -- this exercises the .value_or branches a fully-specified
    // PartParams never touches.
    PartParams tube;
    tube.name = "Rod";
    tube.outerRadius = 0.019;
    tube.length = 0.20;
    tube.density = 680.0;        // innerRadius omitted -> 0 (solid rod)
    auto madeTube = makePart("BodyTube", tube);
    auto* bt = dynamic_cast<model::part::BodyTube*>(madeTube.get());
    ASSERT_NE(bt, nullptr);
    EXPECT_DOUBLE_EQ(bt->getInnerRadius(), 0.0);

    PartParams cone;
    cone.name = "Nose";
    cone.baseRadius = 0.019;
    cone.length = 0.10;
    cone.density = 2700.0;       // wallThickness + solid omitted -> 0 and true
    auto madeCone = makePart("NoseCone", cone);
    auto* nc = dynamic_cast<model::part::ConicalNoseCone*>(madeCone.get());
    ASSERT_NE(nc, nullptr);
    EXPECT_TRUE(nc->isSolid());
    EXPECT_DOUBLE_EQ(nc->getWallThickness(), 0.0);

    PartParams fins;
    fins.name = "Fins";
    fins.finCount = 3u;
    fins.rootChord = 0.10;
    fins.tipChord = 0.05;
    fins.span = 0.05;
    fins.thickness = 0.003;
    fins.bodyRadius = 0.019;
    fins.density = 600.0;        // sweep omitted -> 0 (un-swept)
    auto madeFins = makePart("FinSet", fins);
    auto* fs = dynamic_cast<model::part::FinSet*>(madeFins.get());
    ASSERT_NE(fs, nullptr);
    EXPECT_DOUBLE_EQ(fs->getSweep(), 0.0);
}
