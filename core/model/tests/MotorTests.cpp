/// \cond
// C++ headers
#include <memory>
#include <utility>
#include <vector>
// 3rd party headers
#include <gtest/gtest.h>
/// \endcond

// qtrocket headers
#include "model/PartsModel.h"
#include "model/parts/Parts.h"   // pulls Motor.h + HollowSphere.h
#include "model/tests/PlacementTestSupport.h"
#include "model/MotorModel.h"
#include "model/ThrustCurve.h"

using model::PartNode;
using model::PartsModel;
using model::part::HollowSphere;
using model::part::Motor;
using model::part::Part;

namespace
{
// Synthetic single-use motor with FLAT thrust over [0, burnTime] (so propellant -- and therefore
// mass -- depletes linearly). emptyMass = totalWeight - propWeight; all masses in kg. addThrustCurve
// must precede setMetaData because computeMassCurve() reads the curve.
model::MotorModel makeTestMotor(double totalWeight, double propWeight,
                                           double burnTime, double totalImpulse)
{
    const double flatThrust = totalImpulse / burnTime;
    std::vector<std::pair<double, double>> samples{ {0.0, flatThrust}, {burnTime, flatThrust} };
    ThrustCurve tc(samples);

    model::MotorModel m;
    m.addThrustCurve(tc);                 // must precede setMetaData

    model::MotorModel::MetaData md;
    md.totalWeight = totalWeight; md.propWeight = propWeight;
    md.burnTime = burnTime;       md.totalImpulse = totalImpulse;
    md.diameter = 24.0; md.length = 70.0; // mm, for the solid-cylinder geometric tensor
    m.setMetaData(md);                    // triggers computeMassCurve()
    return m;
}

// Independent closed-form composite inertia oracle for [body at origin] + [a single leaf at offset],
// both with diagonal per-unit-mass tensors and CMs on the z-axis. Computed entirely separately from
// the node walk (this is the parallel-axis math by hand), so it is a genuine oracle for compositeI.
Matrix3 expectedComposite(double mB, const Matrix3& IBpum,
                                   double mM, const Matrix3& IMpum, const Vector3& offset)
{
    const double M = mB + mM;
    const Vector3 cg = (mM * offset) / M;          // body CM at the origin
    auto pa = [](const Vector3& d) { return d.dot(d) * Matrix3::Identity() - d * d.transpose(); };
    return mB * IBpum + mB * pa(-cg) + mM * IMpum + mM * pa(offset - cg);
}

// A leaf whose OWN mass ramps linearly from startMass to endMass over [0, burnTime] then holds
// endMass -- a stand-in for a burning motor -- and counts getMass() calls so a test can detect
// composite recomputes (each compositeI(t) query calls getMass once for the mass-delta gate; a
// rebuild calls it once more inside the composite walk, so the counter jumps by 2 on a rebuild and
// 1 on a cache hit).
class CountingRampPart : public Part
{
public:
    CountingRampPart(double start, double end, double burn)
        : Part("ramp", Matrix3::Identity(), start, Vector3::Zero()),
           startMass(start), endMass(end), burnTime(burn) {}

    double getMass(double t) const override
    {
        ++calls;
        if(t <= 0.0)      return startMass;
        if(t >= burnTime) return endMass;
        return startMass + (endMass - startMass) * (t / burnTime);
    }
    std::string typeName() const override { return "CountingRampPart"; } // Part is abstract; concrete stub

    std::unique_ptr<Part> clone() const override
    { return std::unique_ptr<Part>(new CountingRampPart(*this)); }

    mutable int calls{0};

protected:
    CountingRampPart(const CountingRampPart&) = default; // uses Part's protected copy ctor (fresh id)

private:
    double startMass, endMass, burnTime;
};

// Seed @p pm with a hollow-sphere body root plus an ignited test motor whose CM sits offsetZ along
// +z of the body CM (aft when negative). Returns the motor's node, owned by pm.
const PartNode* makeAssembly(PartsModel& pm, double offsetZ = -0.2)
{
    auto body  = std::make_unique<HollowSphere>("body", 0.04, 0.05, 2700.0);
    auto motor = std::make_unique<Motor>("motor", makeTestMotor(0.100, 0.060, 2.0, 80.0));
    const model::part::StationLink link = model::part::test::cmToCm(*body, *motor, offsetZ);
    pm.installRoot(PartNode::make(std::move(body)));
    const auto id = pm.attach(pm.root()->id(), std::move(motor), link);
    pm.startMotor(0.0);
    return id ? pm.find(*id) : nullptr;
}
} // namespace

// ---- Mass / CG ----------------------------------------------------------------------------------

TEST(MotorTest, GetMassFollowsMotorModelBurn)
{
    auto mm = makeTestMotor(0.100, 0.060, 2.0, 80.0);
    const Motor unlit("motor", mm);
    EXPECT_NEAR(unlit.getMass(0.0), 0.100, 1e-12); // pre-ignition: loaded total weight

    mm.startMotor(0.0);
    const Motor lit("motor", mm); // wraps a copy, ignition epoch included
    EXPECT_GT(lit.getMass(1.0), 0.040);
    EXPECT_LT(lit.getMass(1.0), 0.100);
    EXPECT_NEAR(lit.getMass(2.0), 0.040, 1e-9); // burnout: empty casing mass
    EXPECT_NEAR(lit.getMass(5.0), 0.040, 1e-9); // stays empty
}

TEST(MotorTest, CompositeMassEqualsAirframePlusMotorAtTime)
{
    PartsModel pm;
    const PartNode* motorNode = makeAssembly(pm);
    ASSERT_NE(motorNode, nullptr);
    const double bodyMass = pm.root()->part().getMass(0.0); // body's own (structural) mass
    for(double t : {0.0, 1.0, 2.0})
    {
        EXPECT_NEAR(pm.root()->compositeMass(t), bodyMass + motorNode->part().getMass(t), 1e-12);
    }
}

TEST(MotorTest, CompositeCmShiftsForwardAsMotorBurns)
{
    PartsModel pm;
    makeAssembly(pm, -0.2); // motor aft
    const double cg0 = pm.root()->compositeCm(0.0)(2);
    const double cg2 = pm.root()->compositeCm(2.0)(2);
    // fore-plane datum: the sphere's own CM sits one radius aft of the +z pole, at -0.05
    EXPECT_LT(cg0, -0.05); // loaded motor pulls the CG aft of the body's own CM
    EXPECT_GT(cg2, cg0);   // CG moves forward (toward the body) as propellant burns
}

TEST(MotorTest, CloneIsDeepIndependentAndTypePreserving)
{
    auto mm = makeTestMotor(0.100, 0.060, 2.0, 80.0);
    mm.startMotor(0.0);
    std::unique_ptr<Part> motor = std::make_unique<Motor>("motor", mm); // clone() is public on Part
    const Part::Id originalId = motor->getId();
    auto copy = motor->clone();

    EXPECT_NE(dynamic_cast<Motor*>(copy.get()), nullptr); // type preserved (not sliced)
    EXPECT_NE(copy->getId(), originalId);                 // fresh id

    const double copyMassBefore = copy->getMass(1.0);
    // mutate the original through the routed swap (the only post-attach mutation path)
    PartsModel pm;
    pm.installRoot(PartNode::make(std::make_unique<HollowSphere>("body", 0.04, 0.05, 2700.0)));
    const Part* original = motor.get();
    ASSERT_TRUE(pm.attach(pm.root()->id(), std::move(motor)).has_value());
    ASSERT_TRUE(pm.setMotor(makeTestMotor(0.200, 0.120, 2.0, 80.0)));
    EXPECT_NEAR(original->getMass(0.0), 0.200, 1e-12);      // swap landed on the original
    EXPECT_NEAR(copy->getMass(1.0), copyMassBefore, 1e-12); // clone untouched
}

TEST(MotorTest, ReplaceMotorChangesCompositeMass)
{
    PartsModel pm;
    makeAssembly(pm);
    const double massBefore = pm.root()->compositeMass(0.0);
    const double izzBefore  = pm.root()->compositeI(0.0)(2, 2);

    ASSERT_TRUE(pm.setMotor(makeTestMotor(0.200, 0.120, 2.0, 80.0))); // heavier, in-place swap
    EXPECT_GT(pm.root()->compositeMass(0.0), massBefore);
    EXPECT_NE(pm.root()->compositeI(0.0)(2, 2), izzBefore); // swap dirties the composite cache
}

TEST(MotorTest, DetachMotorRestoresBodyOnlyComposite)
{
    PartsModel pm;
    const PartNode* motorNode = makeAssembly(pm);
    ASSERT_NE(motorNode, nullptr);
    const double bodyMass = pm.root()->part().getMass(0.0);
    pm.root()->compositeI(0.0); // warm the cache so detach must invalidate it

    auto detached = pm.detach(motorNode->id());
    ASSERT_TRUE(detached.has_value());
    EXPECT_FALSE(pm.isMotorSet()); // the motor borrow is released with its sub-tree
    EXPECT_NEAR(pm.root()->compositeMass(0.0), bodyMass, 1e-12);
    const Vector3 cg = pm.root()->compositeCm(0.0);
    EXPECT_NEAR(cg(0), 0.0, 1e-12);
    EXPECT_NEAR(cg(1), 0.0, 1e-12);
    EXPECT_NEAR(cg(2), -0.05, 1e-12); // back at the body's own CM (fore-plane datum)
}

// ---- Composite inertia tensor (the time-varying work) -------------------------------------------

TEST(MotorInertiaTest, MatchesClosedFormDuringBurn)
{
    PartsModel pm;
    const PartNode* motorNode = makeAssembly(pm, -0.2);
    ASSERT_NE(motorNode, nullptr);
    const double  mB    = pm.root()->part().getMass(0.0);
    const Matrix3 IBpum = pm.root()->part().getI();
    const Matrix3 IMpum = motorNode->part().getI();
    const Vector3 offset{0.0, 0.0, -0.2};

    for(double t : {0.0, 1.0, 2.0})
    {
        const double  mM       = motorNode->part().getMass(t);
        const Matrix3 expected = expectedComposite(mB, IBpum, mM, IMpum, offset);
        const Matrix3 actual   = pm.root()->compositeI(t);
        EXPECT_TRUE(actual.isApprox(expected, 1e-9))
            << "t=" << t << "\nexpected:\n" << expected << "\nactual:\n" << actual;
        EXPECT_NEAR(actual(0, 1), 0.0, 1e-12); // purely axial geometry -> no off-diagonals
        EXPECT_NEAR(actual(0, 2), 0.0, 1e-12);
        EXPECT_NEAR(actual(1, 2), 0.0, 1e-12);
    }

    // Anchor against the independently hand-computed values (t=0, loaded).
    EXPECT_NEAR(pm.root()->compositeI(0.0)(0, 0), 4.488506e-3, 1e-7);
    EXPECT_NEAR(pm.root()->compositeI(0.0)(2, 2), 9.576700e-4, 1e-7);
}

TEST(MotorInertiaTest, CgConsistentWithTensorWalk)
{
    PartsModel pm;
    const PartNode* motorNode = makeAssembly(pm, -0.2);
    ASSERT_NE(motorNode, nullptr);
    const double mB = pm.root()->part().getMass(0.0);

    const double t = 1.0;
    const Vector3 cg = pm.root()->compositeCm(t);
    const double mM = motorNode->part().getMass(t);
    // Composite CG is reported on the root's fore-plane (tip) datum: the sphere's own CM sits a
    // radius aft of its +z pole (-0.05), and the link preserves the -0.2 CM-to-CM gap.
    const Vector3 expectedCg =
        (mM * Vector3{0.0, 0.0, -0.2}) / (mB + mM) + Vector3{0.0, 0.0, -0.05};
    EXPECT_TRUE(cg.isApprox(expectedCg, 1e-9)) << "cg=" << cg.transpose();
}

TEST(MotorInertiaTest, IzzDecreasesAndHasNoParallelAxisTerm)
{
    PartsModel pm;
    const PartNode* motorNode = makeAssembly(pm, -0.2);
    ASSERT_NE(motorNode, nullptr);
    const double mB   = pm.root()->part().getMass(0.0);
    const double IBzz = pm.root()->part().getI()(2, 2);
    const double IMzz = motorNode->part().getI()(2, 2);

    // Izz gets NO parallel-axis contribution (purely axial offset) but is NOT constant: it equals
    // mB*IBzz + mM(t)*IMzz and shrinks as the motor's own longitudinal term shrinks with mass.
    for(double t : {0.0, 1.0, 2.0})
    {
        EXPECT_NEAR(pm.root()->compositeI(t)(2, 2),
                        mB * IBzz + motorNode->part().getMass(t) * IMzz, 1e-12);
    }
    EXPECT_GT(pm.root()->compositeI(0.0)(2, 2), pm.root()->compositeI(1.0)(2, 2));
    EXPECT_GT(pm.root()->compositeI(1.0)(2, 2), pm.root()->compositeI(2.0)(2, 2));
}

TEST(MotorInertiaTest, FrozenAtEmptyMassAfterBurnout)
{
    PartsModel pm;
    const PartNode* motorNode = makeAssembly(pm, -0.2);
    ASSERT_NE(motorNode, nullptr);
    const double  mB    = pm.root()->part().getMass(0.0);
    const Matrix3 IBpum = pm.root()->part().getI();
    const Matrix3 IMpum = motorNode->part().getI();
    const Vector3 offset{0.0, 0.0, -0.2};

    const Matrix3 atBurnout = pm.root()->compositeI(2.0);
    // Bitwise-identical for every t >= burnout, including out-of-order probes.
    for(double t : {2.0, 2.0 + 1e-6, 12.0, 2.5})
    {
        EXPECT_EQ((pm.root()->compositeI(t) - atBurnout).norm(), 0.0) << "t=" << t;
    }
    // ...and it equals the true empty-mass tensor (mM = 0.040), not a slightly-pre-burnout value.
    const Matrix3 expectedEmpty = expectedComposite(mB, IBpum, 0.040, IMpum, offset);
    EXPECT_TRUE(atBurnout.isApprox(expectedEmpty, 1e-9));
}

TEST(MotorInertiaTest, NoRecomputeAfterBurnout)
{
    // A single mass-varying leaf so getMass call counting is unambiguous: each compositeI(t) query
    // calls getMass once for the gate (compositeMass) and, only on a rebuild, once more inside the
    // composite walk. So a rebuild costs 2 calls, a cache hit costs 1.
    auto part = std::make_unique<CountingRampPart>(0.100, 0.040, 2.0);
    CountingRampPart* raw = part.get();
    PartsModel pm;
    pm.installRoot(PartNode::make(std::move(part)));
    auto delta = [&](double t)
    { const int before = raw->calls; pm.root()->compositeI(t); return raw->calls - before; };

    delta(0.5);                 // first query always builds (NaN sentinel)
    EXPECT_EQ(delta(1.0), 2);   // mass changed during burn -> rebuild
    EXPECT_EQ(delta(1.5), 2);   // rebuild
    EXPECT_EQ(delta(3.0), 2);   // crossing into post-burnout: final transition rebuild (empty mass)
    EXPECT_EQ(delta(3.5), 1);   // frozen: mass constant -> NO rebuild, gate sum only
    EXPECT_EQ(delta(4.0), 1);   // still frozen
    EXPECT_EQ(delta(3.2), 1);   // out-of-order post-burnout probe: still frozen
    EXPECT_EQ(delta(9.0), 1);
}

TEST(MotorInertiaTest, StructuralChangeInvalidatesAfterBurnout)
{
    PartsModel pm;
    makeAssembly(pm);
    const Matrix3 frozen = pm.root()->compositeI(5.0); // settle into the post-burnout frozen cache

    ASSERT_TRUE(pm.setMotor(makeTestMotor(0.200, 0.120, 2.0, 80.0))); // in-place swap
    const Matrix3 after = pm.root()->compositeI(5.0);
    EXPECT_GT((after - frozen).cwiseAbs().maxCoeff(), 1e-9); // swap dirties the chain -> rebuild
}
