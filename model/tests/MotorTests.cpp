/// \cond
// C++ headers
#include <cmath>
#include <memory>
#include <utility>
#include <vector>
// 3rd party headers
#include <gtest/gtest.h>
/// \endcond

// qtrocket headers
#include "model/parts/Parts.h"   // pulls Motor.h + HollowSphere.h
#include "model/tests/PlacementTestSupport.h"
#include "model/MotorModel.h"
#include "model/ThrustCurve.h"

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
// Part's walk (this is the parallel-axis math by hand), so it is a genuine oracle for getCompositeI.
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
// composite recomputes (each getCompositeI(t) calls getMass once for the gate; a rebuild calls it
// once more inside computeCompositeAt, so the counter jumps by 2 on a rebuild and 1 on a cache hit).
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

   mutable int calls{0};

protected:
   CountingRampPart(const CountingRampPart&) = default; // uses Part's protected copy ctor (fresh id)
   std::shared_ptr<Part> cloneShallow() const override
   { return std::shared_ptr<Part>(new CountingRampPart(*this)); }

private:
   double startMass, endMass, burnTime;
};

// Build an ignited body+motor assembly (motor aft at -0.2 m by default). Returns the body root and
// writes the borrowed motor pointer (the owning shared_ptr lives in the body's child list).
std::shared_ptr<HollowSphere> makeAssembly(Motor*& motorOut, double offsetZ = -0.2)
{
   auto body = std::make_shared<HollowSphere>("body", 0.04, 0.05, 2700.0);
   auto motor = std::make_shared<Motor>("motor", makeTestMotor(0.100, 0.060, 2.0, 80.0));
   motorOut = motor.get();
   motorOut->getMotorModel().startMotor(0.0);
   body->addChildPart(motor, model::part::test::cmToCm(*body, *motor, offsetZ));
   return body;
}
} // namespace

// ---- Mass / CG ----------------------------------------------------------------------------------

TEST(MotorTest, GetMassFollowsMotorModelBurn)
{
   Motor motor("motor", makeTestMotor(0.100, 0.060, 2.0, 80.0));
   EXPECT_NEAR(motor.getMass(0.0), 0.100, 1e-12); // pre-ignition: loaded total weight
   motor.getMotorModel().startMotor(0.0);
   EXPECT_GT(motor.getMass(1.0), 0.040);
   EXPECT_LT(motor.getMass(1.0), 0.100);
   EXPECT_NEAR(motor.getMass(2.0), 0.040, 1e-9); // burnout: empty casing mass
   EXPECT_NEAR(motor.getMass(5.0), 0.040, 1e-9); // stays empty
}

TEST(MotorTest, CompositeMassEqualsAirframePlusMotorAtTime)
{
   Motor* motor = nullptr;
   auto body = makeAssembly(motor);
   const double bodyMass = body->getMass(0.0); // body's own (structural) mass
   for(double t : {0.0, 1.0, 2.0})
   {
      EXPECT_NEAR(body->getCompositeMass(t), bodyMass + motor->getMass(t), 1e-12);
   }
}

TEST(MotorTest, CompositeCmShiftsForwardAsMotorBurns)
{
   Motor* motor = nullptr;
   auto body = makeAssembly(motor, -0.2); // motor aft
   const double cg0 = body->getCompositeCm(0.0)(2);
   const double cg2 = body->getCompositeCm(2.0)(2);
   EXPECT_LT(cg0, 0.0);   // loaded motor pulls the CG aft
   EXPECT_GT(cg2, cg0);   // CG moves forward (toward the body) as propellant burns
}

TEST(MotorTest, CloneIsDeepIndependentAndTypePreserving)
{
   auto motor = std::make_shared<Motor>("motor", makeTestMotor(0.100, 0.060, 2.0, 80.0));
   motor->getMotorModel().startMotor(0.0);
   auto copy = motor->clone();

   EXPECT_NE(dynamic_cast<Motor*>(copy.get()), nullptr); // type preserved (not sliced)
   EXPECT_NE(copy->getId(), motor->getId());             // fresh id

   const double copyMassBefore = copy->getMass(1.0);
   motor->setMotorModel(makeTestMotor(0.200, 0.120, 2.0, 80.0)); // mutate the original
   EXPECT_NEAR(copy->getMass(1.0), copyMassBefore, 1e-12);       // clone untouched
}

TEST(MotorTest, ReplaceMotorChangesCompositeMass)
{
   Motor* motor = nullptr;
   auto body = makeAssembly(motor);
   const double massBefore = body->getCompositeMass(0.0);
   const double izzBefore  = body->getCompositeI(0.0)(2, 2);

   motor->setMotorModel(makeTestMotor(0.200, 0.120, 2.0, 80.0)); // heavier
   EXPECT_GT(body->getCompositeMass(0.0), massBefore);
   EXPECT_NE(body->getCompositeI(0.0)(2, 2), izzBefore); // structural cache invalidated
}

// ---- Composite inertia tensor (the new time-varying work) ---------------------------------------

TEST(MotorInertiaTest, MatchesClosedFormDuringBurn)
{
   auto body = std::make_shared<HollowSphere>("body", 0.04, 0.05, 2700.0);
   const double  mB    = body->getMass(0.0);
   const Matrix3 IBpum = body->getI();
   auto motor = std::make_shared<Motor>("motor", makeTestMotor(0.100, 0.060, 2.0, 80.0));
   Motor* motorRaw = motor.get();
   const Matrix3 IMpum = motorRaw->getI();
   motorRaw->getMotorModel().startMotor(0.0);
   const Vector3 offset{0.0, 0.0, -0.2};
   body->addChildPart(motor, model::part::test::cmToCm(*body, *motor, offset.z()));

   for(double t : {0.0, 1.0, 2.0})
   {
      const double  mM       = motorRaw->getMass(t);
      const Matrix3 expected = expectedComposite(mB, IBpum, mM, IMpum, offset);
      const Matrix3 actual   = body->getCompositeI(t);
      EXPECT_TRUE(actual.isApprox(expected, 1e-9))
         << "t=" << t << "\nexpected:\n" << expected << "\nactual:\n" << actual;
      EXPECT_NEAR(actual(0, 1), 0.0, 1e-12); // purely axial geometry -> no off-diagonals
      EXPECT_NEAR(actual(0, 2), 0.0, 1e-12);
      EXPECT_NEAR(actual(1, 2), 0.0, 1e-12);
   }

   // Anchor against the independently hand-computed values (t=0, loaded).
   EXPECT_NEAR(body->getCompositeI(0.0)(0, 0), 4.488506e-3, 1e-7);
   EXPECT_NEAR(body->getCompositeI(0.0)(2, 2), 9.576700e-4, 1e-7);
}

TEST(MotorInertiaTest, CgConsistentWithTensorWalk)
{
   auto body = std::make_shared<HollowSphere>("body", 0.04, 0.05, 2700.0);
   const double mB = body->getMass(0.0);
   Motor* motor = nullptr;
   auto root = makeAssembly(motor, -0.2);
   (void)body;

   const double t = 1.0;
   const Vector3 cg = root->getCompositeCm(t);
   const double mM = motor->getMass(t);
   // Composite CG is reported in the HollowSphere root's fore-plane (tip) datum, i.e. shifted from the
   // root-own-CM datum by cmLocalZ_root = -outerRadius = -0.05 (the sphere CM sits a radius aft of its
   // +z pole). The motor's -0.2 CM-to-CM offset is preserved by the shim.
   const Vector3 expectedCg =
      (mM * Vector3{0.0, 0.0, -0.2}) / (mB + mM) + Vector3{0.0, 0.0, -0.05};
   EXPECT_TRUE(cg.isApprox(expectedCg, 1e-9)) << "cg=" << cg.transpose();
}

TEST(MotorInertiaTest, IzzDecreasesAndHasNoParallelAxisTerm)
{
   auto body = std::make_shared<HollowSphere>("body", 0.04, 0.05, 2700.0);
   const double mB   = body->getMass(0.0);
   const double IBzz = body->getI()(2, 2);
   auto motor = std::make_shared<Motor>("motor", makeTestMotor(0.100, 0.060, 2.0, 80.0));
   Motor* motorRaw = motor.get();
   const double IMzz = motorRaw->getI()(2, 2);
   motorRaw->getMotorModel().startMotor(0.0);
   body->addChildPart(motor, model::part::test::cmToCm(*body, *motor, -0.2));

   // Izz gets NO parallel-axis contribution (purely axial offset) but is NOT constant: it equals
   // mB*IBzz + mM(t)*IMzz and shrinks as the motor's own longitudinal term shrinks with mass.
   for(double t : {0.0, 1.0, 2.0})
   {
      EXPECT_NEAR(body->getCompositeI(t)(2, 2), mB * IBzz + motorRaw->getMass(t) * IMzz, 1e-12);
   }
   EXPECT_GT(body->getCompositeI(0.0)(2, 2), body->getCompositeI(1.0)(2, 2));
   EXPECT_GT(body->getCompositeI(1.0)(2, 2), body->getCompositeI(2.0)(2, 2));
}

TEST(MotorInertiaTest, FrozenAtEmptyMassAfterBurnout)
{
   auto body = std::make_shared<HollowSphere>("body", 0.04, 0.05, 2700.0);
   const double  mB    = body->getMass(0.0);
   const Matrix3 IBpum = body->getI();
   auto motor = std::make_shared<Motor>("motor", makeTestMotor(0.100, 0.060, 2.0, 80.0));
   Motor* motorRaw = motor.get();
   const Matrix3 IMpum = motorRaw->getI();
   motorRaw->getMotorModel().startMotor(0.0);
   const Vector3 offset{0.0, 0.0, -0.2};
   body->addChildPart(motor, model::part::test::cmToCm(*body, *motor, offset.z()));

   const Matrix3 atBurnout = body->getCompositeI(2.0);
   // Bitwise-identical for every t >= burnout, including out-of-order probes.
   for(double t : {2.0, 2.0 + 1e-6, 12.0, 2.5})
   {
      EXPECT_EQ((body->getCompositeI(t) - atBurnout).norm(), 0.0) << "t=" << t;
   }
   // ...and it equals the true empty-mass tensor (mM = 0.040), not a slightly-pre-burnout value.
   const Matrix3 expectedEmpty = expectedComposite(mB, IBpum, 0.040, IMpum, offset);
   EXPECT_TRUE(atBurnout.isApprox(expectedEmpty, 1e-9));
}

TEST(MotorInertiaTest, NoRecomputeAfterBurnout)
{
   // A single mass-varying leaf so getMass call counting is unambiguous: each getCompositeI(t) query
   // calls getMass once for the gate (getCompositeMass) and, only on a rebuild, once more inside
   // computeCompositeAt. So a rebuild costs 2 calls, a cache hit costs 1.
   auto part = std::make_shared<CountingRampPart>(0.100, 0.040, 2.0);
   auto delta = [&](double t) { int before = part->calls; part->getCompositeI(t); return part->calls - before; };

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
   Motor* motor = nullptr;
   auto body = makeAssembly(motor);
   const Matrix3 frozen = body->getCompositeI(5.0); // settle into the post-burnout frozen cache

   motor->setMotorModel(makeTestMotor(0.200, 0.120, 2.0, 80.0)); // structural edit (setMass/setI)
   const Matrix3 after = body->getCompositeI(5.0);
   EXPECT_GT((after - frozen).cwiseAbs().maxCoeff(), 1e-9); // dirty flag forces a rebuild
}
