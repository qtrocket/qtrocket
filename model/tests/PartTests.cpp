#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <tuple>

#include "model/parts/Part.h"
#include "model/InertiaTensors.h"
#include "model/parts/Parts.h"

class PartTest : public testing::Test
{
protected:
  // Per-test-suite set-up.
  // Called before the first test in this test suite.
  // Can be omitted if not needed.
  static void SetUpTestSuite()
  {
    //shared_resource_ = new ...;

    // If `shared_resource_` is **not deleted** in `TearDownTestSuite()`,
    // reallocation should be prevented because `SetUpTestSuite()` may be called
    // in subclasses of FooTest and lead to memory leak.
    //
    // if (shared_resource_ == nullptr) {
    //   shared_resource_ = new ...;
    // }
  }

  // Per-test-suite tear-down.
  // Called after the last test in this test suite.
  // Can be omitted if not needed.
  static void TearDownTestSuite()
  {
    //delete shared_resource_;
    //shared_resource_ = nullptr;
  }

  // You can define per-test set-up logic as usual.
  void SetUp() override { }

  // You can define per-test tear-down logic as usual.
  void TearDown() override { }

  // Some expensive resource shared by all tests.
  //static T* shared_resource_;
};

//T* FooTest::shared_resource_ = nullptr;

TEST(PartTest, CreationTests)
{
   Matrix3 inertia;
   inertia << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;
   Vector3 cm{1, 0, 0};
   model::part::Part testPart("testPart",
                        inertia,
                        1.0,
                        cm);
   
   Matrix3 inertia2;
   inertia2 << 1, 0, 0,
               0, 1, 0,
               0, 0, 1;
   Vector3 cm2{1, 0, 0};
   Vector3 R{2.0, 2.0, 2.0};
   testPart.addChildPart(std::make_shared<model::part::Part>("testPart2", inertia2, 1.0, cm2), R);


}

namespace
{
// Independent closed-form references for a uniform thick-walled hollow sphere.
double expectedHollowSphereMass(double ri, double ro, double density)
{
   const double volume = (4.0 / 3.0) * std::numbers::pi
                         * (std::pow(ro, 3) - std::pow(ri, 3));
   return density * volume;
}

double expectedHollowSphereInertiaDiagonal(double ri, double ro, double density)
{
   const double mass = expectedHollowSphereMass(ri, ro, density);
   return mass * (2.0 / 5.0) * (std::pow(ro, 5) - std::pow(ri, 5))
                             / (std::pow(ro, 3) - std::pow(ri, 3));
}
} // namespace

TEST(HollowSphereTest, MassAndCompositeInertiaMatchClosedForm)
{
   const double ri = 0.04;
   const double ro = 0.05;
   const double density = 2700.0;

   model::part::HollowSphere sphere("body", ri, ro, density);

   const double expectedMass = expectedHollowSphereMass(ri, ro, density);
   EXPECT_NEAR(sphere.getMass(0.0), expectedMass, 1e-12);
   EXPECT_NEAR(sphere.getVolume(), expectedMass / density, 1e-15);

   // getCompositeI(0.0) is the full, mass-weighted tensor (kg*m^2).
   const Matrix3 I = sphere.getCompositeI(0.0);
   const double expectedDiag = expectedHollowSphereInertiaDiagonal(ri, ro, density);
   EXPECT_NEAR(I(0, 0), expectedDiag, 1e-12);
   EXPECT_NEAR(I(1, 1), expectedDiag, 1e-12);
   EXPECT_NEAR(I(2, 2), expectedDiag, 1e-12);
   // Isotropic: off-diagonals vanish.
   EXPECT_DOUBLE_EQ(I(0, 1), 0.0);
   EXPECT_DOUBLE_EQ(I(0, 2), 0.0);
   EXPECT_DOUBLE_EQ(I(1, 2), 0.0);

   // getI() is per-unit-mass, so getCompositeI(0.0) == mass * getI().
   EXPECT_NEAR(I(0, 0), expectedMass * sphere.getI()(0, 0), 1e-12);
}

TEST(HollowSphereTest, ReducesToSolidSphereWhenInnerRadiusZero)
{
   const double ro = 0.05;
   const double density = 2700.0;

   model::part::HollowSphere sphere("solid", 0.0, ro, density);

   const double mass = sphere.getMass(0.0);
   // Solid sphere: I = (2/5) m ro^2 on each axis.
   EXPECT_NEAR(sphere.getCompositeI(0.0)(0, 0), mass * (2.0 / 5.0) * ro * ro, 1e-12);
   // ... which is exactly mass * InertiaTensors::SolidSphere(ro).
   EXPECT_NEAR(sphere.getCompositeI(0.0)(0, 0),
               mass * model::InertiaTensors::SolidSphere(ro)(0, 0), 1e-12);
}

TEST(HollowSphereTest, RejectsNonPhysicalGeometry)
{
   EXPECT_THROW(model::part::HollowSphere("bad", 0.05, 0.04, 2700.0), std::invalid_argument); // ri > ro
   EXPECT_THROW(model::part::HollowSphere("bad", 0.04, 0.04, 2700.0), std::invalid_argument); // ri == ro
   EXPECT_THROW(model::part::HollowSphere("bad", 0.00, 0.05, 0.0),    std::invalid_argument); // density 0
}

TEST(PartTest, StoresInertiaPerUnitMassWithMassWeightedComposite)
{
   // The bare tensor is per-unit-mass; the composite is full (mass * per-mass). With mass = 2.0 and
   // SolidSphere(1.0) = 0.4 on the diagonal, getI() = 0.4 but getCompositeI(0.0) = 0.8 -- this would be
   // 0.4 if Part stored the tensor un-weighted, so it locks the mass multiply in.
   model::part::Part part("p", model::InertiaTensors::SolidSphere(1.0), 2.0, Vector3{0.0, 0.0, 0.0});
   EXPECT_DOUBLE_EQ(part.getI()(0, 0), 0.4);
   EXPECT_DOUBLE_EQ(part.getCompositeI(0.0)(0, 0), 0.8);
}

namespace
{
// Build a massless-inertia "point mass": all the inertia comes from the parallel-axis shift, which
// is exactly what the composite math is responsible for getting right. Returns a shared_ptr because
// parts are owned through shared_ptr and addChildPart() takes ownership of one.
std::shared_ptr<model::part::Part> pointMass(const std::string& name, double mass)
{
   return std::make_shared<model::part::Part>(name, Matrix3::Zero(), mass, Vector3::Zero());
}

// Mass of a uniform hollow cylinder (tube): density * volume, volume = pi * (ro^2 - ri^2) * length.
double tubeMass(double ri, double ro, double length, double density)
{
   return density * std::numbers::pi * (ro * ro - ri * ri) * length;
}

// Build a tube Part: longitudinal axis on z (per InertiaTensors::Tube), CM at the part origin. Used
// to verify that tubes of equal radii stacked end-to-end along z reproduce a single longer tube.
std::shared_ptr<model::part::Part> tube(const std::string& name, double ri, double ro, double length,
                                  double density)
{
   return std::make_shared<model::part::Part>(name,
                                        model::InertiaTensors::Tube(ri, ro, length),
                                        tubeMass(ri, ro, length, density),
                                        Vector3::Zero());
}
} // namespace

TEST(PartCompositionTest, PointMassPairCompositeCmIsMassWeightedMidpoint)
{
   // Parent mass at its own CM (origin); child mass offset along the axis (-z aft). 3-DOF placement is
   // coaxial, so the offset is axial; a zero-length point mass has its fore plane at its CM, so there
   // is no datum shift here. The composite CM is the mass-weighted average.
   const double mp = 2.0, mc = 3.0, L = 4.0;
   auto parent = pointMass("parent", mp);
   parent->addChildPart(pointMass("child", mc), Vector3{0.0, 0.0, L});

   const Vector3 cm = parent->getCompositeCm(0.0);
   EXPECT_NEAR(cm(0), 0.0, 1e-12);
   EXPECT_NEAR(cm(1), 0.0, 1e-12);
   EXPECT_NEAR(cm(2), mc * L / (mp + mc), 1e-12); // = 2.4
   EXPECT_NEAR(parent->getCompositeMass(0.0), mp + mc, 1e-12);
}

TEST(PartCompositionTest, PointMassPairInertiaIsAboutCompositeCmNotParentCm)
{
   // Two point masses a distance L apart: inertia about their common CM is mu*L^2 on the two
   // transverse axes (mu = reduced mass), 0 about the line joining them. The pre-fix code computed
   // this about the PARENT's CM (mc*L^2), so this value pins the tensor to the composite CM.
   const double mp = 2.0, mc = 3.0, L = 4.0;
   auto parent = pointMass("parent", mp);
   parent->addChildPart(pointMass("child", mc), Vector3{0.0, 0.0, L}); // coaxial: joining line is z

   const double mu = mp * mc / (mp + mc);
   const double expected = mu * L * L; // 19.2
   const Matrix3 I = parent->getCompositeI(0.0);
   EXPECT_NEAR(I(2, 2), 0.0, 1e-12);       // along the joining line (z)
   EXPECT_NEAR(I(0, 0), expected, 1e-12);
   EXPECT_NEAR(I(1, 1), expected, 1e-12);
   EXPECT_NEAR(I(0, 1), 0.0, 1e-12);
   EXPECT_NEAR(I(0, 2), 0.0, 1e-12);
   EXPECT_NEAR(I(1, 2), 0.0, 1e-12);
}

TEST(PartCompositionTest, ThreeMassChainMatchesFlatReferenceDepth2)
{
   // A depth-2 chain root -> child -> grandchild. The pre-fix code shifted each subtree from its
   // part CM rather than its composite CM; because the parallel-axis map is not additive, that is
   // wrong for trees >= 2 deep. Compare against a flat reference that places the three masses at
   // their absolute positions and computes inertia about the common CM directly.
   const double mr = 1.0, mc = 2.0, mg = 3.0;
   const double a = 1.0, b = 2.0;            // child at a from root; grandchild at b from child

   auto child = pointMass("child", mc);
   child->addChildPart(pointMass("grandchild", mg), Vector3{0.0, 0.0, b}); // coaxial (axial chain)
   auto root = pointMass("root", mr);
   root->addChildPart(child, Vector3{0.0, 0.0, a});

   // Flat reference (masses on the axis at 0, a, a+b).
   const double x[3] = {0.0, a, a + b};
   const double m[3] = {mr, mc, mg};
   const double M = mr + mc + mg;
   double xc = 0.0;
   for(int i = 0; i < 3; ++i) xc += m[i] * x[i];
   xc /= M;
   double transverse = 0.0;
   for(int i = 0; i < 3; ++i) transverse += m[i] * (x[i] - xc) * (x[i] - xc);

   EXPECT_NEAR(root->getCompositeMass(0.0), M, 1e-12);
   EXPECT_NEAR(root->getCompositeCm(0.0)(2), xc, 1e-12); // coaxial chain along z (no datum shift: point root)

   const Matrix3 I = root->getCompositeI(0.0);
   EXPECT_NEAR(I(2, 2), 0.0, 1e-12);            // along the joining line (z)
   EXPECT_NEAR(I(0, 0), transverse, 1e-12);
   EXPECT_NEAR(I(1, 1), transverse, 1e-12);
}

TEST(PartCompositionTest, CloneIsADeepIndependentTypePreservingCopy)
{
   // clone() must produce a fully independent deep copy that preserves the dynamic type (no slicing).
   // Build a HollowSphere with a child, clone it, then mutate the original -> the clone is untouched.
   auto body = std::make_shared<model::part::HollowSphere>("body", 0.04, 0.05, 2700.0);
   body->addChildPart(pointMass("tip", 0.1), Vector3{0.2, 0.0, 0.0});

   auto copy = body->clone();
   const double massBefore = copy->getCompositeMass(0.0);
   const double iyyBefore = copy->getCompositeI(0.0)(1, 1);

   // Mutate the original every which way.
   body->setMass(99.0);
   body->addChildPart(pointMass("extra", 50.0), Vector3{1.0, 0.0, 0.0});

   EXPECT_DOUBLE_EQ(copy->getCompositeMass(0.0), massBefore);
   EXPECT_DOUBLE_EQ(copy->getCompositeI(0.0)(1, 1), iyyBefore);

   // Type preserved: the clone is still a HollowSphere, not a sliced base Part.
   EXPECT_NE(dynamic_cast<model::part::HollowSphere*>(copy.get()), nullptr);
}

TEST(PartCompositionTest, SetMassAndSetIInvalidateCompositeCache)
{
   // setMass() and setI() must flag the composite cache stale; before the fix setI() did not, so a
   // later getCompositeI(0.0) returned a value computed from the old tensor.
   model::part::Part part("p", model::InertiaTensors::SolidSphere(1.0), 2.0, Vector3{0.0, 0.0, 0.0});
   EXPECT_DOUBLE_EQ(part.getCompositeI(0.0)(0, 0), 0.8); // 2.0 * 0.4

   part.setMass(4.0);
   EXPECT_DOUBLE_EQ(part.getCompositeMass(0.0), 4.0);
   EXPECT_DOUBLE_EQ(part.getCompositeI(0.0)(0, 0), 1.6); // 4.0 * 0.4 -- setMass invalidated the cache

   part.setI(model::InertiaTensors::SolidSphere(2.0)); // per-unit-mass diagonal 0.4 * 4 = 1.6
   EXPECT_DOUBLE_EQ(part.getCompositeI(0.0)(0, 0), 6.4);   // 4.0 * 1.6 -- setI invalidated the cache
}

namespace
{
// Assert that a composite tensor equals the full (mass-weighted) tensor of a single tube of the
// merged length, element by element, with per-element trace for clear failure messages.
void expectMatchesSingleTube(const Matrix3& actual, double ri, double ro, double totalLength,
                             double totalMass)
{
   const Matrix3 expected = totalMass * model::InertiaTensors::Tube(ri, ro, totalLength);
   for(int r = 0; r < 3; ++r)
   {
      for(int c = 0; c < 3; ++c)
      {
         SCOPED_TRACE(testing::Message() << "inertia element (" << r << ", " << c << ")");
         EXPECT_NEAR(actual(r, c), expected(r, c), 1e-12);
      }
   }
}
} // namespace

TEST(PartCompositionTest, TwoTubesEndToEndEqualOneLongerTube)
{
   // Two coaxial tubes of identical radii, stacked end-to-end along their z-axis, must be
   // indistinguishable from a single tube of the summed length: same mass, same CM at the merged
   // center, same full inertia tensor. The transverse moment depends on L^2, so this exercises the
   // parallel-axis composition far more sharply than point masses do.
   const double ri = 0.02, ro = 0.03, density = 1500.0;
   const double L1 = 0.10, L2 = 0.20;

   auto assembly = tube("t1", ri, ro, L1, density);
   // tube 2's CM sits (L1 + L2)/2 along +z from tube 1's CM (touching faces).
   assembly->addChildPart(tube("t2", ri, ro, L2, density), Vector3{0.0, 0.0, (L1 + L2) / 2.0});

   const double totalLength = L1 + L2;
   const double totalMass = tubeMass(ri, ro, totalLength, density);

   EXPECT_NEAR(assembly->getCompositeMass(0.0), totalMass, 1e-12);

   // Merged center is L2/2 beyond tube 1's own center (relative to tube 1's CM).
   const Vector3 cm = assembly->getCompositeCm(0.0);
   EXPECT_NEAR(cm(0), 0.0, 1e-12);
   EXPECT_NEAR(cm(1), 0.0, 1e-12);
   EXPECT_NEAR(cm(2), L2 / 2.0, 1e-12);

   expectMatchesSingleTube(assembly->getCompositeI(0.0), ri, ro, totalLength, totalMass);
}

TEST(PartCompositionTest, ThreeTubesEndToEndEqualOneLongerTubeDepth2)
{
   // Same idea at depth 2: a chain t1 -> t2 -> t3 stacked along z. t2 is itself a composite (it owns
   // t3) when it is attached to t1, so this checks that the composition shifts each sub-assembly from
   // its OWN composite CM -- the case the pre-fix code got wrong because parallel-axis is not
   // additive across a non-CM intermediate point.
   const double ri = 0.02, ro = 0.03, density = 1500.0;
   const double L1 = 0.10, L2 = 0.20, L3 = 0.30;

   auto t2 = tube("t2", ri, ro, L2, density);
   t2->addChildPart(tube("t3", ri, ro, L3, density), Vector3{0.0, 0.0, (L2 + L3) / 2.0});
   auto assembly = tube("t1", ri, ro, L1, density);
   assembly->addChildPart(t2, Vector3{0.0, 0.0, (L1 + L2) / 2.0});

   const double totalLength = L1 + L2 + L3;
   const double totalMass = tubeMass(ri, ro, totalLength, density);

   EXPECT_NEAR(assembly->getCompositeMass(0.0), totalMass, 1e-12);

   // Merged center is (L2 + L3)/2 beyond tube 1's own center (relative to tube 1's CM).
   const Vector3 cm = assembly->getCompositeCm(0.0);
   EXPECT_NEAR(cm(0), 0.0, 1e-12);
   EXPECT_NEAR(cm(1), 0.0, 1e-12);
   EXPECT_NEAR(cm(2), (L2 + L3) / 2.0, 1e-12);

   expectMatchesSingleTube(assembly->getCompositeI(0.0), ri, ro, totalLength, totalMass);
}

TEST(PartCompositionTest, PartsHaveUniqueIdsAndCloneGetsAFreshId)
{
   // Identical name and mass properties must still yield distinct ids -- the id, not the name, is the
   // identity.
   auto a = pointMass("same", 1.0);
   auto b = pointMass("same", 1.0);
   EXPECT_NE(a->getId(), b->getId());

   // A clone is a separate object, so it gets a fresh id rather than inheriting the original's.
   EXPECT_NE(a->clone()->getId(), a->getId());
}

TEST(PartCompositionTest, FindByIdLocatesAdoptedPartsAndRejectsAbsent)
{
   auto root = pointMass("root", 1.0);
   auto child = pointMass("child", 1.0);
   const model::part::Part::Id childId = child->getId();
   root->addChildPart(child, Vector3{1.0, 0.0, 0.0}); // adopts: same object, same id, now in the tree

   EXPECT_EQ(root->findById(root->getId()), root.get());
   EXPECT_EQ(root->findById(childId), child.get());    // adopted -> findable by its unchanged id
   EXPECT_EQ(root->findById(0), nullptr);              // 0 is reserved and never assigned
   EXPECT_EQ(root->findById(123456789), nullptr);      // absent
}

TEST(PartCompositionTest, TypeNameReportsTheConcreteType)
{
   // The base Part reports "Part"; each concrete leaf reports its own stable tag. These strings are
   // the part-factory keys and the design-file type attribute (P2 persistence), so they are pinned
   // here. A default MotorModel is unignited, so Motor's eager getMass(0) is a safe 0.
   model::part::Part base("base", Matrix3::Zero(), 1.0, Vector3::Zero());
   EXPECT_EQ(base.typeName(), "Part");
   EXPECT_EQ(model::part::HollowSphere("s", 0.04, 0.05, 2700.0).typeName(), "HollowSphere");
   EXPECT_EQ(model::part::BodyTube("b", 0.0, 0.019, 0.20, 680.0).typeName(), "BodyTube");
   EXPECT_EQ(model::part::ConicalNoseCone("n", 0.019, 0.10, 0.0, 2700.0).typeName(), "NoseCone");
   EXPECT_EQ(model::part::FinSet("f", 3, 0.10, 0.05, 0.05, 0.04, 0.003, 0.019, 600.0).typeName(),
             "FinSet");
   EXPECT_EQ(model::part::Motor("m", model::MotorModel{}).typeName(), "Motor");
}

TEST(PartCompositionTest, GetChildPartsExposesChildrenAndAttachPositionsInOrder)
{
   auto root = pointMass("root", 1.0);
   auto a = pointMass("a", 1.0);
   auto b = pointMass("b", 1.0);
   const auto aId = a->getId();
   const auto bId = b->getId();
   root->addChildPart(a, Vector3{0.0, 0.0, -0.3}); // axial: 3-DOF placement is coaxial (radial deferred)
   root->addChildPart(b, Vector3{0.0, 0.0, -0.5});

   const auto& kids = root->getChildParts();
   ASSERT_EQ(kids.size(), 2u);
   EXPECT_EQ(kids[0].first->getId(), aId);   // attachment order is preserved
   EXPECT_EQ(kids[1].first->getId(), bId);
   // The legacy CM-to-CM offset is recovered into a StationLink; for these zero-length point masses the
   // shim stores the axial offset as the gap of an abut link.
   EXPECT_EQ(kids[0].second.seat, model::part::SeatKind::Abut);
   EXPECT_DOUBLE_EQ(kids[0].second.gap, -0.3);
   EXPECT_DOUBLE_EQ(kids[1].second.gap, -0.5);
}

TEST(PartCompositionTest, RemoveChildByIdDetachesReturnsAndRecomputesComposite)
{
   const double mp = 2.0, mc = 3.0, L = 4.0;
   auto parent = pointMass("parent", mp);
   auto child = pointMass("child", mc);
   const auto childId = child->getId();
   parent->addChildPart(child, Vector3{0.0, 0.0, L}); // coaxial (axial)

   // Cache the composite WITH the child, so the post-removal reads must rebuild to stay correct.
   EXPECT_NEAR(parent->getCompositeMass(0.0), mp + mc, 1e-12);
   EXPECT_NEAR(parent->getCompositeCm(0.0)(2), mc * L / (mp + mc), 1e-12);

   auto detached = parent->removeChildById(childId);
   ASSERT_NE(detached, nullptr);
   EXPECT_EQ(detached->getId(), childId);           // returns the very node that was removed
   EXPECT_TRUE(parent->getChildParts().empty());

   // Composite recomputed: a lone parent at its own CM.
   EXPECT_NEAR(parent->getCompositeMass(0.0), mp, 1e-12);
   EXPECT_NEAR(parent->getCompositeCm(0.0)(2), 0.0, 1e-12);

   // A second remove of the now-absent id is a no-op returning nullptr; the root never removes itself.
   EXPECT_EQ(parent->removeChildById(childId), nullptr);
   EXPECT_EQ(parent->removeChildById(parent->getId()), nullptr);
}

TEST(PartCompositionTest, RemoveChildByIdFindsADescendantDeepInTheTree)
{
   // The match is a grandchild, so the recursive branch (not the direct-child scan) does the work,
   // and the whole ancestor chain must end up recomputed.
   auto root = pointMass("root", 1.0);
   auto child = pointMass("child", 1.0);
   auto grandchild = pointMass("grandchild", 1.0);
   const auto gcId = grandchild->getId();
   child->addChildPart(grandchild, Vector3{1.0, 0.0, 0.0});
   root->addChildPart(child, Vector3{1.0, 0.0, 0.0});

   EXPECT_NEAR(root->getCompositeMass(0.0), 3.0, 1e-12);
   auto detached = root->removeChildById(gcId);
   ASSERT_NE(detached, nullptr);
   EXPECT_EQ(detached->getId(), gcId);
   EXPECT_NEAR(root->getCompositeMass(0.0), 2.0, 1e-12);
}

TEST(PartCompositionTest, GetNameReturnsThePartName)
{
   model::part::Part p("AvionicsBay", Matrix3::Zero(), 1.0, Vector3::Zero());
   EXPECT_EQ(p.getName(), "AvionicsBay");
}

TEST(PartCompositionTest, TypeNameDispatchesPolymorphicallyThroughBasePointer)
{
   // The P2 part factory and design serializer read typeName() through a Part* / shared_ptr<Part>,
   // so the VIRTUAL dispatch is the contract that matters downstream -- pin it through base pointers.
   std::shared_ptr<model::part::Part> s =
      std::make_shared<model::part::HollowSphere>("s", 0.04, 0.05, 2700.0);
   std::shared_ptr<model::part::Part> b =
      std::make_shared<model::part::BodyTube>("b", 0.0, 0.019, 0.20, 680.0);
   std::shared_ptr<model::part::Part> n =
      std::make_shared<model::part::ConicalNoseCone>("n", 0.019, 0.10, 0.0, 2700.0);
   std::shared_ptr<model::part::Part> f =
      std::make_shared<model::part::FinSet>("f", 3, 0.10, 0.05, 0.05, 0.04, 0.003, 0.019, 600.0);
   std::shared_ptr<model::part::Part> m =
      std::make_shared<model::part::Motor>("m", model::MotorModel{});
   EXPECT_EQ(s->typeName(), "HollowSphere");
   EXPECT_EQ(b->typeName(), "BodyTube");
   EXPECT_EQ(n->typeName(), "NoseCone");
   EXPECT_EQ(f->typeName(), "FinSet");
   EXPECT_EQ(m->typeName(), "Motor");
}

TEST(PartCompositionTest, GetChildPartsOnALeafIsEmpty)
{
   auto leaf = pointMass("leaf", 1.0);
   EXPECT_TRUE(leaf->getChildParts().empty());
}

TEST(PartCompositionTest, RemoveChildByIdReturnsAnIntactMultiNodeSubtree)
{
   // Remove an INTERMEDIATE node that owns its own child: the returned sub-tree must keep its
   // internal structure (the grandchild still attached, parent pointers intact) and be usable as a
   // standalone root. This is the contract removepart relies on (a removed assembly stays whole).
   auto root = pointMass("root", 1.0);
   auto mid = pointMass("mid", 2.0);
   auto leaf = pointMass("leaf", 3.0);
   const auto midId = mid->getId();
   const auto leafId = leaf->getId();
   mid->addChildPart(leaf, Vector3{1.0, 0.0, 0.0});
   root->addChildPart(mid, Vector3{1.0, 0.0, 0.0});

   auto detached = root->removeChildById(midId);
   ASSERT_NE(detached, nullptr);
   EXPECT_EQ(detached->getId(), midId);

   // Root is now childless; the detached sub-tree kept its shape and works as its own root.
   EXPECT_TRUE(root->getChildParts().empty());
   EXPECT_NEAR(root->getCompositeMass(0.0), 1.0, 1e-12);
   ASSERT_EQ(detached->getChildParts().size(), 1u);
   EXPECT_EQ(std::get<0>(detached->getChildParts()[0])->getId(), leafId);
   EXPECT_NEAR(detached->getCompositeMass(0.0), 5.0, 1e-12); // mid(2) + leaf(3), standalone
   EXPECT_EQ(detached->findById(leafId), std::get<0>(detached->getChildParts()[0]).get());
}

namespace model::part
{
// White-box fixture: grants the re-parenting test access to Part's private parent pointers, child
// list, and dirty flag. Lives in namespace model::part so the unqualified `friend class
// PartCompositionAccess;` in Part.h refers to it, and so TEST_F below finds it by name.
class PartCompositionAccess : public ::testing::Test
{
protected:
   static Part* parentOf(const Part& p) { return p.parent; }
   static Part& childAt(const Part& p, std::size_t i) { return *std::get<0>(p.childParts.at(i)); }
   static bool isDirty(const Part& p) { return p.needsRecomputing; }
};

TEST_F(PartCompositionAccess, AdoptedChildIsReparentedAndDirtyPropagates)
{
   auto root = std::make_shared<Part>("root", Matrix3::Zero(), 1.0, Vector3::Zero());
   auto child = std::make_shared<Part>("child", Matrix3::Zero(), 1.0, Vector3::Zero());
   auto grandchild = std::make_shared<Part>("grandchild", Matrix3::Zero(), 1.0, Vector3::Zero());
   child->addChildPart(grandchild, Vector3{1.0, 0.0, 0.0});
   root->addChildPart(child, Vector3{1.0, 0.0, 0.0});

   // Adopted, not copied: the very objects we created are in the tree, correctly re-parented.
   EXPECT_EQ(&childAt(*root, 0), child.get());
   EXPECT_EQ(parentOf(*child), root.get());
   EXPECT_EQ(parentOf(*grandchild), child.get());

   // Dirtying the deepest node must propagate up to root through those parent pointers.
   root->getCompositeI(0.0); // clean the whole tree
   EXPECT_FALSE(isDirty(*root));
   grandchild->setMass(5.0); // walks up: grandchild -> child -> root
   EXPECT_TRUE(isDirty(*root));
}

TEST_F(PartCompositionAccess, CloneReparentsSubtreeWithFreshUniqueIds)
{
   auto root = std::make_shared<Part>("root", Matrix3::Zero(), 1.0, Vector3::Zero());
   root->addChildPart(std::make_shared<Part>("child", Matrix3::Zero(), 1.0, Vector3::Zero()),
                      Vector3{1.0, 0.0, 0.0});
   childAt(*root, 0).addChildPart(
      std::make_shared<Part>("grandchild", Matrix3::Zero(), 1.0, Vector3::Zero()),
      Vector3{1.0, 0.0, 0.0});

   auto copy = root->clone();
   Part& copyChild = childAt(*copy, 0);
   Part& copyGrandchild = childAt(copyChild, 0);

   // Re-parented within the clone, not pointing back at the originals.
   EXPECT_EQ(parentOf(copyChild), copy.get());
   EXPECT_EQ(parentOf(copyGrandchild), &copyChild);

   // Fresh, unique ids throughout; none shared with the originals.
   EXPECT_NE(copy->getId(), root->getId());
   EXPECT_NE(copyChild.getId(), childAt(*root, 0).getId());
   EXPECT_NE(copyGrandchild.getId(), copy->getId());
   EXPECT_NE(copyGrandchild.getId(), copyChild.getId());

   // findById resolves cloned nodes from the clone's root.
   EXPECT_EQ(copy->findById(copyGrandchild.getId()), &copyGrandchild);
}

TEST_F(PartCompositionAccess, RemoveChildByIdClearsParentAndDirtiesAncestors)
{
   // Mechanism check: removeChildById must clear the detached node's parent pointer and dirty the
   // ex-parent AND every ancestor. The dirty FLAG (not the mass-delta gate) is what drives the
   // rebuild, so a zero-net-mass-change removal still refreshes -- all nodes are mass 1 here; the
   // point is purely the parent-pointer + dirty-flag bookkeeping.
   auto root = std::make_shared<Part>("root", Matrix3::Zero(), 1.0, Vector3::Zero());
   auto child = std::make_shared<Part>("child", Matrix3::Zero(), 1.0, Vector3::Zero());
   auto grandchild = std::make_shared<Part>("grandchild", Matrix3::Zero(), 1.0, Vector3::Zero());
   const Part::Id gcId = grandchild->getId();
   child->addChildPart(grandchild, Vector3{1.0, 0.0, 0.0});
   root->addChildPart(child, Vector3{1.0, 0.0, 0.0});

   root->getCompositeI(0.0);   // cleans root...
   child->getCompositeI(0.0);  // ...but cleaning root does not clean descendants, so clean `child` too
   EXPECT_FALSE(isDirty(*root));
   EXPECT_FALSE(isDirty(*child));

   auto detached = root->removeChildById(gcId); // a descendant of `child`
   ASSERT_NE(detached, nullptr);
   EXPECT_EQ(parentOf(*detached), nullptr);     // detached node is re-rooted (no parent)
   EXPECT_TRUE(isDirty(*child));                // ex-parent dirtied
   EXPECT_TRUE(isDirty(*root));                 // ... and propagated up to the root
}
} // namespace model::part
