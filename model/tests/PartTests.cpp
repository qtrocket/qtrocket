#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <stdexcept>

#include "model/Part.h"
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
   model::Part testPart("testPart",
                        inertia,
                        1.0,
                        cm);
   
   Matrix3 inertia2;
   inertia2 << 1, 0, 0,
               0, 1, 0,
               0, 0, 1;
   Vector3 cm2{1, 0, 0};
   model::Part testPart2("testPart2",
                        inertia2,
                        1.0,
                        cm2);
   Vector3 R{2.0, 2.0, 2.0};
   testPart.addChildPart(testPart2, R);


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

   model::HollowSphere sphere("body", ri, ro, density);

   const double expectedMass = expectedHollowSphereMass(ri, ro, density);
   EXPECT_NEAR(sphere.getMass(0.0), expectedMass, 1e-12);
   EXPECT_NEAR(sphere.getVolume(), expectedMass / density, 1e-15);

   // getCompositeI() is the full, mass-weighted tensor (kg*m^2).
   const Matrix3 I = sphere.getCompositeI();
   const double expectedDiag = expectedHollowSphereInertiaDiagonal(ri, ro, density);
   EXPECT_NEAR(I(0, 0), expectedDiag, 1e-12);
   EXPECT_NEAR(I(1, 1), expectedDiag, 1e-12);
   EXPECT_NEAR(I(2, 2), expectedDiag, 1e-12);
   // Isotropic: off-diagonals vanish.
   EXPECT_DOUBLE_EQ(I(0, 1), 0.0);
   EXPECT_DOUBLE_EQ(I(0, 2), 0.0);
   EXPECT_DOUBLE_EQ(I(1, 2), 0.0);

   // getI() is per-unit-mass, so getCompositeI() == mass * getI().
   EXPECT_NEAR(I(0, 0), expectedMass * sphere.getI()(0, 0), 1e-12);
}

TEST(HollowSphereTest, ReducesToSolidSphereWhenInnerRadiusZero)
{
   const double ro = 0.05;
   const double density = 2700.0;

   model::HollowSphere sphere("solid", 0.0, ro, density);

   const double mass = sphere.getMass(0.0);
   // Solid sphere: I = (2/5) m ro^2 on each axis.
   EXPECT_NEAR(sphere.getCompositeI()(0, 0), mass * (2.0 / 5.0) * ro * ro, 1e-12);
   // ... which is exactly mass * InertiaTensors::SolidSphere(ro).
   EXPECT_NEAR(sphere.getCompositeI()(0, 0),
               mass * model::InertiaTensors::SolidSphere(ro)(0, 0), 1e-12);
}

TEST(HollowSphereTest, RejectsNonPhysicalGeometry)
{
   EXPECT_THROW(model::HollowSphere("bad", 0.05, 0.04, 2700.0), std::invalid_argument); // ri > ro
   EXPECT_THROW(model::HollowSphere("bad", 0.04, 0.04, 2700.0), std::invalid_argument); // ri == ro
   EXPECT_THROW(model::HollowSphere("bad", 0.00, 0.05, 0.0),    std::invalid_argument); // density 0
}

TEST(PartTest, StoresInertiaPerUnitMassWithMassWeightedComposite)
{
   // The bare tensor is per-unit-mass; the composite is full (mass * per-mass). With mass = 2.0 and
   // SolidSphere(1.0) = 0.4 on the diagonal, getI() = 0.4 but getCompositeI() = 0.8 -- this would be
   // 0.4 if Part stored the tensor un-weighted, so it locks the mass multiply in.
   model::Part part("p", model::InertiaTensors::SolidSphere(1.0), 2.0, Vector3{0.0, 0.0, 0.0});
   EXPECT_DOUBLE_EQ(part.getI()(0, 0), 0.4);
   EXPECT_DOUBLE_EQ(part.getCompositeI()(0, 0), 0.8);
}
