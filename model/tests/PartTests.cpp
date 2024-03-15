#include <gtest/gtest.h>

#include "model/Part.h"

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
