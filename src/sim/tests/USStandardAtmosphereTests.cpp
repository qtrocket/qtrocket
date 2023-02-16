#include "../USStandardAtmosphere.h"
#include <gtest/gtest.h>

#include <iostream>

TEST(USStandardAtmosphereTests, DensityTests)
{
   sim::USStandardAtmosphere atm;
   double seaLevelDensity = 1.2250;
   double testDensity = atm.getDensity(0.0);
   ASSERT_EQ(seaLevelDensity, testDensity);

   // Test some other values
   for(int i = 1; i <= 10; ++i)
   {
      double altitude = 1000.0 * i;
      std::cout << altitude << ": " << atm.getDensity(altitude) << std::endl;
   }

}

int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}