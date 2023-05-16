#include <gtest/gtest.h>

#include "sim/USStandardAtmosphere.h"

TEST(USStandardAtmosphereTests, DensityTests)
{
   sim::USStandardAtmosphere atmosphere;

   EXPECT_DOUBLE_EQ(atmosphere.getDensity(0.0), 1.225);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(1000.0), 1.112);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(2000.0), 1.007);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(3000.0), 0.9093);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(4000.0), 0.8194);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(5000.0), 0.7364);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(6000.0), 0.6601);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(7000.0), 0.5900);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(8000.0), 0.5258);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(9000.0), 0.4647);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(10000.0), 0.4135);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(15000.0), 0.1948);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(20000.0), 0.08891);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(25000.0), 0.04008);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(30000.0), 0.01841);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(40000.0), 0.003996);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(50000.0), 0.001027);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(60000.0), 0.0003097);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(70000.0), 0.00008283);
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(80000.0), 0.00001846);
}
