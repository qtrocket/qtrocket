#include <gtest/gtest.h>

#include "sim/USStandardAtmosphere.h"

TEST(USStandardAtmosphereTests, DensityTests)
{
   sim::USStandardAtmosphere atmosphere;

   // Test that the calucated values are with 0.1% of the published values in the NOAA report
   EXPECT_NEAR(atmosphere.getDensity(0.0) / 1.225, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getDensity(1000.0) / 1.112, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getDensity(2000.0) / 1.007, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getDensity(3000.0) / 0.9093, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getDensity(4000.0) / 0.8194, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getDensity(5000.0) / 0.7364, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getDensity(6000.0) / 0.6601, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getDensity(7000.0) / 0.5900, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getDensity(15000.0) / 0.19367, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getDensity(20000.0) / 0.088035, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getDensity(25000.0) / 0.039466, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getDensity(30000.0) / 0.018012, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getDensity(40000.0) / 0.0038510, 1.0, 0.001);

   // These are generally accurate to ~0.5%. Slight deviation of calculated
   // density from the given density in the report table
   EXPECT_NEAR(atmosphere.getDensity(8000.0) / 0.52579, 1.0, 0.005);
   EXPECT_NEAR(atmosphere.getDensity(9000.0) / 0.46706, 1.0, 0.005);
   EXPECT_NEAR(atmosphere.getDensity(10000.0) / 0.41351, 1.0, 0.005);
   EXPECT_NEAR(atmosphere.getDensity(50000.0) / 0.00097752, 1.0, 0.005);
   EXPECT_NEAR(atmosphere.getDensity(60000.0) / 0.00028832, 1.0, 0.005);
   EXPECT_NEAR(atmosphere.getDensity(70000.0) / 0.000074243, 1.0, 0.005);
   EXPECT_NEAR(atmosphere.getDensity(80000.0) / 0.000015701, 1.0, 0.005);

}

TEST(USStandardAtmosphereTests, PressureTests)
{
   sim::USStandardAtmosphere atmosphere;

   // Test that the calucated values are with 0.1% of the published values in the NOAA report
   EXPECT_NEAR(atmosphere.getPressure(0.0)    / 101325.0, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(1000.0) / 89876.0, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(2000.0) / 79501.0, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(3000.0) / 70108.0, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(4000.0) / 61640.0, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(5000.0) / 54019.0, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(6000.0) / 47181.0, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(7000.0) / 41060.0, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(8000.0) / 35599.0, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(9000.0) / 30742.0, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(10000.0) / 26436.0, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(15000.0) / 12044.0, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(20000.0) / 5474.8, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(25000.0) / 2511.0, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(30000.0) / 1171.8, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(40000.0) / 277.52, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(50000.0) / 75.944, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(60000.0) / 20.314, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(70000.0) / 4.6342, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getPressure(80000.0) / 0.88627, 1.0, 0.001);

}

TEST(USStandardAtmosphereTests, TemperatureTests)
{
   sim::USStandardAtmosphere atmosphere;

   // Test that the calucated values are with 0.1% of the published values in the NOAA report
   EXPECT_NEAR(atmosphere.getTemperature(0.0)    / 288.15, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(1000.0) / 281.651, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(2000.0) / 275.154, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(3000.0) / 268.659, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(4000.0) / 262.166, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(5000.0) / 255.676, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(6000.0) / 249.187, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(7000.0) / 242.7, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(8000.0) / 236.215, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(9000.0) / 229.733, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(10000.0) / 223.252, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(15000.0) / 216.65, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(20000.0) / 216.65, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(25000.0) / 221.552, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(30000.0) / 226.509, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(40000.0) / 251.05, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(50000.0) / 270.65, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(60000.0) / 245.45, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(70000.0) / 217.450, 1.0, 0.001);
   EXPECT_NEAR(atmosphere.getTemperature(80000.0) / 196.650, 1.0, 0.001);

}