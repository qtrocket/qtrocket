#include "sim/ConstantAtmosphere.h"
#include "sim/VacuumAtmosphere.h"

#include <array>
#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

namespace {

constexpr std::array<double, 6> altitudeSamples{
   -100.0,
   0.0,
   123.456,
   11000.0,
   1.0e6,
   std::numeric_limits<double>::quiet_NaN(),
};

void expectVacuumProperties(sim::AtmosphericModel& atmosphere, double altitude)
{
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(altitude), 0.0);
   EXPECT_DOUBLE_EQ(atmosphere.getPressure(altitude), 0.0);
   EXPECT_DOUBLE_EQ(atmosphere.getTemperature(altitude), 0.0);
   EXPECT_DOUBLE_EQ(atmosphere.getSpeedOfSound(altitude), 0.0);
   EXPECT_DOUBLE_EQ(atmosphere.getDynamicViscosity(altitude), 0.0);
}

void expectConstantProperties(sim::AtmosphericModel& atmosphere, double altitude)
{
   EXPECT_DOUBLE_EQ(atmosphere.getDensity(altitude), 1.225);
   EXPECT_DOUBLE_EQ(atmosphere.getPressure(altitude), 101325.0);
   EXPECT_DOUBLE_EQ(atmosphere.getTemperature(altitude), 288.15);
   EXPECT_DOUBLE_EQ(atmosphere.getSpeedOfSound(altitude), 340.294);
   EXPECT_DOUBLE_EQ(atmosphere.getDynamicViscosity(altitude), 1.78938e-5);

   EXPECT_GT(atmosphere.getDensity(altitude), 0.0);
   EXPECT_GT(atmosphere.getPressure(altitude), 0.0);
   EXPECT_GT(atmosphere.getTemperature(altitude), 0.0);
   EXPECT_GT(atmosphere.getSpeedOfSound(altitude), 0.0);
   EXPECT_GT(atmosphere.getDynamicViscosity(altitude), 0.0);
}

} // namespace

TEST(VacuumAtmosphereTest, EveryPropertyIsZeroAtAllAltitudes)
{
   sim::VacuumAtmosphere atmosphere;

   for(double altitude : altitudeSamples) {
      expectVacuumProperties(atmosphere, altitude);
   }
}

TEST(VacuumAtmosphereTest, DispatchesThroughAtmosphericModelInterface)
{
   std::unique_ptr<sim::AtmosphericModel> atmosphere =
      std::make_unique<sim::VacuumAtmosphere>();

   for(double altitude : altitudeSamples) {
      expectVacuumProperties(*atmosphere, altitude);
   }
}

TEST(ConstantAtmosphereTest, EveryPropertyMatchesTheSeaLevelConstantAtAllAltitudes)
{
   sim::ConstantAtmosphere atmosphere;

   for(double altitude : altitudeSamples) {
      expectConstantProperties(atmosphere, altitude);
   }
}

TEST(ConstantAtmosphereTest, DispatchesThroughAtmosphericModelInterface)
{
   std::unique_ptr<sim::AtmosphericModel> atmosphere =
      std::make_unique<sim::ConstantAtmosphere>();

   for(double altitude : altitudeSamples) {
      expectConstantProperties(*atmosphere, altitude);
   }
}
