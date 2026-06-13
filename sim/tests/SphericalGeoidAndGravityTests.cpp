// Unit tests for the spherical-Earth geoid and the Newtonian spherical gravity model.
//
// The gravity model runs in the local launch frame (z = altitude above the pad) and uses
// the geoid's ground radius to form a geocentric distance, so it must be FINITE at the pad
// -- the old SphericalGravityModel divided by r = 0 there and produced NaN, the F1 hang in
// TODO.md P0 -- and must weaken with altitude. See TODO.md P0 / F1.

/// \cond
#include <memory>
/// \endcond

#include <gtest/gtest.h>

#include "sim/GravityModel.h"
#include "sim/SphericalGeoidModel.h"
#include "sim/SphericalGravityModel.h"
#include "utils/math/Constants.h"
#include "utils/math/MathTypes.h"

namespace
{
constexpr double R = utils::math::Constants::meanEarthRadiusWGS84;          // 6371008.8 m
constexpr double GM = static_cast<double>(utils::math::Constants::earthGM); // 3.986004418e14 m^3/s^2

// Textbook inverse-square magnitude at geocentric distance r.
double expectedG(double r) { return GM / (r * r); }

// A spherical gravity model backed by the spherical-Earth geoid, held through the base
// interface (the way Environment hands it to RocketModel::getForces).
std::shared_ptr<sim::GravityModel> makeGravity()
{
   return std::make_shared<sim::SphericalGravityModel>(
      std::make_shared<sim::SphericalGeoidModel>());
}
} // namespace

// The spherical geoid returns the mean Earth radius regardless of latitude/longitude.
TEST(SphericalGeoidModelTest, ReturnsMeanRadiusEverywhere)
{
   sim::SphericalGeoidModel geoid;
   EXPECT_DOUBLE_EQ(geoid.getGroundLevel(0.0, 0.0), R);
   EXPECT_DOUBLE_EQ(geoid.getGroundLevel(45.0, -90.0), R);
   EXPECT_DOUBLE_EQ(geoid.getGroundLevel(-33.9, 151.2), R);
}

// F1 regression: at the pad (the local origin) the acceleration must be finite. The
// previous model computed GM / r^3 with r = 0 -> NaN, which spun the propagator loop.
TEST(SphericalGravityModelTest, FiniteAtLaunchOrigin)
{
   auto gravity = makeGravity();
   const Vector3 a = gravity->getAccel(0.0, 0.0, 0.0);
   EXPECT_TRUE(a.allFinite());
}

// At the pad gravity points straight down with magnitude GM / R^2 (~9.82 m/s^2).
TEST(SphericalGravityModelTest, SurfaceGravityIsInverseSquareAtGroundRadius)
{
   auto gravity = makeGravity();
   const Vector3 a = gravity->getAccel(0.0, 0.0, 0.0);
   EXPECT_DOUBLE_EQ(a.x(), 0.0);
   EXPECT_DOUBLE_EQ(a.y(), 0.0);
   EXPECT_NEAR(a.z(), -expectedG(R), 1e-9); // -GM / R^2
   EXPECT_NEAR(a.z(), -9.82, 1e-2);         // sanity: ~ -9.8 m/s^2
}

// Gravity weakens with altitude and tracks GM / (R + z)^2 exactly.
TEST(SphericalGravityModelTest, MagnitudeDecreasesWithAltitude)
{
   auto gravity = makeGravity();
   const double z = 10000.0; // 10 km
   const Vector3 ground = gravity->getAccel(0.0, 0.0, 0.0);
   const Vector3 high   = gravity->getAccel(0.0, 0.0, z);

   EXPECT_LT(high.norm(), ground.norm());
   EXPECT_NEAR(high.z(), -expectedG(R + z), 1e-9);
   EXPECT_DOUBLE_EQ(high.x(), 0.0);
   EXPECT_DOUBLE_EQ(high.y(), 0.0);
}

// Off the launch column the acceleration points back toward Earth's center: a small inward
// horizontal component (toward x = 0) plus the dominant downward component.
TEST(SphericalGravityModelTest, PointsTowardEarthCenter)
{
   auto gravity = makeGravity();
   const Vector3 a = gravity->getAccel(1000.0, 0.0, 0.0); // 1 km downrange
   EXPECT_LT(a.x(), 0.0); // pulled back toward the launch column
   EXPECT_LT(a.z(), 0.0); // and downward
   EXPECT_TRUE(a.allFinite());
}
