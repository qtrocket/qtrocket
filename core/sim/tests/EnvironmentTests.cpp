#include "sim/Environment.h"

#include "utils/math/Constants.h"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

void expectConstantAtmosphere(const std::shared_ptr<sim::AtmosphericModel>& atmosphere)
{
    ASSERT_TRUE(atmosphere);
    EXPECT_DOUBLE_EQ(atmosphere->getDensity(1234.0), 1.225);
    EXPECT_DOUBLE_EQ(atmosphere->getPressure(1234.0), 101325.0);
    EXPECT_DOUBLE_EQ(atmosphere->getTemperature(1234.0), 288.15);
    EXPECT_DOUBLE_EQ(atmosphere->getSpeedOfSound(1234.0), 340.294);
    EXPECT_DOUBLE_EQ(atmosphere->getDynamicViscosity(1234.0), 1.78938e-5);
}

void expectVacuumAtmosphere(const std::shared_ptr<sim::AtmosphericModel>& atmosphere)
{
    ASSERT_TRUE(atmosphere);
    EXPECT_DOUBLE_EQ(atmosphere->getDensity(1234.0), 0.0);
    EXPECT_DOUBLE_EQ(atmosphere->getPressure(1234.0), 0.0);
    EXPECT_DOUBLE_EQ(atmosphere->getTemperature(1234.0), 0.0);
    EXPECT_DOUBLE_EQ(atmosphere->getSpeedOfSound(1234.0), 0.0);
    EXPECT_DOUBLE_EQ(atmosphere->getDynamicViscosity(1234.0), 0.0);
}

void expectConstantGravity(const std::shared_ptr<sim::GravityModel>& gravity)
{
    ASSERT_TRUE(gravity);
    const Vector3 accel = gravity->getAccel(10.0, -20.0, 3000.0);
    EXPECT_DOUBLE_EQ(accel.x(), 0.0);
    EXPECT_DOUBLE_EQ(accel.y(), 0.0);
    EXPECT_DOUBLE_EQ(accel.z(), -utils::math::Constants::g0);
}

} // namespace

TEST(EnvironmentTest, DefaultsToConstantAtmosphereAndConstantGravity)
{
    sim::Environment environment;

    expectConstantAtmosphere(environment.getAtmosphericModel());
    expectConstantGravity(environment.getGravityModel());

    const auto geoid = environment.getGeoidModel();
    ASSERT_TRUE(geoid);
    EXPECT_DOUBLE_EQ(geoid->getGroundLevel(0.0, 0.0),
                           utils::math::Constants::meanEarthRadiusWGS84);
}

TEST(EnvironmentTest, AdvertisesTheAvailableModelNames)
{
    sim::Environment environment;

    const std::vector<std::string> expectedGravity{
        "Constant Gravity",
        "Spherical Gravity"};
    const std::vector<std::string> expectedAtmosphere{
        "Constant Atmosphere",
        "US Standard 1976",
        "Vacuum"};

    EXPECT_EQ(environment.getAvailableGravityModels(), expectedGravity);
    EXPECT_EQ(environment.getAvailableAtmosphereModels(), expectedAtmosphere);
}

TEST(EnvironmentTest, AtmosphereSelectionSwitchesConcreteBehavior)
{
    sim::Environment environment;

    environment.setAtmosphereModel("Vacuum");
    expectVacuumAtmosphere(environment.getAtmosphericModel());

    environment.setAtmosphereModel("US Standard 1976");
    const auto standardAtmosphere = environment.getAtmosphericModel();
    ASSERT_TRUE(standardAtmosphere);
    EXPECT_NEAR(standardAtmosphere->getDensity(0.0), 1.225, 1.225e-3);
    EXPECT_LT(standardAtmosphere->getDensity(10000.0),
                 standardAtmosphere->getDensity(0.0));
    EXPECT_LT(standardAtmosphere->getPressure(10000.0),
                 standardAtmosphere->getPressure(0.0));

    environment.setAtmosphereModel("Constant Atmosphere");
    expectConstantAtmosphere(environment.getAtmosphericModel());
}

TEST(EnvironmentTest, UnknownAtmosphereNameLeavesTheCurrentSelectionUntouched)
{
    sim::Environment environment;
    environment.setAtmosphereModel("Vacuum");

    const auto selected = environment.getAtmosphericModel();
    environment.setAtmosphereModel("Not A Real Atmosphere");

    EXPECT_EQ(environment.getAtmosphericModel(), selected);
    expectVacuumAtmosphere(environment.getAtmosphericModel());
}

TEST(EnvironmentTest, GravitySelectionSwitchesConcreteBehavior)
{
    sim::Environment environment;

    environment.setGravityModel("Spherical Gravity");
    const auto sphericalGravity = environment.getGravityModel();
    ASSERT_TRUE(sphericalGravity);

    const Vector3 padAccel = sphericalGravity->getAccel(0.0, 0.0, 0.0);
    const double earthRadius = environment.getGeoidModel()->getGroundLevel(0.0, 0.0);
    const double expectedSurfaceG =
        static_cast<double>(utils::math::Constants::earthGM) / (earthRadius * earthRadius);

    EXPECT_TRUE(std::isfinite(padAccel.z()));
    EXPECT_DOUBLE_EQ(padAccel.x(), 0.0);
    EXPECT_DOUBLE_EQ(padAccel.y(), 0.0);
    EXPECT_LT(padAccel.z(), 0.0);
    EXPECT_NEAR(-padAccel.z(), expectedSurfaceG, expectedSurfaceG * 1.0e-12);

    const Vector3 highAccel = sphericalGravity->getAccel(0.0, 0.0, 100000.0);
    EXPECT_LT(std::abs(highAccel.z()), std::abs(padAccel.z()));

    environment.setGravityModel("Constant Gravity");
    expectConstantGravity(environment.getGravityModel());
}

TEST(EnvironmentTest, UnknownGravityNameLeavesTheCurrentSelectionUntouched)
{
    sim::Environment environment;
    environment.setGravityModel("Spherical Gravity");

    const auto selected = environment.getGravityModel();
    const Vector3 selectedAccel = selected->getAccel(0.0, 0.0, 0.0);

    environment.setGravityModel("Not A Real Gravity Model");

    EXPECT_EQ(environment.getGravityModel(), selected);
    const Vector3 unchangedAccel = environment.getGravityModel()->getAccel(0.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(unchangedAccel.x(), selectedAccel.x());
    EXPECT_DOUBLE_EQ(unchangedAccel.y(), selectedAccel.y());
    EXPECT_DOUBLE_EQ(unchangedAccel.z(), selectedAccel.z());
}
