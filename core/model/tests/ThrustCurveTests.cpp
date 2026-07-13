#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include "model/ThrustCurve.h"

TEST(ThrustCurveTests, EmptySampleVectorBehavesLikeDefaultCtor)
{
    std::vector<std::pair<double, double>> empty;
    ThrustCurve tc(empty);

    EXPECT_DOUBLE_EQ(tc.getMaxTime(), 0.0);
    EXPECT_DOUBLE_EQ(tc.getThrust(0.0), 0.0);
    EXPECT_DOUBLE_EQ(tc.getThrust(1.0), 0.0);
    ASSERT_EQ(tc.getThrustCurveData().size(), 1u);
}

TEST(ThrustCurveTests, MaxTimeIsLastSampleTime)
{
    std::vector<std::pair<double, double>> samples{
        {0.0, 0.0}, {0.5, 10.0}, {1.5, 4.0}, {2.0, 0.0}};
    ThrustCurve tc(samples);

    EXPECT_DOUBLE_EQ(tc.getMaxTime(), 2.0);
}

TEST(ThrustCurveTests, InterpolatesLinearlyBetweenSamples)
{
    std::vector<std::pair<double, double>> samples{
        {0.0, 0.0}, {1.0, 10.0}, {2.0, 0.0}};
    ThrustCurve tc(samples);

    EXPECT_DOUBLE_EQ(tc.getThrust(1.0), 10.0);   // exact sample
    EXPECT_DOUBLE_EQ(tc.getThrust(0.5), 5.0);    // midpoint of rising edge
    EXPECT_DOUBLE_EQ(tc.getThrust(1.75), 2.5);   // falling edge
    EXPECT_DOUBLE_EQ(tc.getThrust(-0.1), 0.0);   // before ignition
    EXPECT_DOUBLE_EQ(tc.getThrust(2.5), 0.0);    // after burnout
}

// Regression: a curve whose FIRST sample is at t > 0 (no (0,0) origin -- the form thrustcurve.org and
// RSE curves arrive in) must ramp from the implicit origin for queries between ignition and that first
// sample, NOT walk off the front of the sample vector. The old std::prev(begin()) read here was an
// out-of-bounds heap read; its garbage made the early-burn thrust -- and therefore a whole flight --
// depend on allocation layout (an order-dependent, non-reproducible apogee).
TEST(ThrustCurveTests, RampsFromOriginBeforeFirstSampleWithoutOutOfBoundsRead)
{
    std::vector<std::pair<double, double>> samples{
        {0.05, 10.0}, {0.10, 20.0}, {0.20, 0.0}}; // first sample at t = 0.05, NOT 0
    ThrustCurve tc(samples);

    EXPECT_DOUBLE_EQ(tc.getThrust(0.0), 0.0);     // ignition: thrust ramps up from the (0,0) origin
    EXPECT_DOUBLE_EQ(tc.getThrust(0.025), 5.0);   // halfway from the origin to the 10 N first sample
    EXPECT_DOUBLE_EQ(tc.getThrust(0.05), 10.0);   // exact first sample
    EXPECT_DOUBLE_EQ(tc.getThrust(0.075), 15.0);  // between the first and second samples
}
