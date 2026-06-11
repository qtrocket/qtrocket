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
