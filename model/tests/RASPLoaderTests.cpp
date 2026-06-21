/// \cond
// C headers
#include <cstdio>
// C++ headers
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
// 3rd party headers
#include <gtest/gtest.h>
/// \endcond

// qtrocket headers
#include "model/MotorModel.h"
#include "model/RASPLoader.h"
#include "model/RSEDatabaseLoader.h"
#include "utils/Logger.h"

namespace
{

std::filesystem::path tempPath(const std::string& name)
{
   return std::filesystem::temp_directory_path() / name;
}

void writeTextFile(const std::filesystem::path& path, const std::string& text)
{
   std::ofstream file(path);
   ASSERT_TRUE(file.is_open());
   file << text;
   ASSERT_TRUE(file.good());
}

class RASPLoaderTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);
   }
};

} // namespace

TEST_F(RASPLoaderTest, ParsesSpecStyleMotorAndNormalizesImplicitStartPoint)
{
   const auto path = tempPath("qtrocket_rasp_single.eng");
   ASSERT_NO_FATAL_FAILURE(writeTextFile(path, R"(
; A compact RASP-style motor definition.
D12 24 70 0-3-5-7 0.0211 0.0438 Estes
   0.50 10.00
   1.00  0.00
;
)"));

   model::RASPLoader loader(path.string());
   std::remove(path.c_str());

   ASSERT_EQ(loader.getMotors().size(), 1u);
   const model::MotorModel motor = loader.getMotorModelByName("D12");

   EXPECT_EQ(motor.data.commonName, "D12");
   EXPECT_EQ(motor.data.designation, "D12");
   EXPECT_EQ(motor.data.manufacturer.str(), "Estes");
   EXPECT_EQ(motor.data.impulseClass, "D");
   EXPECT_EQ(motor.data.delays, (std::vector<int>{0, 3, 5, 7}));
   EXPECT_DOUBLE_EQ(motor.data.diameter, 24.0);
   EXPECT_DOUBLE_EQ(motor.data.length, 70.0);
   EXPECT_DOUBLE_EQ(motor.data.propWeight, 0.0211);
   EXPECT_DOUBLE_EQ(motor.data.totalWeight, 0.0438);
   EXPECT_DOUBLE_EQ(motor.data.burnTime, 1.0);
   EXPECT_DOUBLE_EQ(motor.data.maxThrust, 10.0);
   EXPECT_DOUBLE_EQ(motor.data.totalImpulse, 5.0);
   EXPECT_DOUBLE_EQ(motor.data.avgThrust, 5.0);

   const auto thrust = motor.getThrustCurve().getThrustCurveData();
   ASSERT_EQ(thrust.size(), 3u);
   EXPECT_DOUBLE_EQ(thrust[0].first, 0.0);
   EXPECT_DOUBLE_EQ(thrust[0].second, 0.0);
   EXPECT_DOUBLE_EQ(motor.getThrustCurve().getMaxTime(), 1.0);
}

TEST_F(RASPLoaderTest, ParsesMultipleMotorsAndPluggedFractionalImpulseClass)
{
   const auto path = tempPath("qtrocket_rasp_multi.eng");
   ASSERT_NO_FATAL_FAILURE(writeTextFile(path, R"(
1/2A3 13 45 P 0.001 0.002 Estes
  0.25 2.0
  0.50 0.0
;
G80T 29 124 4-7 0.060 0.100 AeroTech
  0.10 160.0
  1.90   0.0
;
)"));

   model::RASPLoader loader(path.string());
   std::remove(path.c_str());

   ASSERT_EQ(loader.getMotors().size(), 2u);
   const model::MotorModel halfA = loader.getMotorModelByName("1/2A3");
   EXPECT_EQ(halfA.data.impulseClass, "1/2A");
   EXPECT_EQ(halfA.data.delays, (std::vector<int>{1000}));
   EXPECT_DOUBLE_EQ(halfA.data.burnTime, 0.5);

   const model::MotorModel g80 = loader.getMotorModelByName("G80T");
   EXPECT_EQ(g80.data.impulseClass, "G");
   EXPECT_EQ(g80.data.delays, (std::vector<int>{4, 7}));
   EXPECT_DOUBLE_EQ(g80.data.maxThrust, 160.0);
}

TEST_F(RASPLoaderTest, RejectsMissingFinalZeroThrustSample)
{
   const auto path = tempPath("qtrocket_rasp_missing_zero.eng");
   ASSERT_NO_FATAL_FAILURE(writeTextFile(path, R"(
A8 18 70 3 0.003 0.016 Estes
  0.10 5.0
)"));

   EXPECT_THROW((void)model::RASPLoader(path.string()), std::runtime_error);
   std::remove(path.c_str());
}

TEST_F(RASPLoaderTest, RejectsZeroThrustBeforeMoreSampleData)
{
   const auto path = tempPath("qtrocket_rasp_early_zero.eng");
   ASSERT_NO_FATAL_FAILURE(writeTextFile(path, R"(
A8 18 70 3 0.003 0.016 Estes
  0.10 0.0
  0.20 5.0
  0.30 0.0
)"));

   EXPECT_THROW((void)model::RASPLoader(path.string()), std::runtime_error);
   std::remove(path.c_str());
}

TEST_F(RASPLoaderTest, RejectsInvalidDelayToken)
{
   const auto path = tempPath("qtrocket_rasp_bad_delay.eng");
   ASSERT_NO_FATAL_FAILURE(writeTextFile(path, R"(
A8 18 70 PORK 0.003 0.016 Estes
  0.10 5.0
  0.30 0.0
)"));

   EXPECT_THROW((void)model::RASPLoader(path.string()), std::runtime_error);
   std::remove(path.c_str());
}

TEST_F(RASPLoaderTest, RSELoaderUsesSharedImpulseClassParser)
{
   const auto path = tempPath("qtrocket_rse_fractional.rse");
   ASSERT_NO_FATAL_FAILURE(writeTextFile(path, R"(
<engine-database>
  <engine-list>
    <engine mfg="Estes" code="1/2A3" Type="single-use" dia="13" len="45"
            initWt="2" propWt="1" delays="3" avgThrust="2" peakThrust="4"
            Itot="0.6" burn-time="0.3">
      <data>
        <eng-data t="0" f="0"/>
        <eng-data t="0.15" f="4"/>
        <eng-data t="0.3" f="0"/>
      </data>
    </engine>
  </engine-list>
</engine-database>
)"));

   model::RSEDatabaseLoader loader(path.string());
   std::remove(path.c_str());

   ASSERT_EQ(loader.getMotors().size(), 1u);
   EXPECT_EQ(loader.getMotors()[0].data.impulseClass, "1/2A");
}
