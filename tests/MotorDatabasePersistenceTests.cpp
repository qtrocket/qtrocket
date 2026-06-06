// Tests for utils::MotorModelDatabase persistence: the str()<->toEnum() invariants
// the XML format relies on, and a full saveMotorDatabase()/loadMotorDatabase() round trip.

/// \cond
#include <cstdio>      // std::remove
#include <filesystem>
#include <optional>
#include <string>
/// \endcond

#include <gtest/gtest.h>

#include "QtRocket.h"
#include "model/MotorModel.h"
#include "utils/Logger.h"
#include "utils/MotorModelDatabase.h"

namespace
{
using MM = model::MotorModel;

// The XML format stores enums via str() and parses them back via toEnum(); save/load is
// only lossless if that pair round-trips for every value. This guards both directions
// (and pins the Klima/Quest mapping).
TEST(MotorEnumRoundTrip, EveryWrapperRoundTripsThroughItsString)
{
   for(auto a : {MM::AVAILABILITY::REGULAR, MM::AVAILABILITY::OOP})
      EXPECT_EQ(MM::MotorAvailability::toEnum(MM::MotorAvailability(a).str()), a);

   for(auto c : {MM::CERTORG::AMRS, MM::CERTORG::CAR, MM::CERTORG::NAR,
                 MM::CERTORG::TRA, MM::CERTORG::UNC, MM::CERTORG::UNK})
      EXPECT_EQ(MM::CertOrg::toEnum(MM::CertOrg(c).str()), c);

   for(auto t : {MM::MOTORTYPE::SU, MM::MOTORTYPE::RELOAD, MM::MOTORTYPE::HYBRID})
      EXPECT_EQ(MM::MotorType::toEnum(MM::MotorType(t).str()), t);

   for(auto m : {MM::MOTORMANUFACTURER::AEROTECH, MM::MOTORMANUFACTURER::AMW,
                 MM::MOTORMANUFACTURER::APOGEE,   MM::MOTORMANUFACTURER::CESARONI,
                 MM::MOTORMANUFACTURER::CONTRAIL, MM::MOTORMANUFACTURER::ESTES,
                 MM::MOTORMANUFACTURER::HYPERTEK, MM::MOTORMANUFACTURER::KLIMA,
                 MM::MOTORMANUFACTURER::LOKI,     MM::MOTORMANUFACTURER::QUEST,
                 MM::MOTORMANUFACTURER::UNKNOWN})
      EXPECT_EQ(MM::MotorManufacturer::toEnum(MM::MotorManufacturer(m).str()), m);
}

class MotorDatabaseRoundTrip : public ::testing::Test
{
protected:
   void SetUp() override
   {
      // addMotorModel() logs through the singleton's logger; keep the singleton alive
      // and quiet so a 249-motor import does not flood the test output.
      utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);
      QtRocket::getInstance();
   }
};

TEST_F(MotorDatabaseRoundTrip, SaveThenLoadReproducesTheDatabase)
{
   const std::string rse = std::string(QTROCKET_DATA_DIR) + "/Aerotech.rse";
   const std::string tmp =
      (std::filesystem::temp_directory_path() / "qtrocket_motordb_roundtrip.qmd").string();

   utils::MotorModelDatabase original;
   ASSERT_GT(original.importRSEFile(rse), 0u);

   original.saveMotorDatabase(tmp);

   utils::MotorModelDatabase reloaded;
   reloaded.loadMotorDatabase(tmp);
   std::remove(tmp.c_str());

   EXPECT_EQ(reloaded.size(), original.size());

   // Spot-check a known motor field-by-field. boost::property_tree writes doubles at
   // max_digits10, so the round trip is exact.
   auto a = original.getMotorModel("G80T");
   auto b = reloaded.getMotorModel("G80T");
   ASSERT_TRUE(a.has_value());
   ASSERT_TRUE(b.has_value());

   EXPECT_EQ(b->data.commonName,       a->data.commonName);
   EXPECT_EQ(b->data.manufacturer.str(), a->data.manufacturer.str());
   EXPECT_EQ(b->data.type.str(),       a->data.type.str());
   EXPECT_EQ(b->data.certOrg.str(),    a->data.certOrg.str());
   EXPECT_EQ(b->data.impulseClass,     a->data.impulseClass);
   EXPECT_EQ(b->data.delays,           a->data.delays);
   EXPECT_DOUBLE_EQ(b->data.avgThrust,    a->data.avgThrust);
   EXPECT_DOUBLE_EQ(b->data.totalImpulse, a->data.totalImpulse);
   EXPECT_DOUBLE_EQ(b->data.diameter,     a->data.diameter);
   EXPECT_DOUBLE_EQ(b->data.burnTime,     a->data.burnTime);
   EXPECT_DOUBLE_EQ(b->data.propWeight,   a->data.propWeight);
   EXPECT_DOUBLE_EQ(b->data.totalWeight,  a->data.totalWeight);

   // Thrust curve must survive: it drives the recomputed mass curve on load.
   const auto& ta = a->getThrustCurve().getThrustCurveData();
   const auto& tb = b->getThrustCurve().getThrustCurveData();
   ASSERT_EQ(tb.size(), ta.size());
   for(std::size_t i = 0; i < ta.size(); ++i)
   {
      EXPECT_DOUBLE_EQ(tb[i].first,  ta[i].first);
      EXPECT_DOUBLE_EQ(tb[i].second, ta[i].second);
   }

   // And the derived mass curve matches (only possible because propWeight survived).
   EXPECT_DOUBLE_EQ(b->getMass(0.0), a->getMass(0.0));
}

} // namespace
