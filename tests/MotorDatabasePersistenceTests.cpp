// Tests for model::MotorModelDatabase persistence: the str()<->toEnum() invariants
// the XML format relies on, and a full saveMotorDatabase()/loadMotorDatabase() round trip.

/// \cond
#include <algorithm>
#include <cstdio>      // std::remove
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>
/// \endcond

#include <gtest/gtest.h>

#include "QtRocket.h"
#include "model/MotorModel.h"
#include "utils/Logger.h"
#include "model/MotorModelDatabase.h"
#include "model/ThrustCurveClient.h"

namespace model
{

class MotorModelDatabaseTestAccess
{
public:
   static std::unique_ptr<MotorModelDatabase> makeWithRemoteSource(
      std::unique_ptr<ThrustCurveAPI> thrustCurveApi)
   {
      return std::unique_ptr<MotorModelDatabase>(
         new MotorModelDatabase(std::move(thrustCurveApi)));
   }
};

} // namespace model

namespace
{
using MM = model::MotorModel;

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

std::vector<std::string> summaryNames(const std::vector<model::MotorSummary>& summaries)
{
   std::vector<std::string> names;
   names.reserve(summaries.size());
   for(const auto& summary : summaries)
      names.push_back(summary.commonName);
   return names;
}

const model::MotorSummary* findSummary(const std::vector<model::MotorSummary>& summaries,
                                       const std::string& commonName)
{
   const auto it = std::find_if(summaries.begin(), summaries.end(),
                                [&commonName](const model::MotorSummary& summary)
                                {
                                   return summary.commonName == commonName;
                                });
   return it == summaries.end() ? nullptr : &*it;
}

MM makeMotor(const std::string& commonName,
             const std::string& manufacturer,
             const std::string& impulseClass,
             double diameter,
             double avgThrust,
             double totalImpulse)
{
   std::vector<std::pair<double, double>> thrustData{
      {0.0, 0.0},
      {0.5, avgThrust},
      {1.0, 0.0}};

   MM::MetaData md;
   md.commonName = commonName;
   md.designation = commonName;
   md.manufacturer = MM::MotorManufacturer(MM::MotorManufacturer::toEnum(manufacturer));
   md.impulseClass = impulseClass;
   md.diameter = diameter;
   md.avgThrust = avgThrust;
   md.maxThrust = avgThrust;
   md.totalImpulse = totalImpulse;
   md.burnTime = 1.0;
   md.propWeight = 1.0;
   md.totalWeight = 2.0;

   MM motor;
   motor.addThrustCurve(ThrustCurve(thrustData));
   motor.setMetaData(md);
   return motor;
}

class FakeThrustCurveAPI : public model::ThrustCurveAPI
{
public:
   model::ThrustcurveMetadata metadata;
   std::vector<model::MotorModel> motors;
   model::SearchCriteria lastCriteria;
   int metadataCalls{0};
   int searchCalls{0};

   model::ThrustcurveMetadata getMetadata() override
   {
      ++metadataCalls;
      return metadata;
   }

   std::vector<model::MotorModel> searchMotors(const model::SearchCriteria& c) override
   {
      ++searchCalls;
      lastCriteria = c;
      return motors;
   }
};

const std::string listFixtureXml = R"(<?xml version="1.0" encoding="utf-8"?>
<QtRocketMotorDatabase version="0.1">
  <MotorModels>
    <motor name="G80T">
      <availability>regular</availability>
      <avgThrust>80</avgThrust>
      <burnTime>1</burnTime>
      <certOrg>NAR</certOrg>
      <commonName>G80T</commonName>
      <designation>G80T</designation>
      <diameter>29</diameter>
      <impulseClass>G</impulseClass>
      <infoUrl>https://example.test/g80t</infoUrl>
      <length>124</length>
      <manufacturer>AeroTech</manufacturer>
      <maxThrust>110</maxThrust>
      <motorIdTC>g80t-id</motorIdTC>
      <propType>Blue Thunder</propType>
      <propWeight>1</propWeight>
      <sparky>false</sparky>
      <totalImpulse>120</totalImpulse>
      <totalWeight>2</totalWeight>
      <type>Single Use</type>
      <lastUpdated>2026-06-21</lastUpdated>
      <delays>4,7</delays>
      <thrustCurve>
        <thrust time="0" force="0"/>
        <thrust time="0.5" force="80"/>
        <thrust time="1" force="0"/>
      </thrustCurve>
    </motor>
    <motor name="A8">
      <availability>regular</availability>
      <avgThrust>2.36</avgThrust>
      <burnTime>1</burnTime>
      <certOrg>NAR</certOrg>
      <commonName>A8</commonName>
      <designation>A8</designation>
      <diameter>18</diameter>
      <impulseClass>A</impulseClass>
      <length>70</length>
      <manufacturer>Estes</manufacturer>
      <maxThrust>4.9</maxThrust>
      <propWeight>1</propWeight>
      <sparky>false</sparky>
      <totalImpulse>2.5</totalImpulse>
      <totalWeight>2</totalWeight>
      <type>Single Use</type>
      <delays>3,5</delays>
      <thrustCurve>
        <thrust time="0" force="0"/>
        <thrust time="0.5" force="2.36"/>
        <thrust time="1" force="0"/>
      </thrustCurve>
    </motor>
    <motor name="G61W">
      <availability>regular</availability>
      <avgThrust>61</avgThrust>
      <burnTime>1</burnTime>
      <certOrg>NAR</certOrg>
      <commonName>G61W</commonName>
      <designation>G61W</designation>
      <diameter>38</diameter>
      <impulseClass>G</impulseClass>
      <length>106</length>
      <manufacturer>AeroTech</manufacturer>
      <maxThrust>85</maxThrust>
      <propWeight>1</propWeight>
      <sparky>false</sparky>
      <totalImpulse>125</totalImpulse>
      <totalWeight>2</totalWeight>
      <type>Reload</type>
      <delays>7</delays>
      <thrustCurve>
        <thrust time="0" force="0"/>
        <thrust time="0.5" force="61"/>
        <thrust time="1" force="0"/>
      </thrustCurve>
    </motor>
  </MotorModels>
</QtRocketMotorDatabase>
)";

const std::string edgeFixtureXml = R"(<?xml version="1.0" encoding="utf-8"?>
<QtRocketMotorDatabase version="0.1">
  <MotorModels>
    <note>This node is not a motor and must be ignored.</note>
    <motor name="BareDefaults">
      <commonName>BareDefaults</commonName>
      <burnTime>1</burnTime>
      <propWeight>1</propWeight>
      <totalImpulse>1</totalImpulse>
      <totalWeight>2</totalWeight>
      <delays>4,,7,</delays>
      <thrustCurve>
        <metadata source="ignored"/>
        <thrust time="0" force="0"/>
        <thrust time="0.5" force="2"/>
        <thrust time="1" force="0"/>
      </thrustCurve>
    </motor>
    <motor name="NoCurve">
      <commonName>NoCurve</commonName>
      <burnTime>1</burnTime>
      <propWeight>1</propWeight>
      <totalImpulse>1</totalImpulse>
      <totalWeight>2</totalWeight>
      <delays></delays>
    </motor>
  </MotorModels>
</QtRocketMotorDatabase>
)";

// The XML format stores enums via str() and parses them back via toEnum(); save/load is
// only lossless if that pair round-trips for every value. This guards both directions
// (and pins the Klima/Quest mapping).
TEST(MotorEnumRoundTrip, EveryWrapperRoundTripsThroughItsString)
{
   for(auto a : {MM::AVAILABILITY::REGULAR, MM::AVAILABILITY::OCCASIONAL, MM::AVAILABILITY::OOP})
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

// searchMotors() maps the manufacturer via MotorManufacturer::toEnum, and thrustcurve.org returns
// the full company name (not the short code) in a search result's "manufacturer" field. Guards that
// every enum manufacturer is recognised from that name -- i.e. it works for more than just AeroTech.
TEST(MotorManufacturerMapping, AcceptsThrustcurveOrgNames)
{
   EXPECT_EQ(MM::MotorManufacturer::toEnum("AeroTech"),               MM::MOTORMANUFACTURER::AEROTECH);
   EXPECT_EQ(MM::MotorManufacturer::toEnum("Animal Motor Works"),     MM::MOTORMANUFACTURER::AMW);
   EXPECT_EQ(MM::MotorManufacturer::toEnum("Apogee Components"),      MM::MOTORMANUFACTURER::APOGEE);
   EXPECT_EQ(MM::MotorManufacturer::toEnum("Cesaroni Technology"),    MM::MOTORMANUFACTURER::CESARONI);
   EXPECT_EQ(MM::MotorManufacturer::toEnum("Contrail Rockets"),       MM::MOTORMANUFACTURER::CONTRAIL);
   EXPECT_EQ(MM::MotorManufacturer::toEnum("Estes Industries"),       MM::MOTORMANUFACTURER::ESTES);
   EXPECT_EQ(MM::MotorManufacturer::toEnum("Hypertek"),               MM::MOTORMANUFACTURER::HYPERTEK);
   EXPECT_EQ(MM::MotorManufacturer::toEnum("Raketenmodellbau Klima"), MM::MOTORMANUFACTURER::KLIMA);
   EXPECT_EQ(MM::MotorManufacturer::toEnum("Loki Research"),          MM::MOTORMANUFACTURER::LOKI);
   EXPECT_EQ(MM::MotorManufacturer::toEnum("Quest Aerospace"),        MM::MOTORMANUFACTURER::QUEST);
   // A manufacturer our enum does not model falls back to UNKNOWN rather than misclassifying.
   EXPECT_EQ(MM::MotorManufacturer::toEnum("Gorilla Rocket Motors"),  MM::MOTORMANUFACTURER::UNKNOWN);
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

   model::MotorModelDatabase original;
   ASSERT_GT(original.importRSEFile(rse), 0u);

   original.saveMotorDatabase(tmp);

   model::MotorModelDatabase reloaded;
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

TEST_F(MotorDatabaseRoundTrip, ImportRSEFileReportsNetNewMotorsAndIsIdempotent)
{
   const std::string rse = std::string(QTROCKET_DATA_DIR) + "/Aerotech.rse";

   model::MotorModelDatabase db;
   const std::size_t firstImport = db.importRSEFile(rse);
   ASSERT_GT(firstImport, 0u);
   EXPECT_EQ(db.size(), firstImport);

   const std::size_t secondImport = db.importRSEFile(rse);
   EXPECT_EQ(secondImport, 0u);
   EXPECT_EQ(db.size(), firstImport);

   auto g80t = db.getMotorModel("G80T");
   ASSERT_TRUE(g80t.has_value());
   EXPECT_EQ(g80t->data.commonName, "G80T");
   EXPECT_EQ(g80t->data.manufacturer.str(), "AeroTech");
   EXPECT_EQ(g80t->data.impulseClass, "G");

   EXPECT_FALSE(db.getMotorModel("definitely-not-a-motor").has_value());
}

TEST_F(MotorDatabaseRoundTrip, ImportRASPFileReportsNetNewMotorsAndIsIdempotent)
{
   const auto fixture = tempPath("qtrocket_import_rasp.eng");
   ASSERT_NO_FATAL_FAILURE(writeTextFile(fixture, R"(
1/4A2 13 45 P 0.001 0.002 Estes
  0.25 1.0
  0.50 0.0
;
B6 18 70 4-6 0.006 0.018 Estes
  0.10 12.0
  0.90  0.0
;
)"));

   model::MotorModelDatabase db;
   const std::size_t firstImport = db.importRASPFile(fixture.string());
   ASSERT_EQ(firstImport, 2u);
   EXPECT_EQ(db.size(), firstImport);

   const std::size_t secondImport = db.importRASPFile(fixture.string());
   std::remove(fixture.c_str());
   EXPECT_EQ(secondImport, 0u);
   EXPECT_EQ(db.size(), firstImport);

   auto quarterA = db.getMotorModel("1/4A2");
   ASSERT_TRUE(quarterA.has_value());
   EXPECT_EQ(quarterA->data.commonName, "1/4A2");
   EXPECT_EQ(quarterA->data.impulseClass, "1/4A");
   EXPECT_EQ(quarterA->data.delays, (std::vector<int>{1000}));
   EXPECT_DOUBLE_EQ(quarterA->data.totalWeight, 0.002);

   auto b6 = db.getMotorModel("B6");
   ASSERT_TRUE(b6.has_value());
   EXPECT_EQ(b6->data.impulseClass, "B");
   EXPECT_EQ(b6->data.delays, (std::vector<int>{4, 6}));
}

TEST_F(MotorDatabaseRoundTrip, ListMotorsReturnsSortedSummariesAndAppliesEveryLocalFilter)
{
   const auto fixture = tempPath("qtrocket_motordb_list_filters.qmd");
   ASSERT_NO_FATAL_FAILURE(writeTextFile(fixture, listFixtureXml));

   model::MotorModelDatabase db;
   db.loadMotorDatabase(fixture.string());
   std::remove(fixture.c_str());

   ASSERT_EQ(db.size(), 3u);

   const auto all = db.listMotors();
   EXPECT_EQ(summaryNames(all), (std::vector<std::string>{"A8", "G61W", "G80T"}));

   const model::MotorSummary* g80t = findSummary(all, "G80T");
   ASSERT_NE(g80t, nullptr);
   EXPECT_EQ(g80t->manufacturer, "AeroTech");
   EXPECT_EQ(g80t->impulseClass, "G");
   EXPECT_DOUBLE_EQ(g80t->diameter, 29.0);
   EXPECT_DOUBLE_EQ(g80t->avgThrust, 80.0);
   EXPECT_DOUBLE_EQ(g80t->totalImpulse, 120.0);

   model::MotorQuery query;
   query.manufacturer = "AeroTech";
   EXPECT_EQ(summaryNames(db.listMotors(query)),
             (std::vector<std::string>{"G61W", "G80T"}));

   query = {};
   query.manufacturer = "Estes";
   EXPECT_EQ(summaryNames(db.listMotors(query)),
             (std::vector<std::string>{"A8"}));

   query = {};
   query.impulseClass = "G";
   EXPECT_EQ(summaryNames(db.listMotors(query)),
             (std::vector<std::string>{"G61W", "G80T"}));

   query = {};
   query.diameter = 29.49;
   EXPECT_EQ(summaryNames(db.listMotors(query)),
             (std::vector<std::string>{"G80T"}));

   query = {};
   query.diameter = 29.51;
   EXPECT_TRUE(db.listMotors(query).empty());

   query = {};
   query.nameContains = "80";
   EXPECT_EQ(summaryNames(db.listMotors(query)),
             (std::vector<std::string>{"G80T"}));

   query = {};
   query.manufacturer = "AeroTech";
   query.impulseClass = "G";
   query.diameter = 29.49;
   query.nameContains = "80";
   EXPECT_EQ(summaryNames(db.listMotors(query)),
             (std::vector<std::string>{"G80T"}));

   query.nameContains = "not present";
   EXPECT_TRUE(db.listMotors(query).empty());
}

TEST_F(MotorDatabaseRoundTrip, LoadSkipsNonMotorNodesHonorsDefaultsAndSavesEmptyDelays)
{
   const auto fixture = tempPath("qtrocket_motordb_edge_cases.qmd");
   const auto saved = tempPath("qtrocket_motordb_edge_cases_saved.qmd");
   ASSERT_NO_FATAL_FAILURE(writeTextFile(fixture, edgeFixtureXml));

   model::MotorModelDatabase db;
   db.loadMotorDatabase(fixture.string());
   std::remove(fixture.c_str());

   ASSERT_EQ(db.size(), 2u);
   EXPECT_FALSE(db.getMotorModel("This node is not a motor and must be ignored.").has_value());

   auto bare = db.getMotorModel("BareDefaults");
   ASSERT_TRUE(bare.has_value());
   EXPECT_EQ(bare->data.availability.str(), "regular");
   EXPECT_EQ(bare->data.certOrg.str(), "Uncertified");
   EXPECT_EQ(bare->data.manufacturer.str(), "Unknown");
   EXPECT_EQ(bare->data.type.str(), "Single Use");
   EXPECT_FALSE(bare->data.sparky);
   EXPECT_DOUBLE_EQ(bare->data.avgThrust, 0.0);
   EXPECT_DOUBLE_EQ(bare->data.diameter, 0.0);
   EXPECT_EQ(bare->data.delays, (std::vector<int>{4, 7}));

   const auto bareThrust = bare->getThrustCurve().getThrustCurveData();
   ASSERT_EQ(bareThrust.size(), 3u);
   EXPECT_DOUBLE_EQ(bareThrust[1].first, 0.5);
   EXPECT_DOUBLE_EQ(bareThrust[1].second, 2.0);

   auto noCurve = db.getMotorModel("NoCurve");
   ASSERT_TRUE(noCurve.has_value());
   EXPECT_TRUE(noCurve->data.delays.empty());
   const auto noCurveThrust = noCurve->getThrustCurve().getThrustCurveData();
   ASSERT_EQ(noCurveThrust.size(), 1u);
   EXPECT_DOUBLE_EQ(noCurveThrust[0].first, 0.0);
   EXPECT_DOUBLE_EQ(noCurveThrust[0].second, 0.0);

   db.saveMotorDatabase(saved.string());

   model::MotorModelDatabase reloaded;
   reloaded.loadMotorDatabase(saved.string());
   std::remove(saved.c_str());

   ASSERT_EQ(reloaded.size(), 2u);
   auto reloadedNoCurve = reloaded.getMotorModel("NoCurve");
   ASSERT_TRUE(reloadedNoCurve.has_value());
   EXPECT_TRUE(reloadedNoCurve->data.delays.empty());
}

TEST(MotorDatabaseOnlineSource, FacetsExposeSourceFilterValuesWithoutFullNames)
{
   auto fake = std::make_unique<FakeThrustCurveAPI>();
   auto* fakePtr = fake.get();
   fakePtr->metadata.manufacturers = {
      {"AeroTech", "AeroTech"},
      {"Estes", "Estes Industries"}};
   fakePtr->metadata.diameters = {18.0, 24.0, 29.0};
   fakePtr->metadata.impulseClasses = {"A", "B", "G"};

   auto db = model::MotorModelDatabaseTestAccess::makeWithRemoteSource(std::move(fake));

   const model::MotorSearchFacets facets = db->getOnlineSearchFacets();

   EXPECT_EQ(fakePtr->metadataCalls, 1);
   EXPECT_EQ(facets.manufacturers, (std::vector<std::string>{"AeroTech", "Estes"}));
   EXPECT_EQ(facets.diameters, (std::vector<double>{18.0, 24.0, 29.0}));
   EXPECT_EQ(facets.impulseClasses, (std::vector<std::string>{"A", "B", "G"}));
}

TEST(MotorDatabaseOnlineSource, SearchOnlineBuildsSourceCriteriaAndMergesReturnedMotors)
{
   auto fake = std::make_unique<FakeThrustCurveAPI>();
   auto* fakePtr = fake.get();
   fakePtr->motors.push_back(makeMotor("G80T", "AeroTech", "G", 29.0, 80.0, 120.0));

   auto db = model::MotorModelDatabaseTestAccess::makeWithRemoteSource(std::move(fake));

   model::MotorQuery query;
   query.manufacturer = "AeroTech";
   query.impulseClass = "G";
   query.diameter = 29.0;
   query.nameContains = "local-only";

   const std::vector<model::MotorSummary> found = db->searchOnline(query);

   ASSERT_EQ(fakePtr->searchCalls, 1);
   ASSERT_EQ(fakePtr->lastCriteria.criteria.size(), 3u);
   EXPECT_EQ(fakePtr->lastCriteria.criteria.at("manufacturer"), "AeroTech");
   EXPECT_EQ(fakePtr->lastCriteria.criteria.at("impulseClass"), "G");
   EXPECT_EQ(fakePtr->lastCriteria.criteria.at("diameter"), "29");
   EXPECT_FALSE(fakePtr->lastCriteria.criteria.contains("nameContains"));

   ASSERT_EQ(found.size(), 1u);
   EXPECT_EQ(found[0].commonName, "G80T");
   EXPECT_EQ(found[0].manufacturer, "AeroTech");
   EXPECT_DOUBLE_EQ(found[0].diameter, 29.0);
   EXPECT_DOUBLE_EQ(found[0].totalImpulse, 120.0);

   auto stored = db->getMotorModel("G80T");
   ASSERT_TRUE(stored.has_value());
   EXPECT_DOUBLE_EQ(stored->data.avgThrust, 80.0);

   model::MotorQuery localQuery;
   localQuery.nameContains = "80";
   EXPECT_EQ(summaryNames(db->listMotors(localQuery)),
             (std::vector<std::string>{"G80T"}));
}

} // namespace
