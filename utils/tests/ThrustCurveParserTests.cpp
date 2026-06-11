// Tests for the pure thrustcurve.org response parsers in utils/ThrustCurveAPI.h,
// using canned JSON shaped like real API responses (see the OpenAPI spec at
// https://www.thrustcurve.org/api/v1/swagger.json). No network involved.

/// \cond
#include <optional>
#include <string>
/// \endcond

#include <gtest/gtest.h>

#include "model/MotorModel.h"
#include "utils/Logger.h"
#include "utils/ThrustCurveAPI.h"

namespace
{
using MM = model::MotorModel;

class ThrustCurveParserTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      // The malformed-input tests intentionally provoke parse errors; keep
      // them out of the test output.
      utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);
   }
};

// --- download.json ---------------------------------------------------------

const std::string rockSimEntry = R"({
   "motorId": "m1", "simfileId": "s1", "format": "RockSim", "source": "user",
   "samples": [{"time": 0, "thrust": 0}, {"time": 0.1, "thrust": 5.0}, {"time": 0.25, "thrust": 0}]
})";

const std::string raspEntry = R"({
   "motorId": "m1", "simfileId": "s2", "format": "RASP", "source": "user",
   "samples": [{"time": 0.016, "thrust": 0.243}, {"time": 0.25, "thrust": 0}]
})";

TEST_F(ThrustCurveParserTest, DownloadPrefersRaspRegardlessOfOrder)
{
   // Real responses list a motor's simfiles in arbitrary order.
   for(const std::string& json : {"{\"results\": [" + rockSimEntry + "," + raspEntry + "]}",
                                  "{\"results\": [" + raspEntry + "," + rockSimEntry + "]}"})
   {
      auto samples = utils::parseDownloadResponse(json);
      ASSERT_TRUE(samples.has_value());
      ASSERT_EQ(samples->size(), 2u) << "expected exactly the RASP file's samples";
      EXPECT_DOUBLE_EQ((*samples)[0].first, 0.016);
      EXPECT_DOUBLE_EQ((*samples)[0].second, 0.243);
   }
}

TEST_F(ThrustCurveParserTest, DownloadNeverConcatenatesSimfiles)
{
   const std::string json = "{\"results\": [" + raspEntry + "," + rockSimEntry + "]}";
   auto samples = utils::parseDownloadResponse(json);
   ASSERT_TRUE(samples.has_value());
   // Concatenation would yield 5 samples with time jumping back to 0 mid-curve.
   ASSERT_EQ(samples->size(), 2u);
   for(size_t i = 1; i < samples->size(); ++i)
      EXPECT_GT((*samples)[i].first, (*samples)[i - 1].first) << "time must be increasing";
}

TEST_F(ThrustCurveParserTest, DownloadFallsBackToNonRaspWhenNoRasp)
{
   const std::string json = "{\"results\": [" + rockSimEntry + "]}";
   auto samples = utils::parseDownloadResponse(json);
   ASSERT_TRUE(samples.has_value());
   EXPECT_EQ(samples->size(), 3u);
}

TEST_F(ThrustCurveParserTest, DownloadEmptyResultsYieldsEmptySamples)
{
   // download.json returns an empty results list for motors with no data files.
   auto samples = utils::parseDownloadResponse(R"({"results": []})");
   ASSERT_TRUE(samples.has_value());
   EXPECT_TRUE(samples->empty());
}

// --- metadata.json ---------------------------------------------------------

TEST_F(ThrustCurveParserTest, MetadataTypesParsedAsPlainStrings)
{
   // "types" elements are bare strings, unlike the {name, abbrev} objects in
   // manufacturers/certOrgs.
   const std::string json = R"({
      "manufacturers": [{"name": "Estes Industries", "abbrev": "Estes"}],
      "certOrgs": [{"name": "National Association of Rocketry", "abbrev": "NAR"}],
      "types": ["SU", "reload", "hybrid"],
      "diameters": [13, 18, 24],
      "impulseClasses": ["A", "B", "C"]
   })";
   auto meta = utils::parseMetadataResponse(json);
   ASSERT_TRUE(meta.has_value());

   ASSERT_EQ(meta->types.size(), 3u);
   EXPECT_EQ(meta->types[0].type, MM::MOTORTYPE::SU);
   EXPECT_EQ(meta->types[1].type, MM::MOTORTYPE::RELOAD);
   EXPECT_EQ(meta->types[2].type, MM::MOTORTYPE::HYBRID);

   ASSERT_EQ(meta->certOrgs.size(), 1u);
   EXPECT_EQ(meta->certOrgs[0].org, MM::CERTORG::NAR);
   EXPECT_EQ(meta->manufacturers.at("Estes"), "Estes Industries");
   EXPECT_EQ(meta->diameters.size(), 3u);
   EXPECT_EQ(meta->impulseClasses.size(), 3u);
}

// --- search.json -----------------------------------------------------------

namespace
{
std::string searchResult(const std::string& commonName, const std::string& availability)
{
   return R"({
      "motorId": "id-)" + commonName + R"(", "manufacturer": "Estes Industries",
      "manufacturerAbbrev": "Estes", "designation": ")" + commonName + R"(",
      "commonName": ")" + commonName + R"(", "impulseClass": "A", "diameter": 13,
      "length": 45, "type": "SU", "avgThrustN": 2.36, "maxThrustN": 4.95,
      "totImpulseNs": 0.59, "burnTimeS": 0.25, "totalWeightG": 6.1,
      "propWeightG": 0.83, "propInfo": "black powder", "sparky": false,
      "availability": ")" + availability + R"("
   })";
}
}

TEST_F(ThrustCurveParserTest, SearchReportsMatchesBeyondResults)
{
   // The server caps results (default 20) but always reports the full match
   // count, which is how callers detect truncation.
   const std::string json = R"({"matches": 23, "results": [)" +
      searchResult("A8", "regular") + "," + searchResult("B6", "regular") + "]}";
   auto response = utils::parseSearchResponse(json);
   ASSERT_TRUE(response.has_value());
   EXPECT_EQ(response->matches, 23);
   ASSERT_EQ(response->motors.size(), 2u);
   EXPECT_EQ(response->motors[0].commonName, "A8");
   EXPECT_EQ(response->motors[0].motorIdTC, "id-A8");
   EXPECT_DOUBLE_EQ(response->motors[0].avgThrust, 2.36);
   EXPECT_EQ(response->motors[0].manufacturer.manufacturer, MM::MOTORMANUFACTURER::ESTES);
}

TEST_F(ThrustCurveParserTest, SearchMapsAllThreeAvailabilityValues)
{
   const std::string json = R"({"matches": 3, "results": [)" +
      searchResult("A8", "regular") + "," +
      searchResult("B6", "occasional") + "," +
      searchResult("C6", "OOP") + "]}";
   auto response = utils::parseSearchResponse(json);
   ASSERT_TRUE(response.has_value());
   ASSERT_EQ(response->motors.size(), 3u);
   EXPECT_EQ(response->motors[0].availability.availability, MM::AVAILABILITY::REGULAR);
   EXPECT_EQ(response->motors[1].availability.availability, MM::AVAILABILITY::OCCASIONAL);
   EXPECT_EQ(response->motors[2].availability.availability, MM::AVAILABILITY::OOP);
}

// --- all parsers -----------------------------------------------------------

TEST_F(ThrustCurveParserTest, MalformedJsonReturnsNullopt)
{
   const std::string garbage = "this is not json";
   EXPECT_FALSE(utils::parseSearchResponse(garbage).has_value());
   EXPECT_FALSE(utils::parseMetadataResponse(garbage).has_value());
   EXPECT_FALSE(utils::parseDownloadResponse(garbage).has_value());
}

TEST_F(ThrustCurveParserTest, ErrorFieldYieldsEmptyButValidResults)
{
   const std::string json = R"({"error": "invalid query", "results": []})";
   auto search = utils::parseSearchResponse(json);
   ASSERT_TRUE(search.has_value());
   EXPECT_TRUE(search->motors.empty());

   auto download = utils::parseDownloadResponse(json);
   ASSERT_TRUE(download.has_value());
   EXPECT_TRUE(download->empty());
}

} // namespace
