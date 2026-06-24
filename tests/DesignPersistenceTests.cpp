// End-to-end tests for model::DesignSerializer: a full design save/load round trip (part-tree mass,
// CG and structure, the reference-area override, and motor-by-name resolution), the multi-child
// add_child guarantee, motor-absent tolerance, and rejection of an unsupported file version.

/// \cond
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
/// \endcond

#include <gtest/gtest.h>

#include "model/DesignSerializer.h"
#include "model/RocketModel.h"
#include "model/MotorModelDatabase.h"
#include "model/parts/Parts.h"
#include "utils/Logger.h"

namespace
{
std::string tempFile(const std::string& tag)
{
   return (std::filesystem::temp_directory_path() / ("qtrocket_design_" + tag + ".qrd")).string();
}

std::shared_ptr<model::part::BodyTube> bodyTube(const std::string& name)
{
   return std::make_shared<model::part::BodyTube>(name, 0.0, 0.019, 0.20, 680.0);
}

std::string readFile(const std::string& path)
{
   std::ifstream in(path);
   std::stringstream ss;
   ss << in.rdbuf();
   return ss.str();
}

void writeTextFile(const std::string& path, const std::string& contents)
{
   std::ofstream f(path);
   f << contents;
}

class DesignRoundTrip : public ::testing::Test
{
protected:
   // Quiet the per-motor import logging and the motor-absent warning so the suite output stays clean.
   void SetUp() override { utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_); }
};
} // namespace

TEST_F(DesignRoundTrip, GeometryRoundTripsMassCgStructureAndMultiChild)
{
   // nose -> body, and body has TWO children (fins + coupler): a linear chain would not exercise
   // add_child for repeated siblings (the put-vs-add_child trap), so the second child guards it. The
   // cone's non-central CM is exercised by the real attach offsets.
   model::RocketModel r;
   r.setRoot(std::make_shared<model::part::ConicalNoseCone>("Nose", 0.019, 0.10, 0.0, 2700.0, true));
   const auto noseId = r.getTopPart()->getId();

   auto body = bodyTube("Body");
   const auto bodyId = body->getId();
   ASSERT_TRUE(r.addPart(noseId, body, Vector3{0.0, 0.0, -0.13}));
   ASSERT_TRUE(r.addPart(bodyId,
      std::make_shared<model::part::FinSet>("Fins", 3, 0.10, 0.05, 0.05, 0.04, 0.003, 0.019, 600.0),
      Vector3{0.0, 0.0, -0.08}));
   ASSERT_TRUE(r.addPart(bodyId,
      std::make_shared<model::part::BodyTube>("Coupler", 0.015, 0.019, 0.03, 900.0),
      Vector3{0.0, 0.0, -0.10}));

   const std::string tmp = tempFile("geom");
   model::DesignSerializer::save(r, tmp);

   model::MotorModelDatabase motors; // this design has no motor
   model::RocketModel r2;
   model::DesignSerializer::load(r2, motors, tmp);
   std::filesystem::remove(tmp);

   // Structure: nose root -> one child (body); body -> two children (both survived: add_child, not put).
   ASSERT_NE(r2.getTopPart(), nullptr);
   EXPECT_EQ(r2.getTopPart()->typeName(), "NoseCone");
   ASSERT_EQ(r2.getTopPart()->getChildParts().size(), 1u);
   const auto& body2 = r2.getTopPart()->getChildParts()[0].first;
   EXPECT_EQ(body2->typeName(), "BodyTube");
   EXPECT_EQ(body2->getName(), "Body");
   ASSERT_EQ(body2->getChildParts().size(), 2u);

   // Mass exact; CG via EXPECT_NEAR (reload may re-sum children in a different ULP order). The CG
   // match is what proves the placement (the body's StationLink) round-tripped faithfully -- the
   // absolute offset is no longer stored, it is derived by the resolver.
   EXPECT_DOUBLE_EQ(r2.getMass(0.0), r.getMass(0.0));
   const Vector3 cg = r.getTopPart()->getCompositeCm(0.0);
   const Vector3 cg2 = r2.getTopPart()->getCompositeCm(0.0);
   for(int i = 0; i < 3; ++i) { EXPECT_NEAR(cg2(i), cg(i), 1e-9); }
}

TEST_F(DesignRoundTrip, ReferenceAreaOverrideAndDragRoundTrip)
{
   const std::string tmp = tempFile("refarea");
   {
      model::RocketModel r;
      r.setRoot(bodyTube("Body"));
      r.setReferenceArea(0.0421); // manual override on
      r.setDragCoefficient(0.55);
      ASSERT_TRUE(r.isReferenceAreaOverridden());
      model::DesignSerializer::save(r, tmp);
   }
   model::MotorModelDatabase motors;
   model::RocketModel r2;
   model::DesignSerializer::load(r2, motors, tmp);
   std::filesystem::remove(tmp);

   EXPECT_TRUE(r2.isReferenceAreaOverridden());
   EXPECT_DOUBLE_EQ(r2.getReferenceArea(), 0.0421);
   EXPECT_DOUBLE_EQ(r2.getDragCoefficient(), 0.55);
}

TEST_F(DesignRoundTrip, NonOverriddenReferenceAreaStaysUnoverriddenOnLoad)
{
   const std::string tmp = tempFile("refarea2");
   {
      model::RocketModel r; // reference area at its default, NOT overridden
      r.setRoot(bodyTube("Body"));
      ASSERT_FALSE(r.isReferenceAreaOverridden());
      model::DesignSerializer::save(r, tmp);
   }
   model::MotorModelDatabase motors;
   model::RocketModel r2;
   model::DesignSerializer::load(r2, motors, tmp);
   std::filesystem::remove(tmp);

   EXPECT_FALSE(r2.isReferenceAreaOverridden());
}

TEST_F(DesignRoundTrip, MotorByNameRoundTripsAndPreservesMassCurve)
{
   model::MotorModelDatabase motors;
   ASSERT_GT(motors.importRSEFile(std::string(QTROCKET_DATA_DIR) + "/Aerotech.rse"), 0u);
   const auto g80 = motors.getMotorModel("G80T");
   ASSERT_TRUE(g80.has_value());

   const std::string tmp = tempFile("motor");
   model::RocketModel r;
   r.setRoot(bodyTube("Body"));
   r.setMotorModel(*g80);
   ASSERT_TRUE(r.isMotorSet());
   r.launch(); // ignite so getMass(t) follows the burn curve, as a real flight reads it
   model::DesignSerializer::save(r, tmp);

   model::RocketModel r2;
   model::DesignSerializer::load(r2, motors, tmp);
   std::filesystem::remove(tmp);
   ASSERT_TRUE(r2.isMotorSet());
   r2.launch();

   // Composite mass(t) -- airframe + the motor's burn-time mass -- matches at ignition, mid-burn, and
   // post-burnout. Equal only because the motor (and its derived mass curve) round-tripped by name.
   for(double t : {0.0, 0.5, 5.0})
   {
      EXPECT_DOUBLE_EQ(r2.getMass(t), r.getMass(t));
   }
}

TEST_F(DesignRoundTrip, MotorAbsentInDatabaseLoadsGeometryWithoutTheMotor)
{
   model::MotorModelDatabase full;
   ASSERT_GT(full.importRSEFile(std::string(QTROCKET_DATA_DIR) + "/Aerotech.rse"), 0u);
   const auto g80 = full.getMotorModel("G80T");
   ASSERT_TRUE(g80.has_value());

   const std::string tmp = tempFile("motorabsent");
   model::RocketModel r;
   r.setRoot(bodyTube("Body"));
   r.setMotorModel(*g80);
   model::DesignSerializer::save(r, tmp);

   model::MotorModelDatabase empty; // does NOT contain G80T
   model::RocketModel r2;
   model::DesignSerializer::load(r2, empty, tmp); // logs a warning, loads geometry only
   std::filesystem::remove(tmp);

   EXPECT_FALSE(r2.isMotorSet());
   EXPECT_EQ(r2.getTopPart()->typeName(), "BodyTube"); // geometry still loaded
}

TEST_F(DesignRoundTrip, UnsupportedMajorVersionIsRejectedAndLeavesRocketUntouched)
{
   const std::string tmp = tempFile("badversion");
   {
      std::ofstream f(tmp);
      f << "<QtRocketDesign version=\"9.0\">"
        << "<part type=\"BodyTube\" name=\"B\">"
        << "<params outerRadius=\"0.019\" length=\"0.2\" density=\"680\"/>"
        << "<offset x=\"0\" y=\"0\" z=\"0\"/><children/></part></QtRocketDesign>";
   }
   model::MotorModelDatabase motors;
   model::RocketModel r; // boot placeholder
   EXPECT_THROW(model::DesignSerializer::load(r, motors, tmp), std::exception);
   std::filesystem::remove(tmp);

   // The version check happens before setRoot, so the rocket is unchanged.
   EXPECT_EQ(r.getTopPart()->typeName(), "HollowSphere");
}

TEST_F(DesignRoundTrip, LeafRootWithNoChildrenRoundTrips)
{
   const std::string tmp = tempFile("leaf");
   {
      model::RocketModel r;
      r.setRoot(bodyTube("Solo"));
      model::DesignSerializer::save(r, tmp);
   }
   model::MotorModelDatabase motors;
   model::RocketModel r2;
   model::DesignSerializer::load(r2, motors, tmp);
   std::filesystem::remove(tmp);

   EXPECT_EQ(r2.getTopPart()->typeName(), "BodyTube");
   EXPECT_EQ(r2.getTopPart()->getName(), "Solo");
   EXPECT_TRUE(r2.getTopPart()->getChildParts().empty());
}

TEST_F(DesignRoundTrip, DeeplyNestedDesignRoundTrips)
{
   // A 4-deep chain nose -> body -> coupler -> inner, exercising recursive build/serialize beyond the
   // 2-level cases.
   model::RocketModel r;
   r.setRoot(std::make_shared<model::part::ConicalNoseCone>("Nose", 0.019, 0.10, 0.0, 2700.0, true));
   const auto noseId = r.getTopPart()->getId();
   auto body = bodyTube("Body");
   const auto bodyId = body->getId();
   ASSERT_TRUE(r.addPart(noseId, body, Vector3{0.0, 0.0, -0.10}));
   auto coupler = std::make_shared<model::part::BodyTube>("Coupler", 0.015, 0.019, 0.03, 900.0);
   const auto couplerId = coupler->getId();
   ASSERT_TRUE(r.addPart(bodyId, coupler, Vector3{0.0, 0.0, -0.10}));
   ASSERT_TRUE(r.addPart(couplerId,
      std::make_shared<model::part::BodyTube>("Inner", 0.010, 0.015, 0.02, 900.0),
      Vector3{0.0, 0.0, -0.02}));

   const std::string tmp = tempFile("deep");
   model::DesignSerializer::save(r, tmp);
   model::MotorModelDatabase motors;
   model::RocketModel r2;
   model::DesignSerializer::load(r2, motors, tmp);
   std::filesystem::remove(tmp);

   EXPECT_DOUBLE_EQ(r2.getMass(0.0), r.getMass(0.0));
   const auto& l1 = std::get<0>(r2.getTopPart()->getChildParts().at(0)); // body
   const auto& l2 = std::get<0>(l1->getChildParts().at(0));              // coupler
   const auto& l3 = std::get<0>(l2->getChildParts().at(0));             // inner
   EXPECT_EQ(l1->getName(), "Body");
   EXPECT_EQ(l2->getName(), "Coupler");
   EXPECT_EQ(l3->getName(), "Inner");
}

TEST_F(DesignRoundTrip, MotorThrustWorksAfterReload)
{
   // The flyability guarantee: after reload the re-attached motor's borrowed handle is valid and
   // produces the same thrust as the original (no dangling, motor re-resolved by name).
   model::MotorModelDatabase motors;
   ASSERT_GT(motors.importRSEFile(std::string(QTROCKET_DATA_DIR) + "/Aerotech.rse"), 0u);
   const auto g80 = motors.getMotorModel("G80T");
   ASSERT_TRUE(g80.has_value());

   const std::string tmp = tempFile("thrust");
   model::RocketModel r;
   r.setRoot(bodyTube("Body"));
   r.setMotorModel(*g80);
   model::DesignSerializer::save(r, tmp);

   model::RocketModel r2;
   model::DesignSerializer::load(r2, motors, tmp);
   std::filesystem::remove(tmp);
   ASSERT_TRUE(r2.isMotorSet());

   r.launch();
   r2.launch();
   EXPECT_NO_THROW((void)r2.getThrust(0.5));
   EXPECT_DOUBLE_EQ(r2.getThrust(0.5), r.getThrust(0.5));
}

// ---- Step 10: <link> serialization, version 0.2 --------------------------------------------------

// The writer stamps the current format version 0.2 (the <link> placement form).
TEST_F(DesignRoundTrip, SaveWritesVersion0_2)
{
   const std::string tmp = tempFile("version");
   {
      model::RocketModel r;
      r.setRoot(bodyTube("Solo"));
      model::DesignSerializer::save(r, tmp);
   }
   const std::string xml = readFile(tmp);
   std::filesystem::remove(tmp);
   EXPECT_NE(xml.find("version=\"0.2\""), std::string::npos) << "writer must stamp version 0.2";
}

// A child attached by the zero-config default link (abut) round-trips: the writer ELIDES the default
// <link>, and the reader's neither-element branch recovers the same abut placement. This exercises both
// the elision (write) and the "0.2 file with no <link> attaches by abut" (read) DoD points together.
TEST_F(DesignRoundTrip, DefaultLinkIsElidedAndReloadsAsAbut)
{
   model::RocketModel r;
   r.setRoot(std::make_shared<model::part::ConicalNoseCone>("Nose", 0.019, 0.10, 0.0, 2700.0, true));
   r.getTopPart()->addChildPart(bodyTube("Body")); // default StationLink{} == abut, equal radii

   const std::string tmp = tempFile("defaultlink");
   model::DesignSerializer::save(r, tmp);
   const std::string xml = readFile(tmp);

   // No <link> element anywhere: the body's link was the default (elided), and the root's link is the
   // default placeholder (also elided).
   EXPECT_EQ(xml.find("<link"), std::string::npos) << "a default-equal link must be elided on write";

   model::MotorModelDatabase motors;
   model::RocketModel r2;
   model::DesignSerializer::load(r2, motors, tmp);
   std::filesystem::remove(tmp);

   // The child still attaches, and its placement matches the original abut -- proven by the composite CG.
   ASSERT_EQ(r2.getTopPart()->getChildParts().size(), 1u);
   EXPECT_EQ(r2.getTopPart()->getChildParts()[0].first->getName(), "Body");
   const Vector3 cg  = r.getTopPart()->getCompositeCm(0.0);
   const Vector3 cg2 = r2.getTopPart()->getCompositeCm(0.0);
   for(int i = 0; i < 3; ++i) { EXPECT_NEAR(cg2(i), cg(i), 1e-9); }
}

// A 0.1 file stores CM-to-CM <offset> (no <link>); it must still load, routing each child through the
// deprecated recovery shim. The presence of <offset> (not its absence) selects the shim branch.
TEST_F(DesignRoundTrip, LegacyOffsetFileLoadsViaShim)
{
   const std::string tmp = tempFile("legacyoffset");
   writeTextFile(tmp,
      "<QtRocketDesign version=\"0.1\">"
      "<design name=\"\"/>"
      "<part type=\"NoseCone\" name=\"Nose\">"
      "<params baseRadius=\"0.019\" length=\"0.10\" wallThickness=\"0\" density=\"2700\" solid=\"true\"/>"
      "<offset x=\"0\" y=\"0\" z=\"0\"/>"
      "<children>"
      "<part type=\"BodyTube\" name=\"Body\">"
      "<params innerRadius=\"0\" outerRadius=\"0.019\" length=\"0.20\" density=\"680\"/>"
      "<offset x=\"0\" y=\"0\" z=\"-0.10\"/><children/></part>"
      "</children></part></QtRocketDesign>");

   model::MotorModelDatabase motors;
   model::RocketModel r;
   model::DesignSerializer::load(r, motors, tmp);
   std::filesystem::remove(tmp);

   ASSERT_EQ(r.getTopPart()->typeName(), "NoseCone");
   ASSERT_EQ(r.getTopPart()->getChildParts().size(), 1u);
   EXPECT_EQ(r.getTopPart()->getChildParts()[0].first->getName(), "Body");
   EXPECT_GT(r.getMass(0.0), 0.0); // shim placed the child; composite mass is real
}

// An unknown seat string is rejected fail-closed with a clear error (not silently defaulted).
TEST_F(DesignRoundTrip, UnknownSeatKindIsRejected)
{
   const std::string tmp = tempFile("badseat");
   writeTextFile(tmp,
      "<QtRocketDesign version=\"0.2\">"
      "<design name=\"\"/>"
      "<part type=\"NoseCone\" name=\"Nose\">"
      "<params baseRadius=\"0.019\" length=\"0.10\" wallThickness=\"0\" density=\"2700\" solid=\"true\"/>"
      "<children>"
      "<part type=\"BodyTube\" name=\"Body\">"
      "<params innerRadius=\"0\" outerRadius=\"0.019\" length=\"0.20\" density=\"680\"/>"
      "<link seat=\"Bogus\" parentStation=\"0\" childStation=\"1\" gap=\"0\"/><children/></part>"
      "</children></part></QtRocketDesign>");

   model::MotorModelDatabase motors;
   model::RocketModel r;
   EXPECT_THROW(model::DesignSerializer::load(r, motors, tmp), std::runtime_error);
   std::filesystem::remove(tmp);
}
