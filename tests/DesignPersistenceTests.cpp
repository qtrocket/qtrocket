// End-to-end tests for model::DesignSerializer: a full design save/load round trip (part-tree mass,
// CG and structure, the reference-area override, and motor-by-name resolution with its seat link),
// the multi-child add_child guarantee, motor-absent tolerance, and rejection of an unsupported file
// version.

/// \cond
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
/// \endcond

#include <gtest/gtest.h>

#include "model/DesignSerializer.h"
#include "model/MotorModelDatabase.h"
#include "model/PartsModel.h"
#include "model/RocketModel.h"
#include "model/parts/Parts.h"
#include "model/tests/PlacementTestSupport.h"
#include "utils/Logger.h"

namespace
{
std::string tempFile(const std::string& tag)
{
    return (std::filesystem::temp_directory_path() / ("qtrocket_design_" + tag + ".qrd")).string();
}

std::unique_ptr<model::part::BodyTube> bodyTube(const std::string& name)
{
    return std::make_unique<model::part::BodyTube>(name, 0.0, 0.019, 0.20, 680.0);
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
    // nose -> body, and body has two children (fins + coupler): a linear chain would not exercise
    // add_child for repeated siblings (the put-vs-add_child trap), so the second child guards it. The
    // cone's non-central CM is exercised by the real attach offsets.
    model::RocketModel r;
    r.installDesign(model::PartNode::make(
        std::make_unique<model::part::ConicalNoseCone>("Nose", 0.019, 0.10, 0.0, 2700.0, true)));
    const auto noseId = r.parts().root()->id();

    auto body = bodyTube("Body");
    const auto bodyLink = model::part::test::cmToCm(r.parts().root()->part(), *body, -0.13);
    const auto bodyId = r.parts().attach(noseId, std::move(body), bodyLink);
    ASSERT_TRUE(bodyId.has_value());
    const model::part::Part& bodyPart = r.parts().find(*bodyId)->part();

    auto fins = std::make_unique<model::part::FinSet>("Fins", 3, 0.10, 0.05, 0.05, 0.04, 0.003, 0.019, 600.0);
    const auto finsLink = model::part::test::cmToCm(bodyPart, *fins, -0.08);
    ASSERT_TRUE(r.parts().attach(*bodyId, std::move(fins), finsLink).has_value());
    auto coupler = std::make_unique<model::part::BodyTube>("Coupler", 0.015, 0.019, 0.03, 900.0);
    const auto couplerLink = model::part::test::cmToCm(bodyPart, *coupler, -0.10);
    ASSERT_TRUE(r.parts().attach(*bodyId, std::move(coupler), couplerLink).has_value());

    const std::string tmp = tempFile("geom");
    model::DesignSerializer::save(r, tmp);

    model::MotorModelDatabase motors; // this design has no motor
    model::RocketModel r2;
    model::DesignSerializer::load(r2, motors, tmp);
    std::filesystem::remove(tmp);

    // Structure: nose root -> one child (body); body -> two children (both survived: add_child, not put).
    ASSERT_NE(r2.parts().root(), nullptr);
    EXPECT_EQ(r2.parts().root()->part().typeName(), "NoseCone");
    ASSERT_EQ(r2.parts().root()->children().size(), 1u);
    const model::PartNode& body2 = *r2.parts().root()->children()[0];
    EXPECT_EQ(body2.part().typeName(), "BodyTube");
    EXPECT_EQ(body2.part().getName(), "Body");
    ASSERT_EQ(body2.children().size(), 2u);

    // Mass exact; CG via EXPECT_NEAR (reload may re-sum children in a different ULP order). The CG
    // match is what proves the placement (the body's StationLink) round-tripped faithfully -- only the
    // link intent is stored; the resolver derives the absolute offset.
    EXPECT_DOUBLE_EQ(r2.getMass(0.0), r.getMass(0.0));
    const Vector3 cg = r.parts().root()->compositeCm(0.0);
    const Vector3 cg2 = r2.parts().root()->compositeCm(0.0);
    for(int i = 0; i < 3; ++i) { EXPECT_NEAR(cg2(i), cg(i), 1e-9); }
}

TEST_F(DesignRoundTrip, ReferenceAreaOverrideAndDragRoundTrip)
{
    const std::string tmp = tempFile("refarea");
    {
        model::RocketModel r;
        r.installDesign(model::PartNode::make(bodyTube("Body")));
        r.setReferenceArea(0.0421); // manual override on (after the install, which resets it)
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
        model::RocketModel r; // reference area at its default, not overridden
        r.installDesign(model::PartNode::make(bodyTube("Body")));
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
    r.installDesign(model::PartNode::make(bodyTube("Body")));
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
    r.installDesign(model::PartNode::make(bodyTube("Body")));
    r.setMotorModel(*g80);
    model::DesignSerializer::save(r, tmp);

    model::MotorModelDatabase empty; // does not contain G80T
    model::RocketModel r2;
    model::DesignSerializer::load(r2, empty, tmp); // logs a warning, loads geometry only
    std::filesystem::remove(tmp);

    EXPECT_FALSE(r2.isMotorSet());
    EXPECT_EQ(r2.parts().root()->part().typeName(), "BodyTube"); // geometry still loaded
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
    r.installDesign(model::PartNode::make(
        std::make_unique<model::part::HollowSphere>("Body", 0.04, 0.05, 1956.8)));
    EXPECT_THROW(model::DesignSerializer::load(r, motors, tmp), std::exception);
    std::filesystem::remove(tmp);

    // The version check happens before installDesign, so the rocket is unchanged.
    EXPECT_EQ(r.parts().root()->part().typeName(), "HollowSphere");
}

TEST_F(DesignRoundTrip, LeafRootWithNoChildrenRoundTrips)
{
    const std::string tmp = tempFile("leaf");
    {
        model::RocketModel r;
        r.installDesign(model::PartNode::make(bodyTube("Solo")));
        model::DesignSerializer::save(r, tmp);
    }
    model::MotorModelDatabase motors;
    model::RocketModel r2;
    model::DesignSerializer::load(r2, motors, tmp);
    std::filesystem::remove(tmp);

    EXPECT_EQ(r2.parts().root()->part().typeName(), "BodyTube");
    EXPECT_EQ(r2.parts().root()->part().getName(), "Solo");
    EXPECT_TRUE(r2.parts().root()->children().empty());
}

TEST_F(DesignRoundTrip, DeeplyNestedDesignRoundTrips)
{
    // A 4-deep chain nose -> body -> coupler -> inner, exercising recursive build/serialize beyond the
    // 2-level cases.
    model::RocketModel r;
    r.installDesign(model::PartNode::make(
        std::make_unique<model::part::ConicalNoseCone>("Nose", 0.019, 0.10, 0.0, 2700.0, true)));
    const auto noseId = r.parts().root()->id();
    const auto bodyId = r.parts().attach(noseId, bodyTube("Body"), model::part::abut(-0.10));
    ASSERT_TRUE(bodyId.has_value());
    const auto couplerId = r.parts().attach(*bodyId,
        std::make_unique<model::part::BodyTube>("Coupler", 0.015, 0.019, 0.03, 900.0),
        model::part::abut(-0.10));
    ASSERT_TRUE(couplerId.has_value());
    ASSERT_TRUE(r.parts().attach(*couplerId,
        std::make_unique<model::part::BodyTube>("Inner", 0.010, 0.015, 0.02, 900.0),
        model::part::abut(-0.02)).has_value());

    const std::string tmp = tempFile("deep");
    model::DesignSerializer::save(r, tmp);
    model::MotorModelDatabase motors;
    model::RocketModel r2;
    model::DesignSerializer::load(r2, motors, tmp);
    std::filesystem::remove(tmp);

    EXPECT_DOUBLE_EQ(r2.getMass(0.0), r.getMass(0.0));
    ASSERT_EQ(r2.parts().root()->children().size(), 1u);
    const model::PartNode& l1 = *r2.parts().root()->children()[0]; // body
    ASSERT_EQ(l1.children().size(), 1u);
    const model::PartNode& l2 = *l1.children()[0];                 // coupler
    ASSERT_EQ(l2.children().size(), 1u);
    const model::PartNode& l3 = *l2.children()[0];                 // inner
    EXPECT_EQ(l1.part().getName(), "Body");
    EXPECT_EQ(l2.part().getName(), "Coupler");
    EXPECT_EQ(l3.part().getName(), "Inner");
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
    r.installDesign(model::PartNode::make(bodyTube("Body")));
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

// ---- <link> serialization, version 0.2 -----------------------------------------------------------

// The writer stamps the current format version 0.2 (the <link> placement form).
TEST_F(DesignRoundTrip, SaveWritesVersion0_2)
{
    const std::string tmp = tempFile("version");
    {
        model::RocketModel r;
        r.installDesign(model::PartNode::make(bodyTube("Solo")));
        model::DesignSerializer::save(r, tmp);
    }
    const std::string xml = readFile(tmp);
    std::filesystem::remove(tmp);
    EXPECT_NE(xml.find("version=\"0.2\""), std::string::npos) << "writer must stamp version 0.2";
}

// A child attached by the zero-config default link (abut) round-trips: the writer elides the default
// <link>, and the reader's neither-element branch recovers the same abut placement. This exercises both
// the elision (write) and the "0.2 file with no <link> attaches by abut" (read) DoD points together.
TEST_F(DesignRoundTrip, DefaultLinkIsElidedAndReloadsAsAbut)
{
    model::RocketModel r;
    r.installDesign(model::PartNode::make(
        std::make_unique<model::part::ConicalNoseCone>("Nose", 0.019, 0.10, 0.0, 2700.0, true)));
    // default StationLink{} == abut, equal radii
    ASSERT_TRUE(r.parts().attach(r.parts().root()->id(), bodyTube("Body")).has_value());

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
    ASSERT_EQ(r2.parts().root()->children().size(), 1u);
    EXPECT_EQ(r2.parts().root()->children()[0]->part().getName(), "Body");
    const Vector3 cg  = r.parts().root()->compositeCm(0.0);
    const Vector3 cg2 = r2.parts().root()->compositeCm(0.0);
    for(int i = 0; i < 3; ++i) { EXPECT_NEAR(cg2(i), cg(i), 1e-9); }
}

// A legacy 0.1 file stores CM-to-CM <offset> placements. With the shim retired (0.2-only), such a file
// is rejected with a clear error rather than silently mis-placed.
TEST_F(DesignRoundTrip, LegacyOffsetFileIsRejected)
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
    EXPECT_THROW(model::DesignSerializer::load(r, motors, tmp), std::runtime_error);
    std::filesystem::remove(tmp);
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

// All three distinct seat kinds (Abut / NestInBore / OnSurface) with non-default station and gap fields
// round-trip exactly: each reloaded StationLink equals the original (seat, both stations, gap). childRot
// is not persisted in 0.2 (identity in 3-DOF), so it is not compared. This is the only direct check of
// non-default link fidelity -- the other round-trip tests use the default abut link and verify placement
// only indirectly via the composite CG. (No composite is computed here: purely serialization fidelity.)
TEST_F(DesignRoundTrip, LinkRoundTrips)
{
    using model::part::SeatKind;
    using model::part::StationLink;

    const StationLink lAbut{.parentStation01 = 0.0,  .childStation01 = 1.0, .gap = 0.012, .seat = SeatKind::Abut};
    const StationLink lNest{.parentStation01 = 1.0,  .childStation01 = 0.0, .gap = 0.04,  .seat = SeatKind::NestInBore};
    const StationLink lSurf{.parentStation01 = 0.30, .childStation01 = 0.0, .gap = 0.0,   .seat = SeatKind::OnSurface};

    auto root = model::PartNode::make(
        std::make_unique<model::part::BodyTube>("Root", 0.0376, 0.0395, 0.50, 700.0));
    root->addChild(model::PartNode::make(
        std::make_unique<model::part::BodyTube>("Abutted", 0.0376, 0.0395, 0.10, 700.0)), lAbut);
    root->addChild(model::PartNode::make(
        std::make_unique<model::part::BodyTube>("Nested", 0.030, 0.0376, 0.08, 700.0)), lNest);
    root->addChild(model::PartNode::make(
        std::make_unique<model::part::FinSet>("Surfaced", 3, 0.05, 0.02, 0.03, 0.02, 0.003, 0.0395, 600.0)), lSurf);

    model::RocketModel r;
    r.installDesign(std::move(root));

    const std::string tmp = tempFile("links");
    model::DesignSerializer::save(r, tmp);
    model::MotorModelDatabase motors;
    model::RocketModel r2;
    model::DesignSerializer::load(r2, motors, tmp);
    std::filesystem::remove(tmp);

    // Index reloaded children by name so the comparison is order-independent.
    std::map<std::string, StationLink> got;
    for(const auto& child : r2.parts().root()->children())
    {
        got[child->part().getName()] = child->link();
    }
    ASSERT_EQ(got.size(), 3u);

    const auto expectLink = [&](const std::string& name, const StationLink& exp)
    {
        ASSERT_TRUE(got.count(name) == 1u) << "missing reloaded child '" << name << "'";
        const StationLink& a = got[name];
        EXPECT_EQ(a.seat, exp.seat) << name << " seat";
        EXPECT_DOUBLE_EQ(a.parentStation01, exp.parentStation01) << name << " parentStation";
        EXPECT_DOUBLE_EQ(a.childStation01, exp.childStation01) << name << " childStation";
        EXPECT_DOUBLE_EQ(a.gap, exp.gap) << name << " gap";
    };
    expectLink("Abutted", lAbut);
    expectLink("Nested", lNest);
    expectLink("Surfaced", lSurf);
}

// The motor's seat is persisted like any other edge: a default link (root aft plane) is elided under
// <motor>, and the reader recovers the default on reload.
TEST_F(DesignRoundTrip, MotorDefaultLinkIsElided)
{
    model::MotorModelDatabase motors;
    ASSERT_GT(motors.importRSEFile(std::string(QTROCKET_DATA_DIR) + "/Aerotech.rse"), 0u);
    const auto g80 = motors.getMotorModel("G80T");
    ASSERT_TRUE(g80.has_value());

    const std::string tmp = tempFile("motorlinkdefault");
    model::RocketModel r;
    r.installDesign(model::PartNode::make(bodyTube("Body")));
    r.setMotorModel(*g80); // default seat
    model::DesignSerializer::save(r, tmp);

    // No <link> anywhere: the root's placeholder link and the motor's default seat are both elided.
    const std::string xml = readFile(tmp);
    EXPECT_NE(xml.find("<motor"), std::string::npos);
    EXPECT_EQ(xml.find("<link"), std::string::npos) << "a default motor link must be elided on write";

    model::RocketModel r2;
    model::DesignSerializer::load(r2, motors, tmp);
    std::filesystem::remove(tmp);

    ASSERT_TRUE(r2.isMotorSet());
    const model::PartNode* mn = r2.parts().motorNode();
    ASSERT_NE(mn, nullptr);
    const model::part::StationLink d{};
    EXPECT_EQ(mn->link().seat, d.seat);
    EXPECT_DOUBLE_EQ(mn->link().parentStation01, d.parentStation01);
    EXPECT_DOUBLE_EQ(mn->link().childStation01, d.childStation01);
    EXPECT_DOUBLE_EQ(mn->link().gap, d.gap);
}

// A non-default motor seat round-trips exactly through the <motor><link> element: the reloaded motor
// node carries the authored seat, stations, and gap.
TEST_F(DesignRoundTrip, MotorNonDefaultLinkRoundTrips)
{
    using model::part::SeatKind;
    using model::part::StationLink;

    model::MotorModelDatabase motors;
    ASSERT_GT(motors.importRSEFile(std::string(QTROCKET_DATA_DIR) + "/Aerotech.rse"), 0u);
    const auto g80 = motors.getMotorModel("G80T");
    ASSERT_TRUE(g80.has_value());

    // motor nested part-way up the body bore; every persisted field is off-default
    const StationLink motorLink{.parentStation01 = 0.25, .childStation01 = 0.0, .gap = 0.03,
                                .seat = SeatKind::NestInBore};

    const std::string tmp = tempFile("motorlink");
    model::RocketModel r;
    r.installDesign(model::PartNode::make(bodyTube("Body")));
    ASSERT_TRUE(r.parts().setMotor(*g80, motorLink)); // first install seats by the given link
    model::DesignSerializer::save(r, tmp);

    const std::string xml = readFile(tmp);
    EXPECT_NE(xml.find("<motor"), std::string::npos);
    EXPECT_NE(xml.find("<link"), std::string::npos) << "a non-default motor link must be written";

    model::RocketModel r2;
    model::DesignSerializer::load(r2, motors, tmp);
    std::filesystem::remove(tmp);

    ASSERT_TRUE(r2.isMotorSet());
    const model::PartNode* mn = r2.parts().motorNode();
    ASSERT_NE(mn, nullptr);
    EXPECT_EQ(mn->link().seat, motorLink.seat);
    EXPECT_DOUBLE_EQ(mn->link().parentStation01, motorLink.parentStation01);
    EXPECT_DOUBLE_EQ(mn->link().childStation01, motorLink.childStation01);
    EXPECT_DOUBLE_EQ(mn->link().gap, motorLink.gap);
}
