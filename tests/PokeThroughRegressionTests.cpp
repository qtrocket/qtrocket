// Part-Placement Step 12 / T4: the frozen poke-through offender, exercised END-TO-END through the file
// reader. xl75_multi_pokethrough.qrd carries the whitepaper's edge-defined links (abut body, OnSurface
// fins, NestInBore coupler at 0.04 m insertion), so the resolved coupler projects past the body rim into
// the solid nose and the Layer-2 sweep fires. The production xl75_multi.qrd is clean (its recovered
// CM-to-CM links keep the coupler clear), so without this separate fixture nothing would drive the
// poke-through diagnostic after the cutover. WorkedExampleTests pins the whitepaper geometry; the
// programmatic twin lives in model/tests/ResolverSweepTests.cpp.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "model/DesignSerializer.h"
#include "model/MotorModelDatabase.h"
#include "model/RocketModel.h"
#include "model/parts/Part.h"
#include "model/parts/Placement.h"
#include "utils/Logger.h"

namespace
{
const std::string kPokeThrough =
    std::string(QTROCKET_TEST_DATA_DIR) + "/designs/xl75_multi_pokethrough.qrd";

/// Load the airframe-only tree (empty motor DB: the <motor> is re-attached by name, absent here).
std::shared_ptr<model::part::Part> loadAirframe(const std::string& path)
{
    utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_); // quiet the motor-absent warning
    model::RocketModel        rocket;
    model::MotorModelDatabase motors;
    model::DesignSerializer::load(rocket, motors, path);
    return rocket.getTopPart();
}

const model::part::Placed* find(const std::vector<model::part::Placed>& placed, const std::string& name)
{
    for(const model::part::Placed& p : placed)
    {
        if(p.part->getName() == name) { return &p; }
    }
    return nullptr;
}
} // namespace

TEST(PokeThroughRegressionTests, UnfixedFixtureStillFires)
{
    const std::shared_ptr<model::part::Part> root = loadAirframe(kPokeThrough);

    const model::part::SolveResult& diag = root->placementDiagnostics();
    ASSERT_FALSE(diag.ok) << "the frozen poke-through fixture must still fail the sweep";
    ASSERT_EQ(diag.diagnostics.size(), 1u);

    const std::vector<model::part::Placed> placed = model::part::resolvePlacements(*root, model::part::Pose{});
    const model::part::Placed* coupler = find(placed, "MultiCoupler");
    const model::part::Placed* nose    = find(placed, "MultiNose");
    ASSERT_NE(coupler, nullptr);
    ASSERT_NE(nose, nullptr);

    const model::part::OverlapDiagnostic& d = diag.diagnostics.front();
    EXPECT_EQ(d.offender, coupler->part->getId()); // the coupler pokes ...
    EXPECT_EQ(d.host, nose->part->getId());        // ... through the nose, a non-tree neighbour
    EXPECT_NEAR(d.zWorld, -0.26, 1e-9);
    EXPECT_NEAR(d.penetration, 0.0376 - 0.0395 * (0.26 / 0.30), 1e-9); // ~0.003367 m

    // The composite gate refuses the failed solve rather than returning a silently-wrong inertia.
    EXPECT_THROW((void)root->getCompositeI(0.0), std::runtime_error);
}

TEST(WorkedExampleTests, ResolvedStationsMatchWhitepaper)
{
    // The whitepaper xl75 worked example (Section 6, 10), read end-to-end from the 0.2 fixture: nose span
    // [-0.30, 0]; body abuts to [-1.20, -0.30]; the coupler nests 0.04 m, fore plane at -0.26, span
    // [-0.34, -0.26]; the OnSurface fin seat lands at the body's 0.06 station.
    const std::shared_ptr<model::part::Part> root = loadAirframe(kPokeThrough);
    const std::vector<model::part::Placed>   placed = model::part::resolvePlacements(*root, model::part::Pose{});

    const model::part::Placed* nose    = find(placed, "MultiNose");
    const model::part::Placed* body    = find(placed, "MultiBody");
    const model::part::Placed* coupler = find(placed, "MultiCoupler");
    const model::part::Placed* fins    = find(placed, "MultiFins");
    ASSERT_NE(nose, nullptr);
    ASSERT_NE(body, nullptr);
    ASSERT_NE(coupler, nullptr);
    ASSERT_NE(fins, nullptr);

    EXPECT_NEAR(nose->pose.origin.z(), 0.0, 1e-12);                                  // nose tip at world origin
    EXPECT_NEAR(body->pose.origin.z(), -0.30, 1e-12);                               // body fore abuts nose aft
    EXPECT_NEAR(body->pose.origin.z() - body->part->axialLength(), -1.20, 1e-12);   // body span
    EXPECT_NEAR(coupler->pose.origin.z(), -0.26, 1e-12);                            // nested 0.04 aft of rim
    EXPECT_NEAR(coupler->pose.origin.z() - coupler->part->axialLength(), -0.34, 1e-12);
    EXPECT_NEAR(fins->pose.origin.z(), -1.046, 1e-9);                               // OnSurface 0.06 station
}
