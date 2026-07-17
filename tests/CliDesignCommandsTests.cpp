// End-to-end test of the CLI design commands: drive the real Repl through a build -> save ->
// reload -> fly session and confirm the reloaded rocket has the same composite mass/CG and flies the
// same.

/// \cond
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
/// \endcond

#include <gtest/gtest.h>

#include "core/QtRocket.h"
#include "cli/Repl.h"
#include "utils/Logger.h"

namespace
{
// Execute one REPL command and capture its output.
std::string run(cli::Repl& repl, const std::string& cmd)
{
    std::ostringstream out;
    repl.execute(cmd, out);
    return out.str();
}

bool ok(const std::string& s) { return s.find("OK") != std::string::npos; }

void writeTextFile(const std::filesystem::path& path, const std::string& text)
{
    std::ofstream file(path);
    ASSERT_TRUE(file.is_open());
    file << text;
    ASSERT_TRUE(file.good());
}

// The first line of @p out containing @p needle (or "" if none).
std::string lineWith(const std::string& out, const std::string& needle)
{
    std::istringstream iss(out);
    std::string line;
    while(std::getline(iss, line))
    {
        if(line.find(needle) != std::string::npos)
            return line;
    }
    return "";
}

// The numeric id from an "... id=N ..." line.
std::optional<unsigned long long> extractId(const std::string& s)
{
    const auto p = s.find("id=");
    if(p == std::string::npos)
        return std::nullopt;
    try { return std::stoull(s.substr(p + 3)); }
    catch(const std::exception&) { return std::nullopt; }
}
} // namespace

TEST(CliDesignCommands, BuildSaveReloadFlyEndToEnd)
{
    utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);
    QtRocket* qt = QtRocket::getInstance();
    cli::Repl repl(qt);

    // Motors must be loaded for setmotor and for loaddesign's motor-by-name resolution.
    ASSERT_TRUE(ok(run(repl, std::string("loadmotors ") + QTROCKET_DATA_DIR + "/Aerotech.rse")));

   // Build a multi-part rocket: nose root, body under root, fins under the body (exercises the
   // non-root parent-id path and the cone's non-central CM offset).
   ASSERT_TRUE(ok(run(repl, "newdesign NoseCone name=Nose baseRadius=0.019 length=0.10 density=2700")));
   const std::string addBody = run(repl,
      "addpart root BodyTube name=Body innerRadius=0 outerRadius=0.019 length=0.20 density=680");
   ASSERT_TRUE(ok(addBody));
   const auto bodyId = extractId(addBody);
   ASSERT_TRUE(bodyId.has_value());
   ASSERT_TRUE(ok(run(repl, "addpart " + std::to_string(*bodyId) +
      " FinSet name=Fins finCount=3 rootChord=0.10 tipChord=0.05 span=0.05 sweep=0.04"
      " thickness=0.003 bodyRadius=0.019 density=600 seat=OnSurface parentStation=0 childStation=0")));
   ASSERT_TRUE(ok(run(repl, "setmotor G80T")));

    // Snapshot the composite mass/CG footer and a baseline flight.
    const std::string footerBefore = lineWith(run(repl, "listparts"), "composite:");
    ASSERT_FALSE(footerBefore.empty());
    const std::string launchBefore = run(repl, "launch");
    ASSERT_TRUE(ok(launchBefore));
    const std::string apogeeBefore = lineWith(launchBefore, "apogee");
    ASSERT_FALSE(apogeeBefore.empty());

    // Save, wipe the design, reload it.
    const std::string tmp = (std::filesystem::temp_directory_path() / "qtrocket_cli_e2e.qrd").string();
    ASSERT_TRUE(ok(run(repl, "savedesign " + tmp)));
    ASSERT_TRUE(ok(run(repl, "cleardesign")));
    const std::string loadOut = run(repl, "loaddesign " + tmp);
    std::filesystem::remove(tmp);
    ASSERT_TRUE(ok(loadOut));
    EXPECT_NE(loadOut.find("G80T"), std::string::npos); // motor re-resolved by name

    // The reloaded design has byte-identical composite mass/CG, and flies to the same apogee.
    EXPECT_EQ(lineWith(run(repl, "listparts"), "composite:"), footerBefore);
    const std::string launchAfter = run(repl, "launch");
    ASSERT_TRUE(ok(launchAfter));
    EXPECT_EQ(lineWith(launchAfter, "apogee"), apogeeBefore);

    std::error_code ec;
    std::filesystem::remove(std::filesystem::absolute("qtrocket_run.csv", ec), ec); // launch's CSV side effect
}

TEST(CliDesignCommands, ParsingValidatesInputAndToleratesUnknownKeys)
{
    utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);
    cli::Repl repl(QtRocket::getInstance());

    // Trailing garbage on a numeric value is rejected (not silently truncated to 0.019).
    EXPECT_FALSE(ok(run(repl, "newdesign BodyTube outerRadius=0.019abc length=0.2 density=680")));
    // An unknown key is tolerated (warn + continue): the part still builds.
    EXPECT_TRUE(ok(run(repl, "newdesign BodyTube outerRadius=0.019 length=0.2 density=680 wibble=3")));
    // A bad parent id is rejected.
    EXPECT_FALSE(ok(run(repl, "addpart 9zz BodyTube outerRadius=0.019 length=0.2 density=680")));
    // An absent numeric parent id reports the typed attach error verbatim.
    const std::string absentParent =
        run(repl, "addpart 424242 BodyTube outerRadius=0.019 length=0.2 density=680");
    EXPECT_NE(absentParent.find("ERR addpart: no part with id 424242"), std::string::npos)
        << absentParent;
    // An unknown part type is rejected (by the factory).
    EXPECT_FALSE(ok(run(repl, "newdesign Wibble outerRadius=0.019")));
    // A missing required field is rejected (by the factory).
    EXPECT_FALSE(ok(run(repl, "newdesign BodyTube length=0.2 density=680")));

    // removepart refuses the root and an absent id.
    const std::string nd = run(repl, "newdesign BodyTube outerRadius=0.019 length=0.2 density=680");
    ASSERT_TRUE(ok(nd));
    const auto rootId = extractId(nd);
    ASSERT_TRUE(rootId.has_value());
    EXPECT_FALSE(ok(run(repl, "removepart " + std::to_string(*rootId))));
    EXPECT_FALSE(ok(run(repl, "removepart 999999")));
}

// addpart's link vocabulary (seat / parentStation / childStation / gap) authors every SeatKind, the
// parent can be addressed by name (ids are session-specific, so scripts must not hardcode them), and
// the authored links survive a save/reload round trip.
TEST(CliDesignCommands, AddpartAuthorsSeatVocabularyAndRoundTrips)
{
   utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);
   cli::Repl repl(QtRocket::getInstance());

   ASSERT_TRUE(ok(run(repl, "newdesign NoseCone name=SeatNose baseRadius=0.02 length=0.12 density=1400")));
   ASSERT_TRUE(ok(run(repl, "addpart SeatNose BodyTube name=SeatBody innerRadius=0.0191 outerRadius=0.02"
                            " length=0.40 density=1400"))); // default: abut, gap 0
   ASSERT_TRUE(ok(run(repl, "addpart SeatBody FinSet name=SeatFins finCount=3 rootChord=0.07 tipChord=0.03"
                            " span=0.05 sweep=0.04 thickness=0.003 bodyRadius=0.02 density=1400"
                            " seat=OnSurface parentStation=0 childStation=0")));
   ASSERT_TRUE(ok(run(repl, "addpart SeatBody BodyTube name=SeatCoupler innerRadius=0.018 outerRadius=0.019"
                            " length=0.05 density=1400 seat=NestInBore gap=0.05")));
   ASSERT_TRUE(ok(run(repl, "checkdesign")));

   const std::string tmp = (std::filesystem::temp_directory_path() / "qtrocket_cli_seats.qrd").string();
   ASSERT_TRUE(ok(run(repl, "savedesign " + tmp)));

   // The authored intent is in the file: explicit seats written, the default abut elided.
   std::ifstream in(tmp);
   std::stringstream ss;
   ss << in.rdbuf();
   in.close(); // windows can't delete a file with an open handle; remove(tmp) below needs it closed
   const std::string qrd = ss.str();
   EXPECT_NE(qrd.find("seat=\"OnSurface\""), std::string::npos) << qrd;
   EXPECT_NE(qrd.find("seat=\"NestInBore\""), std::string::npos) << qrd;
   EXPECT_EQ(qrd.find("seat=\"Abut\""), std::string::npos) << "default abut link should be elided:\n" << qrd;

   const std::string footerBefore = lineWith(run(repl, "listparts"), "composite:");
   ASSERT_TRUE(ok(run(repl, "cleardesign")));
   ASSERT_TRUE(ok(run(repl, "loaddesign " + tmp)));
   std::filesystem::remove(tmp);
   EXPECT_TRUE(ok(run(repl, "checkdesign")));
   EXPECT_EQ(lineWith(run(repl, "listparts"), "composite:"), footerBefore);
}

TEST(CliDesignCommands, AddpartRejectsBadLinkTokens)
{
   utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);
   cli::Repl repl(QtRocket::getInstance());

   ASSERT_TRUE(ok(run(repl, "newdesign BodyTube name=Root outerRadius=0.02 length=0.4 density=1400")));
   const std::string tube = " BodyTube outerRadius=0.02 length=0.1 density=1400";
   // Unknown seat kind.
   EXPECT_FALSE(ok(run(repl, "addpart Root" + tube + " seat=Wibble")));
   // NestInBore depth must be non-negative.
   EXPECT_FALSE(ok(run(repl, "addpart Root" + tube + " seat=NestInBore gap=-0.01")));
   // Unknown parent name.
   EXPECT_FALSE(ok(run(repl, "addpart Nowhere" + tube)));
}

// checkdesign is the CLI's physical-sense verdict: OK on a contiguous non-overlapping stack, ERR with
// located diagnostics for an overlap (poke-through) or an air gap. A failed placement solve must
// degrade composite reads (listparts) to an ERR line, never kill the session.
TEST(CliDesignCommands, CheckdesignReportsOverlapsAndAirGaps)
{
   utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);
   cli::Repl repl(QtRocket::getInstance());

   // The whitepaper poke-through: a coupler nested only 0.04 m into the bore projects past the body
   // rim into the solid nose.
   ASSERT_TRUE(ok(run(repl, "newdesign NoseCone name=PokeNose baseRadius=0.0395 length=0.30 density=1700")));
   ASSERT_TRUE(ok(run(repl, "addpart PokeNose BodyTube name=PokeBody innerRadius=0.0376 outerRadius=0.0395"
                            " length=0.90 density=1700")));
   ASSERT_TRUE(ok(run(repl, "addpart PokeBody BodyTube name=PokeCoupler innerRadius=0.036 outerRadius=0.0376"
                            " length=0.08 density=1700 seat=NestInBore gap=0.04")));
   const std::string overlapOut = run(repl, "checkdesign");
   EXPECT_FALSE(ok(overlapOut));
   EXPECT_NE(overlapOut.find("overlap"), std::string::npos) << overlapOut;

   // The composite gate refuses the failed solve; the CLI reports it instead of crashing.
   const std::string partsOut = run(repl, "listparts");
   EXPECT_NE(partsOut.find("ERR"), std::string::npos) << partsOut;
   EXPECT_NE(partsOut.find("placement solve failed"), std::string::npos) << partsOut;

   // An air gap (the pre-seating fixture defect): a negative abut standoff leaves the body floating
   // below the nose.
   ASSERT_TRUE(ok(run(repl, "newdesign NoseCone name=GapNose baseRadius=0.02 length=0.12 density=1400")));
   ASSERT_TRUE(ok(run(repl, "addpart GapNose BodyTube name=GapBody innerRadius=0.0191 outerRadius=0.02"
                            " length=0.40 density=1400 gap=-0.03")));
   const std::string gapOut = run(repl, "checkdesign");
   EXPECT_FALSE(ok(gapOut));
   EXPECT_NE(gapOut.find("air gap"), std::string::npos) << gapOut;
}

TEST(CliDesignCommands, LoadMotorsImportsRaspEngFiles)
{
    utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);
    cli::Repl repl(QtRocket::getInstance());

    const auto fixture = std::filesystem::temp_directory_path() / "qtrocket_cli_rasp.eng";
    ASSERT_NO_FATAL_FAILURE(writeTextFile(fixture, R"(
1/2A3 13 45 P 0.001 0.002 Estes
  0.25 2.0
  0.50 0.0
;
)"));

    const std::string loadOut = run(repl, "loadmotors " + fixture.string());
    std::filesystem::remove(fixture);
    ASSERT_TRUE(ok(loadOut));
    EXPECT_NE(loadOut.find("1 motors"), std::string::npos);
    EXPECT_TRUE(ok(run(repl, "setmotor 1/2A3")));
}

TEST(CliDesignCommands, RunExitCodeReflectsCommandFailures)
{
    // The exit-code contract that makes piped scripts honest for CI consumers: run() returns 0
    // only if every command succeeded, 1 if any reported ERR -- even though the session itself
    // recovers and keeps executing.
    {
        cli::Repl repl(QtRocket::getInstance());
        std::istringstream in("# comment\nstatus\nquit\n");
        std::ostringstream out;
        EXPECT_EQ(repl.run(in, out), 0) << out.str();
    }
    {
        cli::Repl repl(QtRocket::getInstance());
        std::istringstream in("bogus\nstatus\nquit\n");
        std::ostringstream out;
        EXPECT_EQ(repl.run(in, out), 1) << out.str();
        EXPECT_NE(out.str().find("ERR unknown command"), std::string::npos) << out.str();
        EXPECT_NE(out.str().find("OK"), std::string::npos) << "session should continue after an ERR";
    }
}
