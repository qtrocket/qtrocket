// End-to-end test of the CLI design commands: drive the real Repl through a build -> save ->
// reload -> fly session and confirm the reloaded rocket has the same composite mass/CG and flies the
// same. This is the user-visible proof of the P2 milestone.

/// \cond
#include <cstdio>      // std::remove
#include <filesystem>
#include <optional>
#include <sstream>
#include <string>
/// \endcond

#include <gtest/gtest.h>

#include "QtRocket.h"
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
      "addpart root BodyTube name=Body innerRadius=0 outerRadius=0.019 length=0.20 density=680 z=-0.13");
   ASSERT_TRUE(ok(addBody));
   const auto bodyId = extractId(addBody);
   ASSERT_TRUE(bodyId.has_value());
   ASSERT_TRUE(ok(run(repl, "addpart " + std::to_string(*bodyId) +
      " FinSet name=Fins finCount=3 rootChord=0.10 tipChord=0.05 span=0.05 sweep=0.04"
      " thickness=0.003 bodyRadius=0.019 density=600 z=-0.08")));
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
   std::remove(tmp.c_str());
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
