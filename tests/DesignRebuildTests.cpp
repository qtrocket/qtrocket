// The design corpus is CLI-built and physically seated: every committed fixture under
// tests/data/designs/ is the exact product of replaying its build script (designs/scripts/<stem>.cli)
// through the real cli::Repl, and the rebuilt geometry is a contiguous, non-overlapping stack (every
// seam radially compatible, no air gaps, no poke-through). Together these prove the CLI can author
// every SeatKind and that the station-pair seating architecture places the result sensibly.
//
// The one deliberate exception is xl75_multi_pokethrough.qrd -- the frozen self-intersecting offender
// (see PokeThroughRegressionTests) -- which has no build script and is excluded here.
//
// To regenerate the corpus after a deliberate design change: tests/data/designs/regenerate.sh, then
// refresh the placement baseline (see PlacementInvarianceTests.cpp).

/// \cond
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
/// \endcond

#include <gtest/gtest.h>

#include "core/QtRocket.h"
#include "cli/Repl.h"
#include "model/DesignSerializer.h"
#include "model/MotorModelDatabase.h"
#include "model/PartsModel.h"
#include "model/RocketModel.h"
#include "model/parts/PartFactory.h"
#include "model/parts/Placement.h"
#include "utils/Logger.h"

namespace
{
namespace fs = std::filesystem;

const fs::path kTestData    = fs::path(QTROCKET_TEST_DATA_DIR);
const fs::path kDesignsDir  = kTestData / "designs";
const fs::path kScriptsDir  = kDesignsDir / "scripts";
const fs::path kSmallMotors = kTestData / "motors" / "estes_small.qmd";
const fs::path kAerotech    = fs::path(QTROCKET_DATA_DIR) / "Aerotech.rse";

std::string run(cli::Repl& repl, const std::string& cmd)
{
   std::ostringstream out;
   repl.execute(cmd, out);
   return out.str();
}

bool ok(const std::string& s) { return s.find("OK") != std::string::npos; }

// Fixture stems that have a build script / a committed .qrd (pokethrough excluded), sorted.
std::vector<std::string> stems(const fs::path& dir, const std::string& ext)
{
   std::vector<std::string> out;
   for(const auto& e : fs::directory_iterator(dir))
      if(e.is_regular_file() && e.path().extension() == ext
         && e.path().filename().string().find("pokethrough") == std::string::npos)
         out.push_back(e.path().stem().string());
   std::sort(out.begin(), out.end());
   return out;
}

// A design's motor element -- '<motor commonName=".."/>' through any nested <link/> line, or "" if
// none -- read from the raw file text; simpler and more direct than resolving through a motor
// database, and it covers the persisted motor link too.
std::string motorElement(const fs::path& qrd)
{
   std::ifstream in(qrd);
   std::string line;
   std::string out;
   bool inMotor = false;
   while(std::getline(in, line))
   {
      const auto pos = line.find("<motor ");
      if(!inMotor && pos != std::string::npos)
      {
         inMotor = true;
         line = line.substr(pos);
      }
      if(inMotor)
      {
         out += line + "\n";
         if(line.find("/>") != std::string::npos || line.find("</motor>") != std::string::npos)
            break;
      }
   }
   return out;
}

// Field-exact PartParams equality: both trees come from the same serializer, so every double must
// round-trip bit-identically -- no tolerance.
void expectSameParams(const model::part::PartParams& a, const model::part::PartParams& b,
                      const std::string& where)
{
   const auto eq = [&](const std::optional<double>& x, const std::optional<double>& y, const char* k)
   {
      EXPECT_EQ(x.has_value(), y.has_value()) << where << " " << k;
      if(x && y) { EXPECT_EQ(*x, *y) << where << " " << k; }
   };
   EXPECT_EQ(a.name, b.name) << where;
   eq(a.innerRadius, b.innerRadius, "innerRadius");
   eq(a.outerRadius, b.outerRadius, "outerRadius");
   eq(a.baseRadius, b.baseRadius, "baseRadius");
   eq(a.length, b.length, "length");
   eq(a.wallThickness, b.wallThickness, "wallThickness");
   eq(a.density, b.density, "density");
   eq(a.rootChord, b.rootChord, "rootChord");
   eq(a.tipChord, b.tipChord, "tipChord");
   eq(a.span, b.span, "span");
   eq(a.sweep, b.sweep, "sweep");
   eq(a.thickness, b.thickness, "thickness");
   eq(a.bodyRadius, b.bodyRadius, "bodyRadius");
   EXPECT_EQ(a.finCount, b.finCount) << where;
   EXPECT_EQ(a.solid, b.solid) << where;
}

// Recursive structural equality: type, name, geometry params, and the stored StationLink of every
// attachment edge, in order.
void expectSameTree(const model::PartNode& a, const model::PartNode& b, const std::string& where)
{
   EXPECT_EQ(a.part().typeName(), b.part().typeName()) << where;
   EXPECT_EQ(a.part().getName(), b.part().getName()) << where;
   expectSameParams(model::part::params(a.part()), model::part::params(b.part()),
                    where + "/" + a.part().getName());

   const auto ca = a.children();
   const auto cb = b.children();
   ASSERT_EQ(ca.size(), cb.size()) << where << "/" << a.part().getName() << " child count";
   for(std::size_t i = 0; i < ca.size(); ++i)
   {
      const model::part::StationLink& linkA = ca[i]->link();
      const model::part::StationLink& linkB = cb[i]->link();
      const std::string tag =
         where + "/" + a.part().getName() + " link[" + std::to_string(i) + "]";
      EXPECT_EQ(linkA.seat, linkB.seat) << tag;
      EXPECT_EQ(linkA.parentStation01, linkB.parentStation01) << tag;
      EXPECT_EQ(linkA.childStation01, linkB.childStation01) << tag;
      EXPECT_EQ(linkA.gap, linkB.gap) << tag;
      expectSameTree(*ca[i], *cb[i], where + "/" + a.part().getName());
   }
}

// Load a fixture's airframe (empty motor DB -> geometry only). The model owns the node tree, so the
// caller keeps the whole RocketModel alive and reads parts().root().
std::unique_ptr<model::RocketModel> loadAirframe(const fs::path& qrd)
{
   auto rocket = std::make_unique<model::RocketModel>();
   model::MotorModelDatabase motors;
   model::DesignSerializer::load(*rocket, motors, qrd.string());
   return rocket;
}
} // namespace

// Every committed fixture has a build script and vice versa (the corpus and its scripts can't drift).
TEST(DesignRebuild, ScriptsCoverTheCommittedCorpus)
{
   const std::vector<std::string> scripts = stems(kScriptsDir, ".cli");
   const std::vector<std::string> designs = stems(kDesignsDir, ".qrd");
   EXPECT_EQ(scripts, designs);
   EXPECT_EQ(scripts.size(), 24u);
}

// Replaying each build script through the real Repl reproduces the committed fixture exactly: same
// tree (types, names, geometry, stored links), same composite mass/CG/inertia, same baked-in motor.
// Every script line must succeed, including its final `checkdesign` (the CLI's own no-overlap /
// no-air-gap verdict) -- so this is also the end-to-end proof that the CLI authors seated designs.
TEST(DesignRebuild, EveryScriptRebuildsItsCommittedFixture)
{
   utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);
   cli::Repl repl(QtRocket::getInstance());
   ASSERT_TRUE(ok(run(repl, "loaddb " + kSmallMotors.string())));
   ASSERT_TRUE(ok(run(repl, "loadmotors " + kAerotech.string())));

   for(const std::string& stem : stems(kScriptsDir, ".cli"))
   {
      SCOPED_TRACE(stem);

      std::ifstream script(kScriptsDir / (stem + ".cli"));
      ASSERT_TRUE(script.is_open());
      std::string line;
      while(std::getline(script, line))
      {
         if(line.empty() || line[0] == '#')
            continue;
         const std::string out = run(repl, line);
         EXPECT_TRUE(ok(out)) << "script line failed: " << line << "\n" << out;
      }

      const fs::path tmp = fs::temp_directory_path() / ("qtrocket_rebuild_" + stem + ".qrd");
      ASSERT_TRUE(ok(run(repl, "savedesign " + tmp.string())));

      const fs::path committed = kDesignsDir / (stem + ".qrd");
      const std::unique_ptr<model::RocketModel> rebuilt = loadAirframe(tmp);
      const std::unique_ptr<model::RocketModel> fixture = loadAirframe(committed);
      const model::PartNode* rebuiltRoot = rebuilt->parts().root();
      const model::PartNode* fixtureRoot = fixture->parts().root();
      ASSERT_NE(rebuiltRoot, nullptr);
      ASSERT_NE(fixtureRoot, nullptr);
      expectSameTree(*fixtureRoot, *rebuiltRoot, stem);
      EXPECT_EQ(fixtureRoot->compositeMass(0.0), rebuiltRoot->compositeMass(0.0));
      EXPECT_EQ(fixtureRoot->compositeCm(0.0).z(), rebuiltRoot->compositeCm(0.0).z());
      EXPECT_EQ(fixtureRoot->compositeI(0.0), rebuiltRoot->compositeI(0.0));
      EXPECT_EQ(motorElement(committed), motorElement(tmp));

      std::error_code ec;
      fs::remove(tmp, ec);
   }
}

// The physical-sense invariants, asserted on the committed corpus directly (independent of the CLI):
// the placement solve is clean, every seam is radially compatible, and the axial envelope has no air
// gap -- the union of part spans is one contiguous interval.
TEST(DesignRebuild, CommittedCorpusIsContiguousAndSeated)
{
   utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);
   for(const std::string& stem : stems(kDesignsDir, ".qrd"))
   {
      SCOPED_TRACE(stem);
      const std::unique_ptr<model::RocketModel> rocket =
         loadAirframe(kDesignsDir / (stem + ".qrd"));
      const model::PartNode* root = rocket->parts().root();
      ASSERT_NE(root, nullptr);

      EXPECT_TRUE(root->placementDiagnostics().ok);

      // Every stored edge passes the radial seam check for its SeatKind.
      const auto checkSeams = [](const auto& self, const model::PartNode& parent) -> void
      {
         for(const auto& child : parent.children())
         {
            const auto seam =
               model::part::radialSeamCheck(parent.part(), child->part(), child->link());
            EXPECT_FALSE(seam.has_value())
               << parent.part().getName() << " -> " << child->part().getName() << ": "
               << seam->message;
            self(self, *child);
         }
      };
      checkSeams(checkSeams, *root);

      // Axial contiguity: sorted by aft end, each span starts within the coverage so far.
      std::vector<std::pair<double, double>> spans; // (aft, fore)
      for(const model::part::Placed& pl : root->resolvedPlacements())
      {
         const double len = pl.part->axialLength();
         if(len > 0.0)
            spans.emplace_back(pl.pose.origin.z() - len, pl.pose.origin.z());
      }
      ASSERT_FALSE(spans.empty());
      std::sort(spans.begin(), spans.end());
      double covered = spans.front().second;
      for(std::size_t i = 1; i < spans.size(); ++i)
      {
         EXPECT_LE(spans[i].first, covered + 1e-12)
            << "air gap before span starting at z=" << spans[i].first;
         covered = std::max(covered, spans[i].second);
      }
   }
}
